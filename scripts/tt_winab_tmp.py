"""0x187 bit1 A/B：同一版固件里比较 连续PWM(bit1=0) 与 窗口DMA引擎(bit1=1) 的落点。

  python scripts/tt_winab_tmp.py            默认按网页"半圈"0x126 各 5 趟
  python scripts/tt_winab_tmp.py 3200 8     改成定步数 3200、各 8 趟

用板上当前 0x11A/0x11B/0x11C，只改 0x187 和 0x107=0（纯开环，让"沿数==账本"成立）。
跑完把 0x187 归 0 并 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPR = 6400
R_CLOSED, R_STATE, R_TUNING = 0x107, 0x162, 0x187
R_PULSES, R_MODE, R_MOTION = 0x11D, 0x11E, 0x11F
R_ANG, R_FE, R_LEDGER = 0x116, 0x121, 0x134
R_HALF, R_MAXWIN, R_CREEP = 0x126, 0x177, 0x183


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


mode = 'half'
steps = SPR // 2
reps = 5
if len(sys.argv) > 1:
    mode = 'steps'
    steps = int(sys.argv[1])
    reps = int(sys.argv[2]) if len(sys.argv) > 2 else 5

h = TTHost('COM9')
if h.read(0x16D) != 0:
    print('0x16D 外部 → 强制内部')
    h.write(0x16D, 0)
    time.sleep(0.2)
saved = {r: h.read(r) for r in (R_CLOSED, R_TUNING)}
print('板上当前参数：', {hex(k): v for k, v in saved.items()},
      f'start={h.read(0x11A)} max={h.read(0x11B)} acc={h.read(0x11C)}')
h.write(0x124, 1)
time.sleep(1.0)
h.write(0x107, 0)
try:
    for tuning, label in ((0, '连续 PWM  (bit1=0)'), (2, '窗口 DMA  (bit1=1)')):
        h.write(R_TUNING, tuning)
        time.sleep(0.1)
        got = h.read(R_TUNING)
        if got != tuning:
            print(f'{label} 写入失败：读回 {got}（bit1 可能被固件拒绝）')
            continue
        biases = []
        for i in range(reps):
            while h.read(R_MOTION):
                time.sleep(0.05)
            time.sleep(0.25)
            if mode == 'steps':
                h.write(R_PULSES, steps)
                h.write(R_MODE, 0)
                if h.read(R_PULSES) != steps:
                    raise RuntimeError(f'0x11D 丢写 {h.read(R_PULSES)}')
            prev = h.read(R_ANG)
            acc = 0.0
            seen = False
            t_stop = None
            t0 = time.time()
            h.write(R_HALF, 1) if mode == 'half' else h.write(R_MOTION, 1)
            while time.time() - t0 < 20:
                a = h.read(R_ANG)
                if a is not None:
                    acc += unwrap(a - prev)
                    prev = a
                r = h.latest.get(R_MOTION)
                if r == 1:
                    seen = True
                elif seen and t_stop is None:
                    t_stop = time.time() - t0
                    break
            if not seen:
                raise RuntimeError('未进入运动（0x11F 从未置起）')
            time.sleep(0.3)
            b = acc / 56.25 - steps
            biases.append(b)
            print(f'{label} 第{i + 1}趟 耗时 {(t_stop or -1):6.3f}s  '
                  f'实走 {acc / 56.25:9.1f} 偏 {b:+6.2f} 微步({b * 0.05625:+.4f}°)  '
                  f'账本 {h.read(R_LEDGER)}  FE={(s32(h.read(R_FE)) or 0) / 1000.0:+.3f}°  '
                  f'0x177={h.read(R_MAXWIN)} 0x183={h.read(R_CREEP)} '
                  f'0x185={h.read(0x185)} 0x186={h.read(0x186)}')
        m = sum(biases) / len(biases)
        spread = max(biases) - min(biases)
        print(f'>>> {label} 平均 {m:+6.2f} 微步 ({m * 0.05625:+.4f}°)  '
              f'散布(最大-最小) {spread:5.2f} 微步 ({spread * 0.05625:.4f}°)\n')
finally:
    h.write(R_TUNING, 0)
    for reg, val in saved.items():
        if reg != R_TUNING and val is not None:
            h.write(reg, val)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'0x187 已归 0，已失能 0x162={h.read(R_STATE)}')
