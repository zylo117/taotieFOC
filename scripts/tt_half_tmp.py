"""原样复现网页"半圈正向脉冲"（0x126=1），量多走，再扫参数看它随什么变。

  python scripts/tt_half_tmp.py            用板上当前参数按 5 次 0x126
  python scripts/tt_half_tmp.py scan       再按 max=60/120/250 RPM 各扫一组（跑完还原参数）

只做开环（0x107=0）。每趟量编码器解绕累计的"实走 − 3200 微步"。
跑完 0x125 失能；写过的参数一律还原。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_HALF = 0x126
R_ANG, R_MOTION, R_FE = 0x116, 0x11F, 0x121
R_START, R_MAX, R_ACC = 0x11A, 0x11B, 0x11C
R_CLOSED, R_STATE, R_REINIT, R_TIER = 0x107, 0x162, 0x185, 0x186
STEP_UM = 0.05625
SPR = 6400


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
if h.read(0x16D) != 0:
    print('0x16D 外部 → 强制内部')
    h.write(0x16D, 0)
    time.sleep(0.2)
saved = {r: h.read(r) for r in (R_CLOSED, R_START, R_MAX, R_ACC)}
print('板上当前参数：', {hex(k): v for k, v in saved.items()})
h.write(0x124, 1)
time.sleep(0.8)
h.write(0x107, 0)


def press(label, reps=5):
    biases = []
    for i in range(reps):
        while h.read(R_MOTION):
            time.sleep(0.05)
        time.sleep(0.3)
        prev = h.read(R_ANG)
        acc = 0
        seen = False
        t_stop = None
        t0 = time.time()
        h.write(R_HALF, 1)
        while time.time() - t0 < 15:
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
            raise RuntimeError('0x126 没触发运动')
        time.sleep(0.35)
        bias = acc / 56.25 - SPR / 2
        fe = (s32(h.read(R_FE)) or 0) / 1000.0
        biases.append(bias)
        print(f'{label} 第{i + 1}次 耗时 {(t_stop or -1):5.3f}s  实走 {acc / 56.25:8.1f} 微步  '
              f'偏 {bias:+6.2f} 微步({bias * STEP_UM:+.4f}°)  FE={fe:+.3f}°  '
              f'0x185={h.read(R_REINIT)} 0x186={h.read(R_TIER)}')
    m = sum(biases) / len(biases)
    print(f'{label} 平均偏 {m:+6.2f} 微步 ({m * STEP_UM:+.4f}°)  '
          f'散布 {min(biases):+.2f}~{max(biases):+.2f}')
    return m


try:
    press(f'当前参数(start={saved[R_START]}/max={saved[R_MAX]}/acc={saved[R_ACC]})')
    if 'scan' in sys.argv:
        for mx in (60, 120, 250):
            h.write(R_MAX, mx)
            time.sleep(0.1)
            press(f'max={mx}')
finally:
    for reg, val in saved.items():
        if val is not None:
            h.write(reg, val)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'参数已还原，已失能 0x162={h.read(R_STATE)}')
