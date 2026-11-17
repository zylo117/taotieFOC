"""开环 vs 闭环：同一行程，落点残差在停后 1.5s 还剩多少。

  python scripts/tt_cl_tmp.py              行程 3200/6400 微步，各 4 趟，开环/闭环都跑

关键量是【停后 1.5s 的静止残差】= 最终位置 − 起点 − 命令行程。开环只会被动落在
编码器本底（±3 微步）；闭环若有效，这个残差应被压到本底以下。
跑完 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_CLOSED, R_ANG, R_MOTION, R_STATE = 0x107, 0x116, 0x11F, 0x162
R_PULSES, R_MODE, R_FE, R_LEDGER = 0x11D, 0x11E, 0x121, 0x134
REST_S = 1.5


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
    h.write(0x16D, 0)
    time.sleep(0.2)
h.write(0x124, 1)
time.sleep(1.2)
print(f'tuning={h.read(0x187)} start={h.read(0x11A)} max={h.read(0x11B)} acc={h.read(0x11C)}')
try:
    for closed in (0, 1):
        h.write(R_CLOSED, closed)
        time.sleep(0.2)
        for steps in (3200, 6400):
            resid = []
            for i in range(4):
                while h.read(R_MOTION):
                    time.sleep(0.05)
                time.sleep(0.4)
                a0 = h.read(R_ANG)
                h.write(R_PULSES, steps)
                h.write(R_MODE, 0)
                if h.read(R_PULSES) != steps:
                    raise RuntimeError(f'0x11D 丢写 -> {h.read(R_PULSES)}')
                h.write(R_MOTION, 1)
                seen = False
                t0 = time.time()
                last = a0
                while time.time() - t0 < 12:
                    a = h.read(R_ANG)
                    if a is not None:
                        last = a
                    r = h.latest.get(R_MOTION)
                    if r == 1:
                        seen = True
                    elif seen and time.time() - t0 > 0.2:
                        break
                if not seen:
                    raise RuntimeError('未进入运动（0x11F 从未置起）')
                t_end = time.time()
                while time.time() - t_end < REST_S:
                    a = h.read(R_ANG)
                    if a is not None:
                        last = a
                mv = unwrap(last - a0) / 56.25
                b = mv - steps
                resid.append(b)
                print(f'  {"闭环" if closed else "开环"} {steps:5d} 步 第{i + 1}趟  '
                      f'实走 {mv:9.2f}  停后残差 {b:+6.2f} 微步 ({b * 0.05625:+.4f}°)  '
                      f'FE={(s32(h.read(R_FE)) or 0) / 1000.0:+.3f}°  账本={h.read(R_LEDGER)}')
            m = sum(resid) / len(resid)
            print(f'>>> {"闭环" if closed else "开环"} {steps} 步  残差均值 {m:+6.2f} 微步 '
                  f'({m * 0.05625:+.4f}°)  散布 {max(resid) - min(resid):5.2f} 微步\n')
finally:
    h.write(R_CLOSED, 0)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'0x107 已归 0，已失能 0x162={h.read(R_STATE)}')
