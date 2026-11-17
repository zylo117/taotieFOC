"""验证修 B：停车后 FE 参考账应等于命令步数，FE 应等于真实落点误差。

  python scripts/tt_feref_tmp.py          闭环，行程 1600(90°) 与 4800(270°)，各 3 趟

避开正好 180° 的 unwrap 边界（findings §9）。每趟报：
  命令步数 / 0x190 参考账 / 编码器实测行程 / 编码器落点误差 / 0x121 FE / 两者之差
判据：0x190 == 命令；|FE − 编码器落点误差| < 0.1°（修前 FE 约只有它的一半）。
跑完 0x107=0、0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_CLOSED, R_ANG, R_MOTION, R_STATE = 0x107, 0x116, 0x11F, 0x162
R_PULSES, R_MODE, R_FE, R_REF = 0x11D, 0x11E, 0x121, 0x190


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
h.write(R_CLOSED, 1)
print(f'0x190 静态读回 = {h.read(R_REF)}（应 0）')
try:
    for steps in (1600, 4800):
        for i in range(3):
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
            while time.time() - t0 < 12:
                h.read(R_ANG)
                r = h.latest.get(R_MOTION)
                if r == 1:
                    seen = True
                elif seen and time.time() - t0 > 0.2:
                    break
            if not seen:
                raise RuntimeError('未进入运动（0x11F 从未置起）')
            t_end = time.time()
            last = h.read(R_ANG)
            while time.time() - t_end < 1.5:
                a = h.read(R_ANG)
                if a is not None:
                    last = a
            mv_mdeg = unwrap(last - a0)
            enc_err = mv_mdeg / 1000.0 - steps * 0.05625
            ref = s32(h.read(R_REF)) or 0
            fe = (s32(h.read(R_FE)) or 0) / 1000.0
            print(f'命令 {steps:5d} 步 第{i + 1}趟  0x190={ref:6d} (差 {ref - steps:+d})  '
                  f'编码器行程 {mv_mdeg / 1000.0:8.3f}°  落点误差 {enc_err:+7.3f}°  '
                  f'FE={fe:+7.3f}°  FE−落点 {(fe - enc_err):+7.3f}°')
finally:
    h.write(R_CLOSED, 0)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'0x107 已归 0，已失能 0x162={h.read(R_STATE)}')
