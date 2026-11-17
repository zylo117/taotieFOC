"""累积误差：连续 N 趟同向运动，编码器累计位移对 N×命令 的漂移。

  python scripts/tt_drift_tmp.py                 默认 3200 微步 × 20 趟（开环）
  python scripts/tt_drift_tmp.py 6400 20         整圈 × 20 趟
  python scripts/tt_drift_tmp.py 3200 20 rev     反向

每趟结束打印：累计命令 / 累计实走 / 累计误差(微步与度) / 本趟误差。
判读：
  累计误差随趟数线性增长 → 每趟真的多走/少走，是固件的行程刻度问题
  累计误差在 ±3 微步内来回不增长 → 单趟读数是编码器两点之差，电机没有累积漂移
跑完 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_CLOSED, R_ANG, R_MOTION, R_STATE = 0x107, 0x116, 0x11F, 0x162
R_PULSES, R_MODE, R_LEDGER = 0x11D, 0x11E, 0x134

steps = int(sys.argv[1]) if len(sys.argv) > 1 else 3200
reps = int(sys.argv[2]) if len(sys.argv) > 2 else 20
mode = 1 if (len(sys.argv) > 3 and sys.argv[3] == 'rev') else 0

h = TTHost('COM9')
if h.read(0x16D) != 0:
    h.write(0x16D, 0)
    time.sleep(0.2)
h.write(0x124, 1)
time.sleep(1.2)
h.write(R_CLOSED, 0)
print(f'开环 start={h.read(0x11A)} max={h.read(0x11B)} acc={h.read(0x11C)}  '
      f'{steps} 微步 x {reps} 趟  方向={"反" if mode else "正"}')
prev = h.read(R_ANG)
acc_mdeg = 0.0
led_peak = 0
try:
    for i in range(reps):
        while h.read(R_MOTION):
            time.sleep(0.05)
        time.sleep(0.25)
        h.write(R_PULSES, steps)
        h.write(R_MODE, mode)
        if h.read(R_PULSES) != steps:
            raise RuntimeError(f'0x11D 丢写 -> {h.read(R_PULSES)}')
        base = acc_mdeg
        seen = False
        t0 = time.time()
        h.write(R_MOTION, 1)
        while time.time() - t0 < 25:
            a = h.read(R_ANG)
            if a is not None:
                d = a - prev
                while d > 180000:
                    d -= 360000
                while d < -180000:
                    d += 360000
                acc_mdeg += d
                prev = a
            v = h.read(R_LEDGER)
            if isinstance(v, int) and v > led_peak:
                led_peak = v
            r = h.latest.get(R_MOTION)
            if r == 1:
                seen = True
            elif seen and time.time() - t0 > 0.15:
                break
        if not seen:
            raise RuntimeError('未进入运动（0x11F 从未置起）')
        time.sleep(0.15)
        a = h.read(R_ANG)
        if a is not None:
            d = a - prev
            while d > 180000:
                d -= 360000
            while d < -180000:
                d += 360000
            acc_mdeg += d
            prev = a
        exp = (i + 1) * steps
        run_err = acc_mdeg / 56.25 - exp
        leg_err = (acc_mdeg - base) / 56.25 - steps
        print(f'第{i + 1:2d}趟  本趟 {leg_err:+6.2f}  累计误差 {run_err:+7.2f} 微步 '
              f'({run_err * 0.05625:+.3f}°)  累计实走 {acc_mdeg / 56.25:9.1f}/{exp}')
finally:
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'账本峰值={led_peak}  已失能 0x162={h.read(R_STATE)}')
