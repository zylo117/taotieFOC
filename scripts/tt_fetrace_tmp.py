"""闭环停后 FE 轨迹：判断"环被门控关掉"还是"积分太慢"。

  python scripts/tt_fetrace_tmp.py         3200 微步，停后连续采 6 秒

每 ~120ms 记一组 (t, 0x116 绝对角, 0x121 FE, 0x11F, 0x18C 状态位)。
  FE 从 −1° 缓慢爬向 0 → I 太弱/被限幅，调增益或放宽积分门控
  FE 一到 −1° 就纹丝不动 → 停后被门控（SETTLE/保持条件）直接关了修正
跑完 0x107=0、0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_CLOSED, R_ANG, R_MOTION, R_STATE = 0x107, 0x116, 0x11F, 0x162
R_PULSES, R_MODE, R_FE, R_BITS = 0x11D, 0x11E, 0x121, 0x18C
STEPS = 3200


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


h = TTHost('COM9')
if h.read(0x16D) != 0:
    h.write(0x16D, 0)
    time.sleep(0.2)
h.write(0x124, 1)
time.sleep(1.2)
h.write(R_CLOSED, 1)
h.write(R_PULSES, STEPS)
h.write(R_MODE, 0)
if h.read(R_PULSES) != STEPS:
    raise RuntimeError(f'0x11D 丢写 -> {h.read(R_PULSES)}')
print(f'闭环 FE 轨迹（命令 {STEPS} 微步 = 180.0000°）  kp={h.read(0x129)} ki={h.read(0x12A)} '
      f'kd={h.read(0x12B)}  max_rpm={h.read(0x12F)} accel={h.read(0x130)}')
try:
    h.write(R_MOTION, 1)
    t0 = time.time()
    seen = False
    t_stop = None
    while time.time() - t0 < 8:
        ang = h.read(R_ANG)
        fe = s32(h.read(R_FE))
        bits = h.read(R_BITS)
        r = h.latest.get(R_MOTION)
        if r == 1:
            seen = True
        elif seen and t_stop is None:
            t_stop = time.time()
        if fe is not None and (t_stop is None or time.time() - t_stop < 6.0):
            rel = (time.time() - t_stop) if t_stop else -1.0
            print(f't={time.time() - t0:6.2f}s 停后{rel:+5.2f}s  '
                  f'ang={ang if ang is not None else -1:8d} mdeg  '
                  f'FE={(fe or 0) / 1000.0:+7.3f}°  11F={r}  bits={bits}')
        if t_stop and time.time() - t_stop > 6.0:
            break
        time.sleep(0.12)
    if not seen:
        print('未进入运动（0x11F 从未置起）')
finally:
    h.write(R_CLOSED, 0)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'0x107 已归 0，已失能 0x162={h.read(R_STATE)}')
