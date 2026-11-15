"""照抄用户复现条件：25RPM / 50 个脉冲，看是不是堵转守卫断流把转子弹回原位。

读三样东西，一次跑完就能定性质：
  0x116 绝对角  -> 转子到底动没动、动多少、有没有回弹
  0x15D 堵转标志 / 0x15E 堵转事件计数 -> 守卫有没有开火（开火=断流=弹回）
  0x121 跟随误差 -> 命令位置动了吗（同幅=转子自己跳；不动=软件发了脉冲）
另外单独跑一次「把堵转守卫自动断流关掉」的对照（若 0x16x 有开关），
以及一次「同一个 50 脉冲连发 5 次」看是不是每次都被吞。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost, R_MOTION, R_SCHEDULED, R_ENCODER_ANGLE_MDEG, R_FOLLOW_ERR

RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 25
PULSES = int(sys.argv[2]) if len(sys.argv) > 2 else 50
REPS = int(sys.argv[3]) if len(sys.argv) > 3 else 5


def unwrap(x):
    return x - 360000 if x > 180000 else (x + 360000 if x < -180000 else x)


def pump_for(h, sec):
    end = time.time() + sec
    while time.time() < end:
        for _ in h._pump(0.005):
            pass


h = TTHost('COM9')
print(f'口径 0x100={h.read(0x100)} 0x182={h.read(0x182)}  堵转自动断流? 0x16B={h.read(0x16B)}')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(0x107, 0)                      # 开环

print(f'\n=== {RPM} RPM / {PULSES} 微步 = {PULSES * 360.0 / 6400:.3f}° / 连发 {REPS} 次 ===')
try:
    for k in range(REPS):
        pump_for(h, 0.2)
        base = h.latest.get(R_ENCODER_ANGLE_MDEG)
        ev_before = h.read(0x15E) or 0
        st_before = h.read(0x15D) or 0
        h.write(0x11A, max(10, RPM))
        h.write(0x11B, RPM)
        h.write(0x11C, 300)
        h.write(0x11D, PULSES)
        h.write(0x11E, 0)
        h.write(0x11F, 1)

        peak = 0.0                     # 相对起点的最大偏移
        prev = base
        t0 = time.time()
        end_mark = None
        while time.time() - t0 < 3.0:
            for _ in h._pump(0.002):
                pass
            a = h.latest.get(R_ENCODER_ANGLE_MDEG)
            if a is None or prev is None:
                continue
            off = unwrap(a - base) / 1000.0
            peak = max(peak, abs(off))
            prev = a
            if (h.latest.get(R_MOTION) == 0) and end_mark is None:
                end_mark = time.time()
                break
        pump_for(h, 1.2)               # 回弹发生在运动结束之后
        a_end = h.latest.get(R_ENCODER_ANGLE_MDEG)
        off_end = unwrap(a_end - base) / 1000.0 if (a_end and base) else float('nan')
        ev_after = h.read(0x15E) or 0
        st_after = h.read(0x15D) or 0
        ledg = h.read(0x134)
        print(f'第{k + 1}次：账本={ledg}  峰值偏移={peak:.3f}°  1.2s后偏移={off_end:+.3f}°  '
              f'堵转 0x15D {st_before}->{st_after}  事件 0x15E {ev_before}->{ev_after}'
              + ('   <== 守卫开火（断流弹回）' if ev_after > ev_before else ''))
        if st_after:
            h.write(0x15F, 1)
            time.sleep(0.2)
        time.sleep(0.3)
finally:
    h.write(0x11F, 0)
    h.write(0x107, 1)                  # 恢复闭环
    h.write(0x16D, 1)
    h.close()
