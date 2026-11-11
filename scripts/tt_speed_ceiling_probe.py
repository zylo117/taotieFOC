"""低速上升扫速找真实失步上限（温和加速度，避开惯性/共振的叠加效应）。
每个速度点：清堵转+使能 → 60→X RPM（accel 可配）→ 巡航 1.5s → 记录速度中位/是否锁堵转。
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

ACCEL = int(sys.argv[1]) if len(sys.argv) > 1 else 150
POINTS = [int(x) for x in (sys.argv[2].split(',') if len(sys.argv) > 2
           else '300,400,450,500,550,600,650,700'.split(','))]

h = TTHost('COM9')
time.sleep(0.6)
for _ in h._pump(0.4): pass
h.write(0x16D, 0)
time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0:
        break
    time.sleep(0.2)
h.write(0x107, 0)          # 开环纯规划
h.write(0x15F, 1); h.write(0x124, 1)
time.sleep(0.2)
print(f'加速度 {ACCEL} RPM/s，开环')
print(f'{"RPM":>5}  速度中位  峰值   结果')
for rpm in POINTS:
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, ACCEL); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time()
    ramp_s = (rpm - 60) / ACCEL + 0.3
    locked = False
    while time.time() - t0 < ramp_s:
        for _ in h._pump(0.01): pass
        if h.read(0x15D) == 1:
            locked = True
            break
    speeds = []
    t1 = time.time()
    peak = 0
    while not locked and time.time() - t1 < 1.5:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None:
            v = s32(sp)
            speeds.append(v)
            peak = max(peak, v)
        if h.read(0x15D) == 1:
            locked = True
    med = sorted(speeds)[len(speeds) // 2] if speeds else 0
    verdict = '堵转锁存' if locked else ('OK' if abs(med - rpm) <= rpm * 0.05 else f'未达({med})')
    print(f'{rpm:>5}  {med:>7}  {peak:>5}  {verdict}')
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.6)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
