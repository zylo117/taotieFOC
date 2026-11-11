"""爆发修复验证：扫速 300..700（150RPM/s），全程盯 0x177 单窗最大步数 / 0x178 停顿事件。
预期：450 不再必死；0x178 若增长 → 存在任务抢占源（已无害化，可再揪源头）。
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9')
time.sleep(0.6)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0:
        break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1)
time.sleep(0.3)
print(f'boot: 0x177={h.read(0x177)} 0x178={h.read(0x178)}')
print(f'{"RPM":>5}  速度中位  峰值   0x177  0x178  结果')
for rpm in [300, 400, 450, 500, 550, 600, 650, 700]:
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 150); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); locked = False
    while time.time() - t0 < (rpm - 60) / 150 + 0.4:
        for _ in h._pump(0.01): pass
        if h.read(0x15D) == 1:
            locked = True; break
    speeds = []
    t1 = time.time()
    peak = 0
    while not locked and time.time() - t1 < 2.0:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None:
            v = s32(sp); speeds.append(v); peak = max(peak, v)
        if h.read(0x15D) == 1:
            locked = True
    med = sorted(speeds)[len(speeds) // 2] if speeds else 0
    w = h.read(0x177); ev = h.read(0x178)
    verdict = '堵转锁存' if locked else ('OK' if abs(med - rpm) <= rpm * 0.05 else f'未达({med})')
    print(f'{rpm:>5}  {med:>7}  {peak:>5}  {w:>5}  {ev:>5}  {verdict}')
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.6)
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
