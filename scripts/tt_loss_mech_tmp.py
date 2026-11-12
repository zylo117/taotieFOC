"""丢步机制鉴别：FE 漂移率 vs 转速（200/800/1600RPM，各20s巡航）。
恒定比例%→EMI/计数问题；比例随转速陡升→绕组电流跟随(τ)问题。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.3)

print(f'{"RPM":>5} {"斜率°/s":>8} {"丢步%/s":>8} {"整步/s":>7}')
for rpm in (200, 800, 1600):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 600)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    ramp = (rpm - 60) / 600
    time.sleep(ramp + 1.0)
    fe_a = None; t_a = None; locked = False
    t0 = time.time()
    while time.time() - t0 < 21:
        for _ in h._pump(0.1): pass
        if h.read(0x15D) == 1: locked = True; break
        if fe_a is None:
            fe_a = s32(h.read(0x121)); t_a = time.time()
        fe_b = s32(h.read(0x121)); t_b = time.time()
    if locked:
        print(f'{rpm:>5} 堵转'); h.write(0x11F, 0); h.write(0x15F, 1); time.sleep(0.5); continue
    slope = (fe_b - fe_a) / (t_b - t_a) / 1000.0    # deg/s
    pct = slope / (rpm * 360 / 60) * 100            # % of commanded rate
    fullsteps = abs(slope) / 1.8
    print(f'{rpm:>5} {slope:>8.2f} {pct:>8.4f} {fullsteps:>7.2f}')
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < rpm / 600 + 1.5:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    h.write(0x15F, 1); time.sleep(0.4)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
