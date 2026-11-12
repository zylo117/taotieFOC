"""脉宽 A/B 裁决：1000RPM 无限计数，0x172=1000/2000/3000ns 三档各 15s。
驱动吃边沿 → 加宽后斜率显著变小；转子微观滑步 → 三档斜率不变。"""
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

for width in (1000, 2000, 3000, 2000, 1000):
    h.write(0x123, width)
    for _ in h._pump(0.3): pass
    if h.read(0x123) != width: print(f'{width}ns 写入失败'); continue
    h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 1000)
    h.write(0x11D, 0xFFFFFFFF); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); pts = []
    while time.time() - t0 < 15:
        for _ in h._pump(0.1): pass
        if h.read(0x15D) == 1: break
        if time.time() - t0 > 1.2 and (not pts or time.time() - pts[-1][0] >= 1.4):
            pts.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < 4:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    n = len(pts)
    slope = 0.0
    if n >= 4:
        mt = sum(p[0] for p in pts) / n; mf = sum(p[1] for p in pts) / n
        den = sum((t - mt) ** 2 for t, _ in pts)
        slope = sum((t - mt) * (f - mf) for t, f in pts) / den if den else 0.0
    print(f'脉宽 {width}ns: FE漂移 {slope:+.2f}°/s  (FE {pts[0][1]:.1f}→{pts[-1][1]:.1f}, {n}点)')
    h.write(0x15F, 1); time.sleep(1.0)

h.write(0x123, 1000)
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能，脉宽已还原1000ns')
