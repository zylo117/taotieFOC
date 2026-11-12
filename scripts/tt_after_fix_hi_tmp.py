"""修后高速复核：2000RPM 无限计数 15s + 2500RPM 10s，看漂移与速度。缓停。"""
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

for rpm, secs in ((2000, 15), (2500, 10)):
    for _ in h._pump(0.3): pass
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1000)
    h.write(0x11D, 0xFFFFFFFF); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); pts = []; sps = []
    while time.time() - t0 < secs:
        for _ in h._pump(0.05): pass
        if h.read(0x15D) == 1: break
        v = s32(h.read(0x120))
        sps.append(v)
        if time.time() - t0 > 1.5 and (not pts or time.time() - pts[-1][0] >= 1.9):
            pts.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < rpm / 400 + 2:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    fe_end = s32(h.read(0x121)) / 1000.0
    n = len(pts); slope = 0.0
    if n >= 3:
        mt = sum(p[0] for p in pts) / n; mf = sum(p[1] for p in pts) / n
        den = sum((t - mt) ** 2 for t, _ in pts)
        slope = sum((t - mt) * (f - mf) for t, f in pts) / den if den else 0.0
    live = [s for s in sps if s > rpm * 0.3]
    import statistics
    mu = statistics.mean(live) if live else 0
    print(f'{rpm}RPM {secs}s: 速度均值{mu:.0f} 漂移{slope:+.2f}°/s FE轨迹 '
          f'{[round(f,1) for f,_ in pts]} 停后{fe_end:.1f}')
    h.write(0x15F, 1); time.sleep(0.6)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
