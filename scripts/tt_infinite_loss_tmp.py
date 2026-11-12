"""干净重测：无限计数(0xFFFFFFFF)排除数尽自停污染。
1000RPM×30s 持续漂移定标；1Hz直读FE；缓停0x163。"""
import statistics, sys, time
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

INFINITE = 0xFFFFFFFF
for rpm, secs in ((1000, 30), (600, 20)):
    for _ in h._pump(0.3): pass
    fe0 = s32(h.read(0x121)) / 1000.0
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1000)
    h.write(0x11D, INFINITE); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); pts = []; sps = []
    while time.time() - t0 < secs:
        for _ in h._pump(0.02): pass
        if h.read(0x15D) == 1: break
        if time.time() - t0 > 1.2 and (not pts or time.time() - pts[-1][0] >= 0.9):
            pts.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
            sps.append(s32(h.read(0x120)))
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < rpm / 300 + 2:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    fe1 = s32(h.read(0x121)) / 1000.0
    t_ref = [p[0] for p in pts]; f_ref = [p[1] for p in pts]
    n = len(pts); slope = 0.0
    if n > 4:
        mt = sum(t_ref) / n; mf = sum(f_ref) / n
        slope = sum((t - mt) * (f - mf) for t, f in zip(t_ref, f_ref)) / sum((t - mt) ** 2 for t in t_ref)
    mu = statistics.mean(sps) if sps else 0
    print(f'{rpm}RPM 无限计数跑{secs}s: 速度均值{mu:.0f} 漂移斜率 {slope:+.2f}°/s '
          f'({abs(slope)/(rpm*6):.4f}%/s)  FE {f_ref[0]:.1f}→{f_ref[-1]:.1f}(运行) 停后{fe1:.1f}')
    print('  FE轨迹:', [(round(t), round(f, 1)) for t, f in pts[::2]])
    h.write(0x15F, 1); time.sleep(0.5)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
