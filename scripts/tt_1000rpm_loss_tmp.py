"""1000RPM 开环持续丢步量化：60s 长跑，0x121 FE 斜率 = 丢步速率。
FE 单位 mdeg；64细分一步 = 28.125 mdeg。结束 0x163 缓停 + 0x125 失能。"""
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

for rpm, secs in ((1000, 60), (400, 30)):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 600)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    ramp = (rpm - 60) / 600
    t0 = time.time(); samples = []; speeds = []
    while time.time() - t0 < ramp + secs:
        for _ in h._pump(0.05): pass
        if time.time() - t0 > ramp + 1.0:
            fe = h.latest.get(0x121); sp = h.latest.get(0x120)
            if fe is not None and sp is not None:
                samples.append((time.time() - t0, s32(fe)))
                speeds.append(s32(sp))
        if h.read(0x15D) == 1:
            print('STALL LOCK'); break
    fe0 = samples[0][1] if samples else 0
    fe1 = samples[-1][1] if samples else 0
    dt = (samples[-1][0] - samples[0][0]) if len(samples) > 1 else 1
    revs = rpm / 60 * dt
    loss_steps = (abs(fe1) - abs(fe0)) / 28.125   # 64细分步
    med = sorted(speeds)[len(speeds)//2] if speeds else 0
    print(f'{rpm}RPM 跑{dt:.0f}s({revs:.0f}圈): 速度中位{med} FE {fe0/1000:.1f}°->{fe1/1000:.1f}° '
          f'丢步≈{loss_steps:.1f}步 即{loss_steps/revs:.3f}步/圈')
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < rpm / 600 + 1.5:
        for _ in h._pump(0.02): pass
        sp = h.latest.get(0x120)
        if sp is not None and abs(s32(sp)) < 15: break
    h.write(0x15F, 1); time.sleep(0.3)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
