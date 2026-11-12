"""1000RPM 绝对角起止对照：物理真相版。15s（250圈），起/终绝对角+FE，结束失能。"""
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
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.5)

for rpm, secs in ((1000, 15), (1000, 15)):
    for _ in h._pump(0.3): pass
    a0 = s32(h.read(0x116)) / 1000.0
    fe0 = s32(h.read(0x121)) / 1000.0
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1000)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); fe_mid = []
    while time.time() - t0 < secs:
        for _ in h._pump(0.1): pass
        if h.read(0x15D) == 1: break
        if int(time.time() - t0) % 5 == 0 and (not fe_mid or time.time() - fe_mid[-1][0] > 4):
            fe_mid.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < 3.5:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(1.2)
    for _ in h._pump(0.3): pass
    a1 = s32(h.read(0x116)) / 1000.0
    fe1 = s32(h.read(0x121)) / 1000.0
    d = a1 - a0
    if d > 180: d -= 360
    if d < -180: d += 360
    print(f'{rpm}RPM {secs}s({rpm*secs/60:.0f}圈): 绝对角偏移 {d:+.2f}° | FE {fe0:.1f}→{fe1:.1f}° '
          f'(Δ{fe1-fe0:+.1f}°) | 中段FE采样 {[(round(t,0), round(f,1)) for t,f in fe_mid]}')
    h.write(0x15F, 1); time.sleep(0.5)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
