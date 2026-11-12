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
for rpm in (2000, 2500, 3000, 4000):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1000); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); speeds = []; locked = False
    while time.time() - t0 < (rpm - 60) / 1000 + 2.0:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None: speeds.append(s32(sp))
        if h.read(0x15D) == 1: locked = True; break
    h.write(0x11B, 10); t1 = time.time()
    while time.time() - t1 < (rpm / 1000) + 1.2:
        for _ in h._pump(0.02): pass
        sp = h.latest.get(0x120)
        if sp is not None and abs(s32(sp)) < 15: break
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.4)
    med = sorted(speeds)[len(speeds)//2] if speeds else 0
    print(f'target {rpm}: median {med} peak {max(speeds) if speeds else 0} {"LOCK" if locked else ""}')
h.write(0x125, 1); h.write(0x16D, 1); h.close()
