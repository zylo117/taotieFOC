import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9')
time.sleep(0.6)
for _ in h._pump(0.4): pass
h.write(0x16D, 0)
time.sleep(0.15)
for _ in range(4):
    v = h.read(0x16D)
    if v == 0:
        break
    time.sleep(0.2)
h.write(0x15F, 1); h.write(0x107, 0); h.write(0x124, 1)
time.sleep(0.2)
for rpm in (300, 550, 650):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1500); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time()
    locked = None
    speeds = []
    while time.time() - t0 < 5.0:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None:
            speeds.append(s32(sp))
        if h.read(0x15D) == 1 and locked is None:
            locked = time.time() - t0
            break
    med = sorted(speeds)[len(speeds) // 2] if speeds else None
    tag = ('锁存 %.1fs' % locked) if locked is not None else '未锁存(跑满5s)'
    print(f'{rpm}: {tag}  速度中位={med}')
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.8)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
