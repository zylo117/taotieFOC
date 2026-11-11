import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

# 找 CPU/控制环频率寄存器
# 0x153 CPU? 0x12E/0x12F pos/vel loop hz
CAND = [0x120, 0x10E, 0x10F, 0x137]

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
for rpm in (300, 400, 450):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 150); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    ramp = (rpm - 60) / 150 + 0.4
    t0 = time.time()
    while time.time() - t0 < ramp:
        for _ in h._pump(0.02): pass
        if h.read(0x15D) == 1:
            break
    snap = {}
    t1 = time.time()
    while time.time() - t1 < 1.2:
        for _ in h._pump(0.02): pass
        for r in CAND:
            v = h.latest.get(r)
            if v is not None:
                snap.setdefault(r, []).append(s32(v))
        if h.read(0x15D) == 1:
            break
    med = {r: (sorted(v)[len(v) // 2]) for r, v in snap.items()}
    print(f'rpm={rpm} speed_med={med.get(0x120)} 快照={med}')
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.6)
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
