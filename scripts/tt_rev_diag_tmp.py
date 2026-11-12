"""T3 纯反转堵转取证：每 0.3s 采 速度0x120 / FE0x121 / 已发0x134 / 堵转0x15D 时间线。"""
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
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x123, 1000)
h.write(0x124, 1); time.sleep(0.4)

h.write(0x11A, 120); h.write(0x11B, 800); h.write(0x11C, 300)
h.write(0x11D, 256000); h.write(0x11E, 1); h.write(0x11F, 1)
t0 = time.time()
while time.time() - t0 < 12:
    for _ in h._pump(0.05): pass
    if time.time() - t0 >= 0.3 and True:
        sp = s32(h.read(0x120)); fe = s32(h.read(0x121)) / 1000.0
        em = h.read(0x134); st = h.read(0x15D)
        print(f't+{time.time()-t0:4.1f} 速度{sp:>6} FE{fe:>8.1f}° 已发{em:>7} 堵转{st}')
        if st == 1 or (h.latest.get(0x11F) == 0 and time.time() - t0 > 1.5):
            break
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 5:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x15F, 1); time.sleep(0.3)
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
