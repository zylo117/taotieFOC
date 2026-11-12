"""1000RPM 矛盾诊断：latest 缓存 vs 直读寄存器逐 500ms 对照。"""
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

h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 600)
h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
t0 = time.time()
while time.time() - t0 < 10:
    for _ in h._pump(0.3): pass
    r120 = h.read(0x120); r14c = h.read(0x14C); r11f = h.read(0x11F)
    r15d = h.read(0x15D); r121 = h.read(0x121); r113 = h.read(0x113)
    print(f't+{time.time()-t0:4.1f}s 120dir={s32(r120):>6} 120c={h.latest.get(0x120)} '
          f'14C={r14c} 11F={r11f} 15D={r15d} 121={s32(r121)/1000:.1f}deg 113={r113}')
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 3.5:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
