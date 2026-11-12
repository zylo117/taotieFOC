"""引擎判别：速度模式(连续引擎) accel600 120→1000，巡航段 FE 是否干净。
干净 → 窗口引擎重武装是元凶；也漂 → 斜坡物理本身。跑3.5s看巡航段，自动数尽前 0x163 缓停。"""
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

h.write(0x11A, 120); h.write(0x11B, 1000); h.write(0x11C, 600)
h.write(0x11D, 640000); h.write(0x11E, 2); h.write(0x11F, 1)
t0 = time.time(); curve = []
while time.time() - t0 < 3.6:
    for _ in h._pump(0.02): pass
    if not curve or time.time() - curve[-1][0] >= 0.25:
        curve.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    if h.latest.get(0x11F) == 0: break
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 5:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
print('速度模式 accel600 巡航段 FE：', [(round(t, 1), round(f, 1)) for t, f in curve])
h.write(0x15F, 1); time.sleep(0.4)
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
