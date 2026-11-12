"""真丢步 vs 账面bug 裁决：开环 200RPM 跑精确 60 圈（768000 µstep@64细分），
起/终读绝对角 0x116。绝对角偏差≈FE漂移 → 真丢；绝对角回位而 FE 说有 → 账面bug。"""
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

for _ in h._pump(0.3): pass
a0 = s32(h.read(0x116)) / 1000.0
fe0 = s32(h.read(0x121)) / 1000.0
print(f'起始绝对角 {a0:.2f}°  FE {fe0:.2f}°')

REV = 12800  # 64细分
h.write(0x11A, 60); h.write(0x11B, 200); h.write(0x11C, 600)
h.write(0x11D, 60 * REV); h.write(0x11E, 2); h.write(0x11F, 1)
t0 = time.time()
while time.time() - t0 < 30:
    for _ in h._pump(0.1): pass
    if h.read(0x11F) == 0 or h.read(0x15D) == 1:
        break
    sp = h.latest.get(0x120)
run_t = time.time() - t0
time.sleep(1.5)  # 停稳
for _ in h._pump(0.3): pass
a1 = s32(h.read(0x116)) / 1000.0
fe1 = s32(h.read(0x121)) / 1000.0
d = a1 - a0
if d > 180: d -= 360
if d < -180: d += 360
print(f'跑完 {run_t:.1f}s  终点绝对角 {a1:.2f}°  偏移 {d:+.2f}°  FE {fe1:.2f}° (Δ{fe1-fe0:+.2f}°)')
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
