"""对质：dt>150µs 事件计数(0x178) 与 FE 漏记速率的相关性。
1000RPM 跑 20s：FE斜率 + 0x178增量 + 反推每事件漏记量。"""
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

h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 1000)
h.write(0x11D, 0xFFFFFFFF); h.write(0x11E, 2); h.write(0x11F, 1)
time.sleep(2.0)
ev0 = h.read(0x178); mx0 = h.read(0x177)
fe0 = s32(h.read(0x121)) / 1000.0
t0 = time.time()
while time.time() - t0 < 20:
    for _ in h._pump(0.1): pass
    if h.read(0x15D) == 1: break
fe1 = s32(h.read(0x121)) / 1000.0
ev1 = h.read(0x178); mx1 = h.read(0x177)
dt = time.time() - t0
slope = (fe1 - fe0) / dt
dev = ev1 - ev0
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 4:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print(f'20s@1000RPM: FE斜率 {slope:+.2f}°/s (总{slope*dt:+.1f}°)')
print(f'0x178 晚拍事件: {ev0} → {ev1} (+{dev}, {dev/dt:.1f}/s)  0x177 单窗最大步数 {mx0}→{mx1}')
if dev:
    print(f'每事件漏记 = {abs(slope)*dt/ (1.8*dev):.2f} 整步 = {abs(slope)*dt*12800/ (64*dev):.1f} µstep'
          f' ≈ 每事件 dt超出 {abs(slope)*dt/0.028125/dev/213333*1e6:.0f}µs')
print('已失能')
