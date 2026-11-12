"""窗口引擎拒发现行取证：position模式 accel600 跑，前后对比 0x177(单窗最大索要步数)
/0x178(任务晚拍事件)，并采速度确认巡航 213k pps。"""
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

e0 = h.read(0x178); m0 = h.read(0x177)
h.write(0x11A, 120); h.write(0x11B, 1000); h.write(0x11C, 600)
h.write(0x11D, 640000); h.write(0x11E, 0); h.write(0x11F, 1)
t0 = time.time(); sps = []
while time.time() - t0 < 12:
    for _ in h._pump(0.05): pass
    if h.latest.get(0x11F) == 0 and time.time() - t0 > 2:
        break
    v = s32(h.read(0x120))
    if v > 300: sps.append(v)
e1 = h.read(0x178); m1 = h.read(0x177)
fe_end = s32(h.read(0x121)) / 1000.0
import statistics
mu = statistics.mean(sps) if sps else 0
print(f'巡航速度均值 {mu:.0f} RPM；末值FE {fe_end:.1f}°')
print(f'0x178 晚拍事件 +{e1-e0}   0x177 单窗最大索要步数 {m0}->{m1}')
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 5:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
