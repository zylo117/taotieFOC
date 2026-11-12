"""复现网页内部规划运动大误差：position模式 120→1000RPM accel600 count640000。
A: 0x123=200ns（网页当前默认）；B: 0x123=1000ns。FE 全程轨迹 + 峰值 + 末值。"""
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
h.write(0x107, 0); h.write(0x15F, 1)

for width in (200, 1000):
    h.write(0x123, width)
    for _ in h._pump(0.3): pass
    rd = h.read(0x123)
    h.write(0x124, 1); time.sleep(0.4)
    for _ in h._pump(0.3): pass
    h.write(0x11A, 120); h.write(0x11B, 1000); h.write(0x11C, 600)
    h.write(0x11D, 640000); h.write(0x11E, 0); h.write(0x11F, 1)
    t0 = time.time(); fe_min = 0.0; curve = []
    while time.time() - t0 < 45:
        for _ in h._pump(0.05): pass
        if time.time() - t0 > 0.5 and (not curve or time.time() - curve[-1][0] >= 1.0):
            f = s32(h.read(0x121)) / 1000.0
            curve.append((time.time() - t0, f))
            fe_min = min(fe_min, f)
        if h.latest.get(0x11F) == 0 and time.time() - t0 > 3:
            break
        if h.read(0x15D) == 1:
            print('  [堵转]'); break
    time.sleep(1.0)
    for _ in h._pump(0.3): pass
    fe_end = s32(h.read(0x121)) / 1000.0
    peak = min(min(f for _, f in curve), fe_min)
    print(f'0x123 写入{width} 回读{rd}: 峰值|FE| {abs(peak):.2f}° 末值 {fe_end:.2f}°')
    print('  FE轨迹(°):', [round(f, 1) for _, f in curve])
    h.write(0x15F, 1); time.sleep(1.0)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
