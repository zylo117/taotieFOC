"""加速度 A/B：640000步 position模式 max1000RPM，accel 150 vs 600 vs 300。
FE 轨迹 + 峰值 + 末值。1000ns。"""
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

for accel in (150, 300, 600, 150):
    h.write(0x11A, 120); h.write(0x11B, 1000); h.write(0x11C, accel)
    h.write(0x11D, 640000); h.write(0x11E, 0); h.write(0x11F, 1)
    t0 = time.time(); curve = []
    while time.time() - t0 < 25:
        for _ in h._pump(0.02): pass
        if h.latest.get(0x11F) == 0 and time.time() - t0 > 2:
            break
        if not curve or time.time() - curve[-1][0] >= 0.5:
            curve.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    time.sleep(1.2)
    for _ in h._pump(0.3): pass
    fe_end = s32(h.read(0x121)) / 1000.0
    peak = min(min(f for _, f in curve), fe_end)
    print(f'accel {accel}: 用时{time.time()-t0:.1f}s 峰值|FE| {abs(peak):.2f}° 末值 {fe_end:.2f}°')
    print('   ', [(round(t, 1), round(f, 1)) for t, f in curve])
    h.write(0x15F, 1); time.sleep(1.0)

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
