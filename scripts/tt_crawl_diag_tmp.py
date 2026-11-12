"""计划运动爬行定位：1s 粒度采 0x120 速度 / 0x134 已发步数差(=实际pps) / FE。
120→1000RPM accel600 count640000 position模式 1000ns。"""
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
h.write(0x11D, 640000); h.write(0x11E, 0); h.write(0x11F, 1)
t0 = time.time(); prev = None
while time.time() - t0 < 14:
    for _ in h._pump(0.3): pass
    if time.time() - t0 >= (len(str(int(time.time()-t0))) and 0) and (h.latest.get(0x11F) == 0 and time.time() - t0 > 2):
        break
    if prev is None or time.time() - prev[0] >= 1.0:
        em = h.read(0x134); sp = s32(h.read(0x120)); fe = s32(h.read(0x121)) / 1000.0
        if prev is not None:
            dpps = (em - prev[1]) / (time.time() - prev[0])
            print(f't+{time.time()-t0:4.1f}s 速度{sp:>5} 实际pps{dpps:>8.0f} '
                  f'({dpps*60/12800:>6.0f}RPM) 已发{em:>7} FE{fe:>7.1f}°')
        prev = (time.time(), em)
print('结束态 0x11F =', h.latest.get(0x11F), ' 已发 =', h.read(0x134))
h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 6:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
