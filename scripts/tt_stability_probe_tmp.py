"""稳定区侦查：温柔加速(300RPM/s 长斜坡) vs 常速(1000RPM/s)，2500/3000 两点。
每点巡航 5s，缓停 0x163。目的：区分'转矩不够'与'加速斜率激发振荡'。"""
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

print(f'{"acc":>5} {"RPM":>6} {"中位":>6} {"峰值":>6} {"巡航中位":>8} {"FE峰°":>8}')
for accel in (1000, 300):
    for rpm in (2500, 3000):
        h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, accel)
        h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
        ramp = (rpm - 60) / accel
        t0 = time.time(); speeds = []; cruise = []; fe_max = 0
        while time.time() - t0 < ramp + 5.0:
            for _ in h._pump(0.01): pass
            sp = h.latest.get(0x120)
            if sp is not None:
                speeds.append(s32(sp))
                if time.time() - t0 > ramp + 1.0: cruise.append(s32(sp))
            fe = h.latest.get(0x121)
            if fe is not None: fe_max = max(fe_max, abs(s32(fe)))
            if h.read(0x15D) == 1: break
        h.write(0x163, 1); t1 = time.time()
        while time.time() - t1 < rpm / 1000 + 1.5:
            for _ in h._pump(0.02): pass
            sp = h.latest.get(0x120)
            if sp is not None and abs(s32(sp)) < 15: break
        h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.5)
        med = sorted(speeds)[len(speeds)//2] if speeds else 0
        cmed = sorted(cruise)[len(cruise)//2] if cruise else 0
        print(f'{accel:>5} {rpm:>6} {med:>6} {max(speeds) if speeds else 0:>6} {cmed:>8} {fe_max/1000:>8.1f}')
h.write(0x125, 1); h.write(0x16D, 1); h.close()
