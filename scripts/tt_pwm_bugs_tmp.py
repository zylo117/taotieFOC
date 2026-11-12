"""钉死连续PWM两个bug：
A. 低速死区：目标 10RPM（<17.9RPM 下限）→ 预期复现"电机不动+堵转锁存"
B. 高速封顶：扫 1000/1500/2000/2400/3000 → 预期速度饱和 ~2343RPM，
   且 0x121 跟随误差（假账）随目标超限而暴涨
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9')
time.sleep(0.6)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0:
        break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1)
time.sleep(0.3)

def run(rpm, accel, dur, tag):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, accel); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); locked = False
    fe_max = 0; speeds = []
    while time.time() - t0 < dur:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None: speeds.append(s32(sp))
        fe = h.latest.get(0x121)
        if fe is not None: fe_max = max(fe_max, abs(s32(fe)))
        if h.read(0x15D) == 1:
            locked = True; break
    med = sorted(speeds)[len(speeds) // 2] if speeds else 0
    print(f'{tag} 目标{rpm}: {"堵转" if locked else "跑满"} 速度中位={med} FE峰值={fe_max/1000:.1f}°')
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.6)

print('--- A. 低速死区测试 ---')
run(10, 100, 3.0, 'A')
run(30, 100, 3.0, 'A')
print('--- B. 高速封顶测试 (accel 600) ---')
for rpm in (1000, 1500, 2000, 2400, 3000):
    run(rpm, 600, (rpm - 60) / 600 + 2.5, 'B')
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
