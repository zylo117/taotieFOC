"""决定性实验：64细分→16细分（同 RPM 脉冲率 /4）。
天花板随脉冲率走（≈4倍 RPM）→ 码/驱动输入级限脉冲速率；
天花板钉在同一 RPM → 转矩/供电电压物理限（与细分无关）。
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
print('细分回读', h.read(0x100))
h.write(0x100, 16)
time.sleep(0.3)
print('写 16 后回读', h.read(0x100))

def sweep(rpm_list, tag):
    print(f'--- {tag} ---')
    for rpm in rpm_list:
        h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 150); h.write(0x11D, 0)
        h.write(0x11E, 2); h.write(0x11F, 1)
        t0 = time.time(); locked = False
        while time.time() - t0 < (rpm - 60) / 150 + 0.4:
            for _ in h._pump(0.01): pass
            if h.read(0x15D) == 1:
                locked = True; break
        speeds = []
        t1 = time.time()
        while not locked and time.time() - t1 < 1.5:
            for _ in h._pump(0.01): pass
            sp = h.latest.get(0x120)
            if sp is not None: speeds.append(s32(sp))
            if h.read(0x15D) == 1: locked = True
        med = sorted(speeds)[len(speeds) // 2] if speeds else 0
        print(f'  {rpm}: {"堵转" if locked else "OK"} 速度中位={med}')
        h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
        time.sleep(0.6)

sweep([300, 450, 600, 700, 900, 1100, 1300], '16 细分')
h.write(0x100, 64)   # 恢复
time.sleep(0.2)
print('恢复细分=', h.read(0x100))
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
