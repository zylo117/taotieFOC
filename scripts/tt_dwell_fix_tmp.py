"""启动沉降修复验证：
对照A: 使能→0.2s→立刻1000RPM×15s（复现-43°）
修复B: 使能→0.8s沉降→1000RPM×15s（预期FE漂移≈0）
交叉跑两遍排除偶然。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

def run_case(h, dwell):
    h.write(0x125, 1); time.sleep(1.2)          # 失能+转子自由静置
    h.write(0x124, 1); time.sleep(dwell)        # 使能→沉降 dwell
    for _ in h._pump(0.3): pass
    fe0 = s32(h.read(0x121)) / 1000.0
    h.write(0x15F, 1)
    h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 1000)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); locked = False
    while time.time() - t0 < 15:
        for _ in h._pump(0.05): pass
        if h.read(0x15D) == 1: locked = True; break
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < 3.5:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    fe1 = s32(h.read(0x121)) / 1000.0
    return fe0, fe1, locked

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.3)

for tag, dwell in (('A对照 d=0.2s', 0.2), ('B修复 d=0.8s', 0.8), ('A对照 d=0.2s', 0.2), ('B修复 d=0.8s', 0.8)):
    fe0, fe1, locked = run_case(h, dwell)
    print(f'{tag}: FE {fe0:.2f} → {fe1:.2f}  Δ={fe1-fe0:+.2f}° {"[堵转]" if locked else ""}')

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
