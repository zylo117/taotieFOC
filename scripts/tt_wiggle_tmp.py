"""抖振归位理论验证（不改固件，用计数速度模式±18整步来回抖）：
冷启动 → 抖4下(+18,-18,+18,-18 整步@60RPM) → 静置0.3s → 1000RPM×15s → ΔFE。
通过标准 |ΔFE|<3°（修复前冷启动必 -43°）。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

WIG = 18 * 64   # 18 整步 = 1152 µstep@64细分

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.3)

def wiggle(h):
    h.write(0x11A, 60); h.write(0x11B, 60); h.write(0x11C, 3000)
    h.write(0x11D, WIG)
    for mode in (2, 3, 2, 3):
        h.write(0x11E, mode); h.write(0x11F, 1)
        t0 = time.time()
        while time.time() - t0 < 1.2:
            for _ in h._pump(0.02): pass
            if h.latest.get(0x11F) == 0: break
        time.sleep(0.05)
    h.write(0x15F, 1)

for rnd in range(2):
    h.write(0x125, 1); time.sleep(1.2)     # 失能，转子自由
    h.write(0x124, 1); time.sleep(0.2)     # 使能，只等0.2s（最恶劣）
    wiggle(h)
    time.sleep(0.3)
    for _ in h._pump(0.3): pass
    fe0 = s32(h.read(0x121)) / 1000.0
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
    print(f'抖振冷启动轮{rnd+1}: FE {fe0:.2f} → {fe1:.2f}  Δ={fe1-fe0:+.2f}° {"[堵转!]" if locked else ""}')

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
