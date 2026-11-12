"""验收：预同步沉降(0x179)消除冷启动拉入丢步。
3 轮冷启动（失能1.2s→使能0.2s→立即启动 1000RPM×15s→缓停）。
修复前：每轮 ΔFE≈-43°。通过标准：每轮 |ΔFE|<3°。"""
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
dw = h.read(0x179)
print(f'0x179 预同步沉降 = {dw} ms')
h.write(0x124, 1); time.sleep(0.3)

ok = True
for rnd in range(3):
    h.write(0x125, 1); time.sleep(1.2)          # 失能，转子自由
    h.write(0x124, 1)                            # 使能
    time.sleep(0.2)                              # 故意只等 0.2s（修复前必丢的场景）
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
    d = fe1 - fe0
    if abs(d) > 3.0 or locked: ok = False
    print(f'冷启动轮{rnd+1}: FE {fe0:.2f} → {fe1:.2f}  Δ={d:+.2f}° {"[堵转!]" if locked else ""}')

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能', '— 验收通过：冷启动不再丢步' if ok else '— 验收未过')
