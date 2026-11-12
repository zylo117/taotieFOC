"""T5 重测：按真实用法跑角度模式（小行程 +30°，再 +330° 回原位=正向绕一大圈）。
角度模式单向是既有设计：只验正向到位精度，不逼它倒车。"""
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
h.write(0x15F, 1); h.write(0x123, 1000); h.write(0x124, 1); time.sleep(0.4)

base = (s32(h.read(0x116)) / 1000.0) % 360.0
tgt = base
for tgt_off, tmo in ((5.0, 8), (5.0, 8), (5.0, 8)):
    tgt = (tgt + tgt_off) % 360.0
    h.write(0x128, int(tgt * 1000))
    t0 = time.time(); done = False
    while time.time() - t0 < tmo:
        for _ in h._pump(0.03): pass
        if h.read(0x15D) == 1: done = 'STALL'; break
        if h.latest.get(0x11F) == 0 and time.time() - t0 > 0.8: done = True; break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    now = (s32(h.read(0x116)) / 1000.0) % 360.0
    d = ((now - tgt + 180.0) % 360.0) - 180.0
    print(f'角度→{tgt:.1f}°: {"到位" if done is True else done} 实测{now:.1f}° 误差{d:+.2f}°')
    if done == 'STALL':
        h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.5)

h.write(0x107, 0); h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
