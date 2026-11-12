"""摆动假说 vs 真丢步：对比冷启动轮与热轮的 0x120 瞬时速度抖动。
大摆动+漂移 → 振动导致测量偏置；小摆动+漂移 → 真丢极。"""
import statistics, sys, time
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

def run15(tag):
    for _ in h._pump(0.3): pass
    fe0 = s32(h.read(0x121)) / 1000.0
    h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 1000)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); sp = []; fe_t = []
    while time.time() - t0 < 15:
        for _ in h._pump(0.02): pass
        v = h.latest.get(0x120)
        if v is not None and time.time() - t0 > 1.5:
            sp.append(s32(v))
        if h.read(0x15D) == 1: break
        if len(fe_t) == 0 or time.time() - fe_t[-1][0] > 2.0:
            fe_t.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < 3.5:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    fe1 = s32(h.read(0x121)) / 1000.0
    mu = statistics.mean(sp); sd = statistics.pstdev(sp)
    print(f'{tag}: 速度均值{mu:.0f} σ={sd:.1f} ({sd/mu*100:.2f}%)  '
          f'FE {fe0:.1f}→{fe1:.1f} (Δ{fe1-fe0:+.1f}°)  FE轨迹 {[(round(t,0), round(f,1)) for t,f in fe_t]}')
    h.write(0x15F, 1)

h.write(0x125, 1); time.sleep(3.0)
h.write(0x124, 1); time.sleep(0.2)
run15('冷轮1   ')
run15('轮2     ')
run15('轮3     ')
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
