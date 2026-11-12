"""复现网页路径的速度模式（有限脉冲数 0x11D≠0）验证 2062 容量墙已消失。
缓停用固件原生 0x163（按 0x11C 减速到 0 就地收尾）。"""
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

print(f'{"RPM":>6} {"中位":>6} {"峰值":>6} {"FE峰°":>8}  结果')
for rpm in (2000, 2500, 3000, 4000):
    # 网页路径：0x11D 写有限计数（2M 脉冲 ≈156 圈，测试期内跑不完）
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 1000)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); speeds = []; fe_max = 0; locked = False
    while time.time() - t0 < (rpm - 60) / 1000 + 2.0:
        for _ in h._pump(0.01): pass
        sp = h.latest.get(0x120)
        if sp is not None: speeds.append(s32(sp))
        fe = h.latest.get(0x121)
        if fe is not None: fe_max = max(fe_max, abs(s32(fe)))
        if h.read(0x15D) == 1: locked = True; break
    # 固件原生缓停 0x163
    h.write(0x163, 1)
    t1 = time.time()
    while time.time() - t1 < rpm / 1000 + 1.5:
        for _ in h._pump(0.02): pass
        sp = h.latest.get(0x120)
        if sp is not None and abs(s32(sp)) < 15: break
    h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.4)
    med = sorted(speeds)[len(speeds)//2] if speeds else 0
    peak = max(speeds) if speeds else 0
    err = (med - rpm) / rpm * 100 if med else -100
    tag = '堵转' if locked else ('OK' if abs(err) < 3 else f'掉速{err:.0f}%')
    print(f'{rpm:>6} {med:>6} {peak:>6} {fe_max/1000:>8.1f}  {tag}')
h.write(0x125, 1); h.write(0x16D, 1); h.close()
