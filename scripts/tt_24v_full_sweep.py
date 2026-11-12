"""24V 全速域复测（用户要求：缓停，不要急停）。
每点：巡航结束 → 写 0x11B=10 让梯形机按减速度自然降速 → 低速才 0x11F=0。
先复验 450/550 悬崖（1µs 宽脉冲是否治好 24V），再冲 1000~4500。"""
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

def soft_stop(cur_rpm, accel):
    """降速到 10RPM 再切断，避免高速急停冲击硬件。"""
    h.write(0x11B, 10)
    ramp = max(0.3, (cur_rpm - 10) / accel)
    t0 = time.time()
    while time.time() - t0 < ramp + 0.6:
        for _ in h._pump(0.02): pass
        sp = h.latest.get(0x120)
        if sp is not None and abs(s32(sp)) < 15:
            break
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.4)

print(f'{"RPM":>6} {"中位":>6} {"峰值":>6} {"FE峰°":>8}  结果')
points = [(450, 600), (550, 600), (1000, 1000), (1500, 1000), (2000, 1000),
          (2500, 1000), (3000, 1000), (3500, 1000), (4000, 1000), (4500, 1000)]
for rpm, accel in points:
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, accel); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    dur = (rpm - 60) / accel + 2.2
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
    cur = s32(h.latest.get(0x120) or 0)
    if not locked:
        soft_stop(abs(cur) if cur else rpm, accel)
    else:
        h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.4)
    med = sorted(speeds)[len(speeds) // 2] if speeds else 0
    peak = max(speeds) if speeds else 0
    err = (med - rpm) / rpm * 100 if med else -100
    tag = '堵转' if locked else ('OK' if abs(err) < 3 else f'掉速{err:.0f}%')
    print(f'{rpm:>6} {med:>6} {peak:>6} {fe_max/1000:>8.1f}  {tag}')
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
