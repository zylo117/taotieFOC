"""64细分高速冲刺扫描：验证窄脉冲模式能否复刻老固件 4000RPM 的战绩。
每点记录：编码器速度中位/峰、0x121 FE 峰值、失稳与否。"""
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

ACCEL = 1000
print(f'{"RPM":>6} {"中位":>6} {"峰值":>6} {"FE峰°":>8}  结果')
for rpm in (1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500):
    h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, ACCEL); h.write(0x11D, 0)
    h.write(0x11E, 2); h.write(0x11F, 1)
    dur = (rpm - 60) / ACCEL + 2.2
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
    h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
    time.sleep(0.5)
    if speeds:
        med = sorted(speeds)[len(speeds) // 2]; peak = max(speeds)
    else:
        med = peak = 0
    err = (med - rpm) / rpm * 100 if med else -100
    tag = '堵转' if locked else ('OK' if abs(err) < 3 else f'掉速{err:.0f}%')
    print(f'{rpm:>6} {med:>6} {peak:>6} {fe_max/1000:>8.1f}  {tag}')
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
