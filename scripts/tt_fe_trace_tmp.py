"""T1 精确定位：密集采 (t, 转速, FE)，找 FE 从跟随变成发散的时刻与当时转速。

用法：py -X utf8 scripts/tt_fe_trace_tmp.py [max_rpm] [accel] [steps]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


MAX_RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
ACCEL = int(sys.argv[2]) if len(sys.argv) > 2 else 600
STEPS = int(sys.argv[3]) if len(sys.argv) > 3 else 640000

h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x15F, 1)
h.write(0x124, 1)          # 使能
time.sleep(0.4)
h.write(0x123, 1000)
h.write(0x11A, 120)
h.write(0x11B, MAX_RPM)
h.write(0x11C, ACCEL)
h.write(0x11D, STEPS)
h.write(0x11E, 0)
print(f'位置正转 120->{MAX_RPM} a{ACCEL} count={STEPS}  故障字=0x{h.read(0x108) or 0:X}', flush=True)

h.write(0x11F, 1)
t0 = time.time()
nxt = t0
rows = []
while time.time() - t0 < 25:
    for _ in h._pump(0.01):
        pass
    now = time.time()
    if now >= nxt:
        rpm = s32(h.latest.get(0x120) or 0)
        fe = s32(h.read(0x121) or 0) / 1000.0
        rows.append((now - t0, rpm, fe))
        nxt += 0.1
        if h.read(0x15D) == 1:
            print(f'  >>> STALL @ t={now-t0:.2f}s rpm={rpm} FE={fe:.2f}°', flush=True)
            break
        if h.latest.get(0x11F) == 0 and now - t0 > 1.0:
            print(f'  >>> 正常结束 @ t={now-t0:.2f}s FE={fe:.2f}°', flush=True)
            break

# 摘要：FE 第一次越阈值的时刻 + 每 1s 的 (rpm, FE) 采样
print('\n  t(s)   rpm     FE(°)')
for t, rpm, fe in rows[::5]:
    print(f'  {t:5.2f} {rpm:6d} {fe:8.2f}')
first = {}
for t, rpm, fe in rows:
    for th in (2, 5, 10, 30, 90):
        if th not in first and abs(fe) >= th:
            first[th] = (t, rpm, fe)
print('\n  FE 首次越阈值：')
for th in sorted(first):
    t, rpm, fe = first[th]
    print(f'    >={th:3d}°  @ t={t:.2f}s rpm={rpm} FE={fe:.2f}°')
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
