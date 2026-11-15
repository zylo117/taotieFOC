"""用 0x116 绝对角自己算真实转速，判定 0x120（双EMA估计）是否失真。
速度模式 300RPM、闭环补偿开，取 3 秒遥测流，按 100ms 窗口算瞬时转速。
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 250
COMP = int(sys.argv[2]) if len(sys.argv) > 2 else 1     # 1=闭环补偿开  0=开环
h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x107, COMP)
time.sleep(0.2)
print(f'0x107={h.read(0x107)}（{ "闭环补偿开" if COMP else "开环" }）  0x167={h.read(0x167)} mdeg')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(0x11A, min(120, RPM))
h.write(0x11B, RPM)
h.write(0x11C, 600)
h.write(0x11D, 0xFFFFFFFF)
h.write(0x11E, 2)
h.write(0x11F, 1)
time.sleep(2.5)
print('\n每 0.25s 一个窗口：真实转速（由 0x116 角度增量算）对比 0x120 估计、0x165 补偿')
samples = []
for i in range(12):
    t0 = time.time()
    a0 = h.latest.get(0x116)
    n = 0
    dsum = 0.0
    prev = a0
    while time.time() - t0 < 0.25:
        for _ in h._pump(0.02):
            pass
        a = h.latest.get(0x116)
        if a is not None and prev is not None:
            d = a - prev
            if d > 180000:
                d -= 360000
            elif d < -180000:
                d += 360000
            dsum += d
            n += 1
            prev = a
    dt = time.time() - t0
    true_rpm = (dsum / 1000.0 / 360.0) / dt * 60.0 if dt > 0 else 0.0
    est = h.read(0x120)
    comp = h.read(0x165)
    samples.append((true_rpm, est))
    print(f'  真实 {true_rpm:7.1f} RPM   0x120 估计 {est}   0x165 {comp}   采样点 {n}')
h.write(0x163, 1)
time.sleep(2.0)
tr = [s[0] for s in samples]
es = [float(s[1]) for s in samples if s[1] is not None]
print(f'\n真实转速 min/mean/max = {min(tr):.1f} / {sum(tr)/len(tr):.1f} / {max(tr):.1f} RPM'
      f'   （命令 {RPM}）')
if es:
    print(f'0x120 估计 min/mean/max = {min(es):.1f} / {sum(es)/len(es):.1f} / {max(es):.1f} RPM')
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
