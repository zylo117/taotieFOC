"""连续慢爬升走查：从起步就开始每 0.512s 记一组（转速, 抽头主频/幅值/震度/掩码），
把「随转速升高哪个频率先长起来、幅值到多大时崩」的整个过程抓下来。
用法：py -X utf8 scripts/tt_tap_sweep_tmp.py [accel] [目标RPM]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

ACCEL = int(sys.argv[1]) if len(sys.argv) > 1 else 100
TARGET = int(sys.argv[2]) if len(sys.argv) > 2 else 1000

h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(0.4)
h.write(0x123, 1000)
h.write(0x107, 0)              # 开环
h.write(0x11A, 120)
h.write(0x11C, ACCEL)
h.write(0x11B, TARGET)
h.write(0x11D, 0xFFFFFFFF)
h.write(0x11E, 2)
print(f'连续慢爬升：120 -> {TARGET} RPM，加速 {ACCEL} RPM/s，开环。'
      f'每 0.512s 一组抽头。', flush=True)
print('\n  时刻s  转速RPM | 主频Hz  主峰mdeg  震度mdeg  掩码', flush=True)
h.write(0x11F, 1)
t0 = time.time()
nxt = t0 + 0.55
while time.time() - t0 < 25.0:
    for _ in h._pump(0.01):
        pass
    if time.time() >= nxt:
        nxt += 0.55
        rv = h.latest.get(0x120) or 0
        rpm_val = rv - 0x100000000 if rv & 0x80000000 else rv
        f = h.read(0x17A)
        a = h.read(0x17B)
        r = h.read(0x17C)
        m = h.read(0x17D)
        st = h.read(0x15D)          # 只在同一节拍出，避免抢总线
        print(f'  {time.time()-t0:5.2f} {rpm_val:7d} | '
              f'{(f or 0):6d} {(a or 0):8d} {(r or 0):8d}   0x{(m or 0):02X}', flush=True)
        if st == 1:
            print('  >>> 失步，停在这里', flush=True)
            break
h.write(0x163, 1)
time.sleep(2.5)
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
print('已缓停、失能、运动源交还外部跟随', flush=True)
h.close()
