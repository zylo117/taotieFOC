"""钉死：运行中改写 0x11B 会不会把运动终止？
起步 300 → 确认在转 → 改写 0x11B=500 → 每 300ms 打一组状态。
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


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
h.write(0x107, 0)
h.write(0x11A, 120)
h.write(0x11C, 300)
h.write(0x11B, 300)
h.write(0x11D, 0xFFFFFFFF)
h.write(0x11E, 2)


def snap(tag):
    r11f = h.read(0x11F)
    cmd = s32(h.read(0x135) or 0) / 1000.0
    enc = s32(h.read(0x120) or 0)
    steps = h.read(0x134) or 0
    print(f'  {tag:22s} 11F(运动)={r11f}  135命令={cmd:7.1f}  0x120实测={enc:5d} '
          f'  134已发步={steps:9d}  15D堵转={h.read(0x15D)}  108故障=0x{h.read(0x108) or 0:X}',
          flush=True)


print('起步 300 RPM', flush=True)
h.write(0x11F, 1)
time.sleep(2.5)
snap('300 巡航中')
time.sleep(1.0)
snap('300 巡航中(2)')

print('\n>>> 改写 0x11B = 500（运行中换速）', flush=True)
h.write(0x11B, 500)
for i in range(10):
    time.sleep(0.3)
    snap(f'+{(i + 1) * 0.3:.1f}s')

print('\n>>> 再改回 0x11B = 300', flush=True)
h.write(0x11B, 300)
for i in range(6):
    time.sleep(0.3)
    snap(f'+{(i + 1) * 0.3:.1f}s')

print('\n>>> 重新 startMotion（0x11F=1）看能否恢复', flush=True)
h.write(0x11B, 500)
h.write(0x11F, 1)
for i in range(6):
    time.sleep(0.4)
    snap(f'重启+{(i + 1) * 0.4:.1f}s')

h.write(0x163, 1)
time.sleep(2.5)
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
print('已缓停、失能、运动源交还外部跟随', flush=True)
h.close()
