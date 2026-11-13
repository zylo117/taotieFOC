"""单条 T1 等价位置运动 + 抓 printf，看 [STALL]/[MOTION] 报什么。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def clean(line):
    return ''.join(chr(b) for b in line if 32 <= b < 127)


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x15F, 1)          # 清堵转
h.write(0x123, 1000)
h.write(0x124, 1)          # 使能
time.sleep(0.4)

print('运动前：故障字=0x%X 堵转=%s 源码11A=%s 11B=%s 11C=%s' % (
    h.read(0x108) or 0, h.read(0x15D), h.read(0x11A), h.read(0x11B), h.read(0x11C)), flush=True)

# T1 等价：位置正转 120->1000 a600 640000
h.write(0x11A, 120)
h.write(0x11B, 1000)
h.write(0x11C, 600)
h.write(0x11D, 640000)
h.write(0x11E, 0)

h.ser.reset_input_buffer()
h.write(0x11F, 1)
t0 = time.time()
nxt = t0
while time.time() - t0 < 12:
    n = h.ser.in_waiting
    if n:
        raw = h.ser.read(n)
        for line in raw.split(b'\n'):
            s = clean(line).strip()
            if s:
                print('  | ' + s, flush=True)
    for _ in h._pump(0.01):
        pass
    now = time.time()
    if now >= nxt:
        print('  t=%.2fs 0x120=%6d 0x121=%8d 0x11F=%s 0x15D=%s' % (
            now - t0, s32(h.latest.get(0x120) or 0), s32(h.read(0x121) or 0),
            h.latest.get(0x11F), h.read(0x15D)), flush=True)
        nxt += 0.5
    if h.latest.get(0x11F) == 0 and now - t0 > 1.5:
        break

h.write(0x11F, 0)
time.sleep(0.6)
for _ in range(30):
    n = h.ser.in_waiting
    if n:
        raw = h.ser.read(n)
        for line in raw.split(b'\n'):
            s = clean(line).strip()
            if s:
                print('  | ' + s, flush=True)
    time.sleep(0.02)
print('结束：故障字=0x%X 堵转=%s FE=%s' % (h.read(0x108) or 0, h.read(0x15D), s32(h.read(0x121) or 0)), flush=True)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
