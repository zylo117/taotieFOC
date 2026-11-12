"""跑一次完整磁编码器校准（0x136=1），抓 [CALIBRATION]/[CAL] printf 日志定位卡点。
校准会驱电机约两圈、约 3 分钟；抓到 finished 后失能并交还外部跟随。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def clean(line):
    return ''.join(chr(b) for b in line if 32 <= b < 127)

h = TTHost('COM9'); time.sleep(0.4)
for _ in h._pump(0.3): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x15F, 1); h.write(0x123, 1000); h.write(0x124, 1); time.sleep(0.4)

h.ser.reset_input_buffer()
print('=== 启动校准 0x136=1 ===', flush=True)
h.write(0x136, 1)
t0 = time.time(); buf = b''; finished = False; nlines = 0
while time.time() - t0 < 240:
    n = h.ser.in_waiting
    if n:
        buf += h.ser.read(n)
    else:
        time.sleep(0.02)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        s = clean(raw).strip()
        if not s:
            continue
        nlines += 1
        if any(k in s for k in ('CAL', 'MOTION', 'STALL', 'SAFETY', 'SOURCE', 'polarity', 'REJECT', 'WARN')):
            print(f'{time.time()-t0:6.1f}s {s}', flush=True)
        if 'finished' in s:
            finished = True
            break
    if finished:
        break

print(f'=== 捕获 {nlines} 行, finished={finished} ===', flush=True)
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能，交还外部跟随')
