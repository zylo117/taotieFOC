"""烧录后冒烟：抓启动日志几秒，确认 lag-angle 加载不卡 + 通信活着。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def clean(line):
    return ''.join(chr(b) for b in line if 32 <= b < 127)

h = TTHost('COM9'); time.sleep(0.3)
# 抓串口里 [LAG]/[BOOT]/[INIT]/[CALIB] 相关行
h.ser.reset_input_buffer()
t0 = time.time(); buf = b''
while time.time() - t0 < 3:
    n = h.ser.in_waiting
    if n:
        buf += h.ser.read(n)
    else:
        time.sleep(0.02)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        s = clean(raw).strip()
        if s and any(k in s.upper() for k in ('LAG', 'BOOT', 'INIT', 'FIT', 'CALIB', 'READY', 'CTRL')):
            print(s, flush=True)
for _ in h._pump(0.3): pass
ang = h.read(0x116)
fault = h.read(0x108)
print(f'通信 OK：角度原始读数={ang} 故障字={fault}')
h.close()
