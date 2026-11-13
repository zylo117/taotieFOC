"""纯 raw 监听诊断：命令盲发，固件 printf 完整可读（不与帧解析抢字节）。"""
import sys, time, serial
sys.path.insert(0, 'scripts')
from tt_host import build_frame, _u32


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


PORT = 'COM9'
ser = serial.Serial(PORT, 115200, timeout=0.02, write_timeout=1.0)
time.sleep(0.3)
ser.reset_input_buffer()


def cmd(reg, val):
    ser.write(build_frame(0x02, reg, _u32(val)))
    time.sleep(0.012)


def drain(label=''):
    buf = b''
    end = time.time() + 0.25
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.005)
    txt = ''.join(chr(b) if 32 <= b < 127 else '\n' for b in buf)
    for line in txt.split('\n'):
        line = line.strip()
        # 丢掉二进制帧残骸：只留有字母的行
        if len(line) > 4 and any(c.isalpha() for c in line):
            print(('  | ' if not label else f'  |{label} ') + line, flush=True)


print('--- 初始化 ---', flush=True)
cmd(0x16D, 0)
cmd(0x15F, 1)
cmd(0x123, 1000)
cmd(0x124, 1)          # 使能
time.sleep(0.5)
drain()

print('--- T1 等价：位置正转 120->1000 a600 640000 ---', flush=True)
cmd(0x11A, 120)
cmd(0x11B, 1000)
cmd(0x11C, 600)
cmd(0x11D, 640000)
cmd(0x11E, 0)
ser.reset_input_buffer()
cmd(0x11F, 1)
t0 = time.time()
nxt = t0
while time.time() - t0 < 8:
    n = ser.in_waiting
    if n:
        buf = ser.read(n)
        txt = ''.join(chr(b) if 32 <= b < 127 else '\n' for b in buf)
        for line in txt.split('\n'):
            line = line.strip()
            if len(line) > 4 and any(c.isalpha() for c in line):
                print(f'  [{time.time()-t0:.2f}s] ' + line, flush=True)
    else:
        time.sleep(0.005)
    if time.time() >= nxt:
        nxt += 1.0
print('--- 收尾 ---', flush=True)
cmd(0x11F, 0)
time.sleep(0.5)
drain()
cmd(0x125, 1)
cmd(0x16D, 1)
ser.close()
