"""确认 COM14(CH340) 的写命令是否被固件接受: 发使能+低速短行程, 看 printf 是否变化。
同时再试一次读, 确认读回复是否走另一通道。
"""
import sys, time
sys.path.insert(0, r"D:\Documents\PlatformIO\Projects\taotieFOC\scripts")
import serial
from tt_proto import TT, R_ENABLE, R_DISABLE, R_MOTION_CMD, R_START_RPM, R_MAX_RPM, R_ACCEL, R_PULSE_COUNT, R_MOTION_MODE, R_MICROSTEPS

tt = TT("COM14")
s = tt.s
time.sleep(0.3)
s.reset_input_buffer()

print("== 读 0x100 (预期: 无回复, 因为回复走 USB) ==")
v = tt.read(R_MICROSTEPS)
print("read 0x100 ->", v)

print("== 发使能 + 20rpm 短行程正向 0.2圈 ==")
tt.write(R_ENABLE, 1)
time.sleep(0.2)
tt.write(R_START_RPM, 20)
tt.write(R_MAX_RPM, 40)
tt.write(R_ACCEL, 100)
tt.write(R_PULSE_COUNT, 2560)
tt.write(R_MOTION_MODE, 0)
tt.write(R_MOTION_CMD, 1)
time.sleep(0.3)
print("-- 观察 printf 4s --")
end = time.time() + 4.0
while time.time() < end:
    n = s.in_waiting
    if n:
        d = s.read(n)
        sys.stdout.write(d.decode("utf-8","ignore"))
        sys.stdout.flush()
# 停并失能
tt.write(R_MOTION_CMD, 0)
time.sleep(0.5)
tt.write(R_DISABLE, 1)
print("\n== done, 已失能 ==")
s.close()
try: tt.close()
except: pass
