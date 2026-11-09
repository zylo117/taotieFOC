import sys, time
sys.path.insert(0, r"D:\Documents\PlatformIO\Projects\taotieFOC\scripts")
import serial
from tt_proto import crc8

ser = serial.Serial("COM14", 115200, timeout=0.3)
time.sleep(0.5)
ser.reset_input_buffer()
# read microsteps 0x100
f = bytes([0x55,0xAA,0x01,0x01,0x00,0,0,0,0])
f += bytes([crc8(f,9)])
print("TX:", f.hex())
ser.write(f)
time.sleep(0.3)
data = ser.read(ser.in_waiting)
print("RX bytes:", len(data))
print("RX hex:", data.hex())
print("RX ascii:", data)
ser.close()
