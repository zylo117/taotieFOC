import sys, time
sys.path.insert(0, r"D:\Documents\PlatformIO\Projects\taotieFOC\scripts")
import serial
from tt_proto import crc8

def probe(port, baud=115200):
    try:
        s = serial.Serial(port, baud, timeout=0.25)
    except Exception as e:
        return None
    try:
        time.sleep(0.15)
        s.reset_input_buffer()
        f = bytes([0x55,0xAA,0x01,0x01,0x00,0,0,0,0]) + bytes([crc8(bytes([0x55,0xAA,0x01,0x01,0x00,0,0,0,0]),9)])
        s.write(f)
        time.sleep(0.25)
        d = s.read(s.in_waiting)
        return d
    finally:
        s.close()

for port in ["COM14","COM3","COM4","COM5","COM6","COM7","COM8","COM9","COM11","COM12"]:
    d = probe(port)
    tag = d.hex() if d else "none"
    ok = "55aa" in tag.lower()
    print("%-6s -> %s  %s" % (port, tag[:60], "<<< TAOTIE" if ok else ""))
