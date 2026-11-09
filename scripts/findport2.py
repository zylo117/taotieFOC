import sys, time
sys.path.insert(0, r"D:\Documents\PlatformIO\Projects\taotieFOC\scripts")
import serial
from tt_proto import crc8

def probe(port, baud=115200):
    s = None
    try:
        s = serial.Serial(port, baud, timeout=0.15)
        time.sleep(0.1)
        s.reset_input_buffer()
        hdr = bytes([0x55,0xAA,0x01,0x01,0x00,0,0,0,0])
        f = hdr + bytes([crc8(hdr,9)])
        s.write(f)
        time.sleep(0.2)
        d = s.read(s.in_waiting or 1)
        return d
    except Exception as e:
        return ("ERR:"+str(e)).encode()
    finally:
        if s: s.close()

for port in ["COM14","COM12","COM11"]:
    d = probe(port)
    tag = d.hex() if isinstance(d,bytes) else str(d)
    ok = "55aa" in tag.lower()
    print("%-6s -> %s  %s" % (port, tag[:80], "<<< TAOTIE" if ok else ""), flush=True)
print("done")
