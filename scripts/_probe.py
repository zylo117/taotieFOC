"""临时：发 DFU 魔法帧并读 COM14 日志，确认跳转走的是哪条分支。"""
import sys
import time

import serial

sys.path.insert(0, "scripts")
from auto_flash import build_frame  # noqa: E402


def p(*args):
    print(*args, flush=True)


log = serial.Serial("COM14", 115200, timeout=0.3, write_timeout=0.3)
log.reset_input_buffer()
cdc = serial.Serial("COM9", 115200, timeout=1.0, write_timeout=1.0)

frame = build_frame(2, 0x7FFE, 0x454C4946)
p("TX", frame.hex().upper())
cdc.write(frame)
time.sleep(1.5)

try:
    p("CDC RX", cdc.read(64).hex().upper())
except Exception as error:  # noqa: BLE001
    p("CDC read fail:", error)

text = log.read(16384).decode("utf-8", "replace")
p("---- LOG ----")
p(text[-1500:] if len(text) > 1500 else text)

try:
    cdc.close()
except Exception:  # noqa: BLE001
    pass
log.close()
p("done")
