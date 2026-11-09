"""taotieFOC COM 串口协议驱动（与固件 usb_cdc_protocol.cpp 帧格式一致）。

帧: 55 AA cmd reg_hi reg_lo v0 v1 v2 v3 crc8
  cmd: 0x01 读, 0x02 写; 回复 0x81 读回 / 0x82 写 ack; 0x10 遥测帧
  crc8: init=0x00, poly=0x07, 前 9 字节
"""
import serial
import time

# ---- 寄存器 ----
R_MICROSTEPS = 0x100
R_FAULT = 0x108
R_FOLLOW_ERR = 0x121  # 0x121 网页用瞬时跟随误差
R_START_RPM = 0x11A
R_MAX_RPM = 0x11B
R_ACCEL = 0x11C
R_PULSE_COUNT = 0x11D
R_MOTION_MODE = 0x11E
R_MOTION_CMD = 0x11F
R_SPEED_RPM = 0x120
R_SPEED = 0x135          # 命令速度 mRPM (×1000, 带方向)
R_ENC_ANGLE = 0x114      # ENCODER_ANGLE_MDEG? 实际 0x114=FOLLOW_ERROR; 角度在 0x116
R_ENABLE = 0x124
R_DISABLE = 0x125
R_POS_KP = 0x129
R_POS_KI = 0x12A
R_POS_KD = 0x12B
R_VEL_KP = 0x12C
R_VEL_KI = 0x12D
R_VEL_KD = 0x12E
R_LAG_ANGLE_MDEG = 0x151
R_PID_TUNE_CTRL = 0x131
R_PID_TUNE_SCORE = 0x132
R_PID_TUNE_SAMPLES = 0x133
R_SAVE = 0x154


def crc8(data, ln):
    c = 0
    for b in data[:ln]:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c & 0xFF


class TT:
    def __init__(self, port="COM14", baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.2)
        self._rx = b""

    def _read_byte(self, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            if not self._rx:
                n = self.s.in_waiting
                if n:
                    self._rx += self.s.read(n)
            if self._rx:
                b = self._rx[0:1]
                self._rx = self._rx[1:]
                return b
        return None

    def _scan_frame(self):
        """在 _rx 里找一帧, 返回 (cmd, reg, val) 或 None。"""
        while len(self._rx) >= 10:
            if self._rx[0] != 0x55 or self._rx[1] != 0xAA:
                self._rx = self._rx[1:]
                continue
            if len(self._rx) < 10:
                return None
            f = self._rx[:10]
            if crc8(f, 9) != f[9]:
                self._rx = self._rx[1:]
                continue
            cmd = f[2]
            reg = (f[3] << 8) | f[4]
            val = (f[5] << 24) | (f[6] << 16) | (f[7] << 8) | f[8]
            self._rx = self._rx[10:]
            return (cmd, reg, val)
        return None

    def _drain(self, ms=10):
        time.sleep(ms / 1000.0)
        n = self.s.in_waiting
        if n:
            self._rx += self.s.read(n)

    def write(self, reg, val, timeout_s=0.4):
        self.s.reset_input_buffer()
        self._rx = b""
        f = bytes([0x55, 0xAA, 0x02, (reg >> 8) & 0xFF, reg & 0xFF,
                   (val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF])
        f += bytes([crc8(f, 9)])
        self.s.write(f)
        self._drain(15)
        return True

    def read(self, reg, timeout_s=0.6):
        self.s.reset_input_buffer()
        self._rx = b""
        f = bytes([0x55, 0xAA, 0x01, (reg >> 8) & 0xFF, reg & 0xFF, 0, 0, 0, 0])
        f += bytes([crc8(f, 9)])
        self.s.write(f)
        end = time.time() + timeout_s
        while time.time() < end:
            r = self._scan_frame()
            if r and r[0] == 0x81 and r[1] == reg:
                return r[2]
            n = self.s.in_waiting
            if n:
                self._rx += self.s.read(n)
        return None

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass
