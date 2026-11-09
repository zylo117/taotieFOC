"""taotieFOC 主机侧协议库（USB CDC，默认 COM9）。

帧格式（与固件 usb_cdc_protocol.cpp 完全一致）：
  55 AA cmd reg_hi reg_lo v0 v1 v2 v3 crc8
  cmd: 0x01 读, 0x02 写; 回复 0x81 读回 / 0x82 写 ack; 0x10 遥测帧
  crc8: init=0x00, poly=0x07, 覆盖前 9 字节

为什么不用 scripts/tt_proto.py：
  - 那个版本 write() 只"发出去然后睡 15ms"，不等 0x82 ACK。整定时连续写几十条参数，
    只要有一条丢在 USB 缓冲里，后面所有试验的评分都对应错的增益，而且完全看不出来。
  - 这里把读/写都做成"等到 reg 匹配的那一帧才算成功"，并顺手把 0x10 遥测帧收进
    latest 快照，整定器就能在不打断控制的前提下看到 0x11F/0x120/0x121。
"""

from __future__ import annotations

import time
from collections import deque

import serial

# ---------------- 寄存器 ----------------
R_MICROSTEPS = 0x100
R_CLOSED_LOOP = 0x107
R_FAULT = 0x108
R_START_RPM = 0x11A
R_MAX_RPM = 0x11B
R_ACCEL = 0x11C
R_PULSE_COUNT = 0x11D
R_MODE = 0x11E
R_MOTION = 0x11F
R_SPEED_RPM = 0x120
R_FOLLOW_ERR = 0x121
R_STEP_WIDTH = 0x123
R_ENABLE = 0x124
R_DISABLE = 0x125
R_ANGLE = 0x128
R_POS_KP, R_POS_KI, R_POS_KD = 0x129, 0x12A, 0x12B
R_VEL_KP, R_VEL_KI, R_VEL_KD = 0x12C, 0x12D, 0x12E
R_ANGLE_MAX_RPM = 0x12F
R_ANGLE_ACCEL = 0x130
R_TUNE_CTRL = 0x131
R_TUNE_SCORE = 0x132
R_TUNE_SAMPLES = 0x133
R_SCHEDULED = 0x134
R_CMD_SPEED_MRPM = 0x135
R_LAG_MDEG = 0x151
R_SAVE = 0x154
R_STALL_DETECTED = 0x15D
R_STALL_EVENTS = 0x15E
R_STALL_CLEAR = 0x15F
R_TUNE_PEAK = 0x160

# 运动模式
MODE_POS_FWD, MODE_POS_REV = 0, 1
MODE_VEL_FWD, MODE_VEL_REV = 2, 3
MODE_ALT_FWD, MODE_ALT_REV = 6, 7


def crc8(data: bytes, ln: int) -> int:
    c = 0
    for b in data[:ln]:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c & 0xFF


def build_frame(cmd: int, reg: int, value: int = 0) -> bytes:
    f = bytes([
        0x55, 0xAA, cmd, (reg >> 8) & 0xFF, reg & 0xFF,
        (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF,
    ])
    return f + bytes([crc8(f, 9)])


def _u32(v: int) -> int:
    return v & 0xFFFFFFFF


def _s32(v: int) -> int:
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


class TTHost:
    def __init__(self, port: str = "COM9", baud: int = 115200, verbose: bool = True):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self.verbose = verbose
        self._rx = bytearray()
        self.latest: dict[int, int] = {}
        self.motion_history: deque[int] = deque(maxlen=4000)
        self.telemetry_frames = 0
        self._drain_quiet(0.25)

    # ---------------- 底层 ----------------
    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _drain_quiet(self, seconds: float):
        end = time.time() + seconds
        self.ser.reset_input_buffer()
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                self.ser.read(n)
            time.sleep(0.005)
        self._rx.clear()

    def _parse(self):
        """把 _rx 里能凑齐的帧全部解析掉，返回列表。"""
        out = []
        while len(self._rx) >= 2:
            if self._rx[0] != 0x55 or self._rx[1] != 0xAA:
                del self._rx[0]
                continue
            if len(self._rx) < 10:
                break
            f = bytes(self._rx[:10])
            if crc8(f, 9) != f[9]:
                del self._rx[0]
                continue
            del self._rx[:10]
            cmd = f[2]
            reg = (f[3] << 8) | f[4]
            val = int.from_bytes(f[5:9], "big")
            if cmd == 0x10:
                self.telemetry_frames += 1
                self.latest[reg] = val
                if reg == R_MOTION:
                    self.motion_history.append(val)
            out.append((cmd, reg, val))
        return out

    def _pump(self, timeout: float):
        end = time.time() + timeout
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                self._rx.extend(self.ser.read(n))
            frames = self._parse()
            if frames:
                yield from frames
            else:
                time.sleep(0.001)

    def read(self, reg: int, timeout: float = 0.6):
        self.ser.write(build_frame(0x01, reg))
        for cmd, r, v in self._pump(timeout):
            if cmd == 0x81 and r == reg:
                return v
        return None

    def write(self, reg: int, value: int, timeout: float = 0.6, need_ack: bool = True) -> bool:
        self.ser.write(build_frame(0x02, reg, _u32(value)))
        if not need_ack:
            return True
        for cmd, r, _v in self._pump(timeout):
            if cmd == 0x82 and r == reg:
                return True
        return False

    def wait_motion_state(self, want: int, timeout: float) -> bool:
        """用遥测帧（0x10 reg=0x11F）等待运动状态，避免频繁主动读把总线塞满。"""
        end = time.time() + timeout
        while time.time() < end:
            if self.latest.get(R_MOTION) == want:
                return True
            for cmd, r, v in self._pump(0.05):
                if cmd == 0x10 and r == R_MOTION and v == want:
                    return True
        return False

    def wait_settled(self, timeout: float = 15.0) -> bool:
        """等运动结束：先看到 running=1，再看到 running=0。"""
        if not self.wait_motion_state(1, min(timeout, 3.0)):
            return False
        return self.wait_motion_state(0, timeout)

    # ---------------- 语义 ----------------
    def fault_flags(self):
        v = self.read(R_FAULT)
        return 0 if v is None else _u32(v)

    def stall(self) -> bool:
        v = self.read(R_STALL_DETECTED)
        return bool(v)

    def clear_stall(self):
        self.write(R_STALL_CLEAR, 1)

    def enable(self):
        self.clear_stall()
        self.write(R_ENABLE, 1)

    def disable(self):
        self.write(R_DISABLE, 1)
        # MOTOR_DISABLE 会置 user_disabled_，再启动前必须重新 ENABLE（enable() 已包含）

    def set_pid(self, pos_kp, pos_ki, pos_kd, vel_kp, vel_ki, vel_kd):
        ok = True
        for reg, gain in (
            (R_POS_KP, pos_kp), (R_POS_KI, pos_ki), (R_POS_KD, pos_kd),
            (R_VEL_KP, vel_kp), (R_VEL_KI, vel_ki), (R_VEL_KD, vel_kd),
        ):
            ok = self.write(reg, int(round(gain * 1000.0))) and ok
        return ok

    def get_pid(self):
        vals = [self.read(r) for r in (R_POS_KP, R_POS_KI, R_POS_KD, R_VEL_KP, R_VEL_KI, R_VEL_KD)]
        if any(v is None for v in vals):
            return None
        return tuple(v / 1000.0 for v in vals)

    def save_params(self) -> bool:
        self.write(R_SAVE, 1)
        time.sleep(0.15)
        return self.read(R_SAVE) == 1

    def microsteps(self) -> int:
        v = self.read(R_MICROSTEPS)
        return 64 if v is None or v == 0 else int(v)

    def run_motion(self, start_rpm, max_rpm, accel_rpm_s, pulse_count, mode,
                   timeout=None, poll=True) -> dict:
        """跑一次规划运动，返回评分/峰值/样本数。调用方负责 enable/disable。"""
        self.motion_history.clear()
        self.write(R_START_RPM, int(start_rpm))
        self.write(R_MAX_RPM, int(max_rpm))
        self.write(R_ACCEL, int(accel_rpm_s))
        self.write(R_PULSE_COUNT, int(pulse_count))
        self.write(R_MODE, int(mode))
        self.write(R_TUNE_CTRL, 1)
        if timeout is None:
            turns = pulse_count / float(self.microsteps() * 200)
            accel_s = max(0.0, (max_rpm - start_rpm)) / max(1.0, accel_rpm_s)
            timeout = max(6.0, turns * 60.0 / max(20.0, max_rpm) * 3.0 + accel_s * 4.0 + 4.0)
        t0 = time.time()
        self.write(R_MOTION, 1)
        started = self.wait_motion_state(1, 3.0)
        finished = self.wait_motion_state(0, timeout)
        elapsed = time.time() - t0
        self.write(R_TUNE_CTRL, 0)
        score = self.read(R_TUNE_SCORE)
        samples = self.read(R_TUNE_SAMPLES)
        peak = self.read(R_TUNE_PEAK)
        return {
            "started": started,
            "finished": finished,
            "elapsed_s": elapsed,
            "score_mdeg": None if score is None else _u32(score),
            "samples": 0 if samples is None else _u32(samples),
            "peak_mdeg": 0 if peak is None else _u32(peak),
        }
