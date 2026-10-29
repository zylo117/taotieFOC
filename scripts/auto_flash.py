#!/usr/bin/env python3
"""AT32F403A 自动刷机 + 日志抓取闭环。

流程：
  1. 通过 USB CDC 发 DFU 魔法命令，让 MCU 从应用直接跳进片内 bootloader
     （不需要再手动拨 BOOT0）
  2. 轮询 dfu-util，等 AT32 DFU 设备出现
  3. 刷 .pio/build/genericAT32F403ACCT7/firmware.bin
  4. 等 MCU 重启，打开 CH340 串口抓 N 秒日志

用法：
  python scripts/auto_flash.py                      # 自动找口，刷机并抓 8 秒日志
  python scripts/auto_flash.py --seconds 15         # 抓 15 秒
  python scripts/auto_flash.py --skip-enter         # MCU 已经在 DFU 模式（手动拨了 BOOT0）
  python scripts/auto_flash.py --only-log           # 只抓日志，不刷机
  python scripts/auto_flash.py --dfu-port COM20     # 手动指定 USB CDC 口

依赖：pyserial（.venv 里已有）
"""

import argparse
import os
import subprocess
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial，请用 .venv\\Scripts\\python.exe 运行本脚本", file=sys.stderr)
    sys.exit(1)

# AT32 的 DFU 设备 VID:PID（注意不是 ST 的 0x0483:0xdf11）
AT32_VID = 0x2E3C
AT32_PID = 0xDF11
# AT32 USB CDC 也用同一个 VID
ARTERY_VID = 0x2E3C
# CH340（UART 日志口）的 VID
CH340_VID = 0x1A86

DFU_UTIL = r"C:\Users\Jessi\.platformio\packages\tool-dfuutil\bin\dfu-util.exe"
FIRMWARE = os.path.join(".pio", "build", "genericAT32F403ACCT7", "firmware.bin")

# 进入 DFU 的魔法：WRITE reg=0x7FFE value="FILE"
DFU_REG = 0x7FFE
DFU_MAGIC = 0x454C4946


def crc8(data: bytes) -> int:
    """TMC/USB CDC 帧用的 CRC8：多项式 0x07，初值 0。"""
    crc = 0
    for byte in data:
        current = byte
        for _ in range(8):
            mix = ((crc >> 7) ^ (current & 0x01)) & 0x01
            crc = (crc << 1) & 0xFF
            if mix:
                crc ^= 0x07
            current >>= 1
    return crc


def build_frame(cmd: int, reg: int, value: int) -> bytes:
    body = bytes([0x55, 0xAA, cmd, (reg >> 8) & 0xFF, reg & 0xFF,
                  (value >> 24) & 0xFF, (value >> 16) & 0xFF,
                  (value >> 8) & 0xFF, value & 0xFF])
    return body + bytes([crc8(body)])


def find_port(vid: int):
    for port in list_ports.comports():
        if port.vid == vid:
            return port.device
    return None


def find_ch340_port():
    """CH340 有时不给 vid（旧驱动），退化成按描述匹配。"""
    for port in list_ports.comports():
        if port.vid == CH340_VID:
            return port.device
    for port in list_ports.comports():
        desc = (port.description or "").upper()
        if "CH340" in desc:
            return port.device
    return None


def dfu_devices():
    if not os.path.exists(DFU_UTIL):
        return ""
    try:
        out = subprocess.run([DFU_UTIL, "--list"], capture_output=True, text=True, timeout=15)
        return out.stdout + out.stderr
    except Exception as error:  # noqa: BLE001
        print(f"dfu-util 调用失败：{error}", file=sys.stderr)
        return ""


def wait_for_dfu(timeout_s=10.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if "2e3c:df11" in dfu_devices().lower():
            return True
        time.sleep(0.3)
    return False


def enter_dfu_via_usb(port: str) -> bool:
    print(f"[1/4] 通过 {port} 发 DFU 魔法命令 ...", flush=True)
    try:
        with serial.Serial(port, 115200, timeout=1.5) as ser:
            ser.reset_input_buffer()
            ser.write(build_frame(0x02, DFU_REG, DFU_MAGIC))
            ser.flush()
            ack = ser.read(10)
            if ack:
                print(f"      收到回包 {ack.hex().upper()}", flush=True)
            else:
                print("      没收到 ACK（MCU 可能没在跑应用，或该口不是 USB CDC）", flush=True)
    except serial.SerialException as error:
        print(f"      打开 {port} 失败：{error}", flush=True)
        return False
    return True


def flash() -> bool:
    if not os.path.exists(FIRMWARE):
        print(f"固件不存在：{FIRMWARE}", file=sys.stderr)
        return False
    print(f"[3/4] 刷机 {FIRMWARE} ...", flush=True)
    cmd = [DFU_UTIL, "-a", "0", "-s", "0x08000000:leave", "-D", FIRMWARE]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    tail = "\n".join((result.stdout + result.stderr).strip().splitlines()[-6:])
    print(tail, flush=True)
    return result.returncode == 0


def capture_log(port: str, seconds: float, baud: int = 115200) -> None:
    print(f"[4/4] 抓日志 {port} @ {baud}，{seconds}s ...", flush=True)
    try:
        with serial.Serial(port, baud, timeout=0.3) as ser:
            deadline = time.time() + seconds
            buffer = b""
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        text = line.decode("utf-8", errors="replace").rstrip("\r")
                        if text.strip():
                            print(text, flush=True)
    except serial.SerialException as error:
        print(f"抓日志失败：{error}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dfu-port", help="USB CDC 所在 COM 口，默认自动按 VID 0x2E3C 找")
    parser.add_argument("--log", help="日志串口，默认自动按 CH340 找")
    parser.add_argument("--seconds", type=float, default=8.0, help="抓日志秒数")
    parser.add_argument("--skip-enter", action="store_true", help="MCU 已在 DFU，跳过第 1 步")
    parser.add_argument("--only-log", action="store_true", help="只抓日志，不刷机")
    args = parser.parse_args()

    log_port = args.log or find_ch340_port()

    if args.only_log:
        if not log_port:
            print("找不到 CH340 串口", file=sys.stderr)
            return 1
        capture_log(log_port, args.seconds)
        return 0

    if not args.skip_enter:
        cdc_port = args.dfu_port or find_port(ARTERY_VID)
        if not cdc_port:
            print("找不到 USB CDC 口（VID 0x2E3C）。若 MCU 已在 DFU 模式，加 --skip-enter",
                  file=sys.stderr)
            return 1
        if not enter_dfu_via_usb(cdc_port):
            return 1

    print("[2/4] 等 DFU 设备出现 ...", flush=True)
    if not wait_for_dfu(12.0):
        print("      超时：dfu-util 没看到 2e3c:df11。跳转可能失败，需要手动拨 BOOT0",
              file=sys.stderr)
        return 1
    print("      DFU 设备已就绪", flush=True)

    if not flash():
        return 1

    time.sleep(2.0)  # 等 MCU 重启并重新枚举串口
    if log_port:
        capture_log(log_port, args.seconds)
    else:
        print("找不到 CH340 串口，跳过日志", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
