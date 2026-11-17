#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""taotieFOC DFU 刷机工具（自定义 bootloader，全程不碰按键）。

    python tools/dfu_flash.py                    进 DFU → 刷应用 → 回读校验 → 回应用
    python tools/dfu_flash.py --port COM9        指定应用当前在哪个 COM 口
    python tools/dfu_flash.py --bin path/x.bin   指定镜像
    python tools/dfu_flash.py --no-enter         板子已经在 DFU 里了，直接刷
    python tools/dfu_flash.py --protect          顺手给 BL 区上 EPP 硬件写保护
    python tools/dfu_flash.py --enter-only       只进 DFU，不刷（手动排查用）
    python tools/dfu_flash.py --list             看现在总线上有没有 DFU 设备
    python tools/dfu_flash.py --bootloader ...   刷 BL 自己（只在芯片内置 DFU 下有效）

镜像格式（和 src/bootloader/bl_flash.c 的 bl_app_valid 严格对应）：
    [ 固件字节 ][ 0xFF 补齐到 0x1AFF0 ][ 16 字节尾签 ]   合计正好 0x1A000 = 应用分区
    尾签 = magic('ZFIL') | payload_len | crc32(payload) | flags
    flags.bit0 = 请求 BL 给自己上 EPP 写保护

为什么"刷不死"：
  * BL 区（0x08000000~0x08001FFF）在 BL 自己的地址守卫之外，DFU 一条命令都写不进去；
    --protect 之后再叠一层硬件 EPP，连 BL 自己都擦不掉。
  * 有效镜像的判据是"尾签 + 整镜像 CRC"，尾签要等整包传完、BL 校验通过才算数。
    中途拔线/断电 → CRC 不符 → 下次上电照样停在 DFU，重跑本脚本即可，不需要续传。
  * 所以下载前先整片擦是安全的：擦完断电，镜像无效，还是停在 DFU。
"""

from __future__ import annotations

import argparse
import binascii
import glob
import os
import struct
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 和 include/bl_layout.h 必须一致
BL_BASE = 0x08000000
BL_SIZE = 0x2000
APP_BASE = 0x08002000
APP_SIZE = 0x1A000
DATA_START = 0x0801C000
PAYLOAD_MAX = APP_SIZE - 16
TRAILER_MAGIC = 0x4C49465A          # 'ZFIL'
FLAG_PROTECT_EPP = 0x00000001

DFU_VID = "2e3c"
DFU_PID = "df11"
# 每个 USB 传输固定 64 字节，和 BL 里的 DFU_XFER_SIZE 一致。
# 显式传给 dfu-util（-t）而不是让它从功能描述符里读：老一版 BL 的功能描述符写错了
# （bDescriptorType 0x04 + bwTimeOut 多写 2 字节被截断），dfu-util 读不到 wTransferSize
# 就直接拒绝；显式指定既绕开这个坑，也保证"主机分片尺寸 == 设备缓冲区尺寸"这件事
# 由工具单方面钉死，不依赖对方自报。
DFU_XFER = 64

REG_DFU_REBOOT = 0x19A
REG_BL_INFO = 0x19B
REG_ENABLE = 0x124
REG_DISABLE = 0x125
REG_CLOSED_LOOP = 0x107
REG_MOTION = 0x11F
REG_MOTION_SOURCE = 0x16D


# ---------------------------------------------------------------- dfu-util
def find_dfu_util() -> str:
    """优先用 PlatformIO 自带的 dfu-util，避免让用户再装一份。"""
    cands = glob.glob(os.path.join(os.path.expanduser("~"), ".platformio",
                                   "packages", "tool-dfuutil*", "bin", "dfu-util*"))
    for c in cands:
        if os.path.isfile(c):
            return c
    return "dfu-util"


def dfu_util_exists(path: str) -> bool:
    try:
        subprocess.run([path, "--version"], capture_output=True, timeout=15)
        return True
    except Exception:
        return False


def dfu_flavor(tool: str) -> str | None:
    """分清总线上的 2e3c:df11 到底是谁 —— 这对 VID/PID 是**故意**共用的：
    芯片内置 BOOT0 DFU 和我的 BL 完全一样，靠 Windows 已绑的驱动免装 Zadig。
    区分办法：内置的 serial="AT32" 且带 alt=1 "Option Byte"；我的是 serial="TAOTIE"、
    只有一个 alt、iProduct="taotie DFU"。"""
    try:
        out = subprocess.run([tool, "--list", "-v"], capture_output=True, timeout=20)
    except Exception:
        return None
    txt = (out.stdout + out.stderr).decode("utf-8", "replace")
    if "2e3c:df11" not in txt.lower() and "VID_2E3C" not in txt.upper():
        return None
    return "mine" if "TAOTIE" in txt else "builtin"


def dfu_present(tool: str) -> bool:
    return dfu_flavor(tool) is not None


def wait_dfu(tool: str, want: bool, timeout: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if dfu_present(tool) == want:
            return True
        time.sleep(0.4)
    return False


# ---------------------------------------------------------------- 镜像
def strip_dfu_suffix(blob: bytes) -> bytes:
    """PlatformIO 的 upload_protocol=dfu 会给 bin 追加 16 字节 DFU suffix。
    我们的镜像是定长填满应用区的，再多 16 字节就写到 0x0801C000（滞后角元数据）上，
    BL 的地址守卫会一票拒绝 —— 所以在这里剥掉，而不是让守卫去背这个锅。

    DFU suffix 布局（尾部 16 字节）：
        [0:2] bcdDevice  [2:4] PID  [4:6] VID  [6:9]="UFD"  [9:11]=0x011A  [11]=0x10  [12:16]=CRC
    所以 "UFD" 在 -10、长度字节在 -5 —— 不是"末尾往前数三个"，别凭印象写。
    （现在 upload_protocol 已经是 custom，suffix 根本不会再被加上；这段纯属防御。）"""
    if len(blob) >= 16 and blob[-10:-7] == b"UFD" and blob[-5] == 16:
        return blob[:-16]
    return blob


def build_app_image(fw_path: str, protect: bool) -> bytes:
    with open(fw_path, "rb") as f:
        blob = strip_dfu_suffix(f.read())
    if len(blob) == 0:
        raise SystemExit(f"镜像是空的：{fw_path}")
    if len(blob) > PAYLOAD_MAX:
        raise SystemExit(
            f"应用镜像 {len(blob)} 字节，超过应用区可用 {PAYLOAD_MAX} 字节。"
            f"要么把代码缩小，要么重谈 BL/应用分区（见 docs/bootloader.md）。")
    payload = blob + b"\xff" * (PAYLOAD_MAX - len(blob))
    crc = binascii.crc32(payload) & 0xFFFFFFFF
    flags = FLAG_PROTECT_EPP if protect else 0
    trailer = struct.pack("<IIII", TRAILER_MAGIC, len(payload), crc, flags)
    image = payload + trailer
    assert len(image) == APP_SIZE, (len(image), APP_SIZE)
    return image


# ---------------------------------------------------------------- 进 DFU
def find_app_port(hint: str | None):
    """找到应用（CDC，VID_2E3C PID_5740）现在挂在哪个 COM 口。"""
    if hint:
        return hint
    try:
        import serial.tools.list_ports as lp
    except Exception:
        return None
    for p in lp.comports():
        vidpid = f"{p.vid:04x}:{p.pid:04x}" if p.vid and p.pid else ""
        if vidpid == "2e3c:5740" or "2e3c" in vidpid:
            return p.device
    for p in lp.comports():
        if "taotie" in (p.description or "").lower() or "AT32" in (p.description or "").upper():
            return p.device
    return None


def enter_dfu(port: str | None, verbose: bool) -> bool:
    """让应用自己复位进 BL 的 DFU —— 这就是"不用按键"的那一步。"""
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    try:
        from tt_host import TTHost
    except Exception as e:                                  # pragma: no cover
        raise SystemExit(f"找不到 scripts/tt_host.py：{e}")

    p = find_app_port(port)
    if not p:
        if verbose:
            print("[enter] 没找到应用的 COM 口；板子可能已经在 DFU 里，或没插。")
        return False
    if verbose:
        print(f"[enter] 连接 {p}，写 0x{REG_DFU_REBOOT:03X}=1")
    h = TTHost(p, verbose=False)
    try:
        # 尽力而为地先把电机带回安全状态：0x19A 的处理函数自己也会 stopMotion +
        # 失能，这里再补一发是为了覆盖"正在跟随/正在跑位置模式"这两种情况。
        try:
            if h.read(REG_MOTION) == 1:
                h.write(REG_MOTION, 0)
                time.sleep(0.25)
        except Exception:
            pass
        h.write(REG_CLOSED_LOOP, 0, need_ack=False)
        h.write(REG_DFU_REBOOT, 1)                          # 等 0x82 ACK
    finally:
        # ACK 到手就断开：复位之后这个 COM 口会消失，留着句柄没有意义
        h.close()
    return True


def wait_app_back(port: str | None, timeout: float) -> str | None:
    """BL 刷完会自己复位；等应用的 CDC 口回来，就是"新固件真的跑起来了"的证据。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = find_app_port(port)
        if p:
            return p
        time.sleep(0.5)
    return None


# ---------------------------------------------------------------- 主流程
def run_dfu(tool: str, args_list: list[str], verbose: bool) -> tuple[int, str]:
    cmd = [tool, "-d", f"{DFU_VID}:{DFU_PID}"] + args_list
    if verbose:
        print("[dfu] " + " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, timeout=900)
    txt = (r.stdout + r.stderr).decode("utf-8", "replace")
    if verbose:
        print(txt.strip())
    return r.returncode, txt


def main() -> int:
    ap = argparse.ArgumentParser(description="taotieFOC 自定义 DFU 刷机")
    ap.add_argument("--bin", default=os.path.join(ROOT, ".pio", "build",
                                                  "genericAT32F403ACCT7", "firmware.bin"))
    ap.add_argument("--port", default=None, help="应用当前所在的 COM 口")
    ap.add_argument("--no-enter", action="store_true", help="已经在 DFU 里，跳过进 DFU")
    ap.add_argument("--enter-only", action="store_true", help="只进 DFU 不刷")
    ap.add_argument("--list", action="store_true", help="列出 DFU 设备后退出")
    ap.add_argument("--protect", action="store_true", help="顺带请求 BL 上 EPP 硬件写保护")
    ap.add_argument("--bootloader", action="store_true",
                    help="刷 BL 自己到 0x08000000（只在芯片内置 DFU / BOOT0 下有效）")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args()
    v = not a.quiet

    tool = find_dfu_util()
    if not dfu_util_exists(tool):
        print("找不到 dfu-util（PlatformIO 的 tool-dfuutil 包里应该有一个）。", file=sys.stderr)
        return 2
    if v:
        print(f"[tool] {tool}")

    if a.list:
        code, txt = run_dfu(tool, ["--list"], v)
        print(txt)
        print("识别为:", dfu_flavor(tool))
        return 0

    # ---- 刷 BL 自己：走芯片内置 DFU（按一次 BOOT0），这是唯一还需要手的场合
    if a.bootloader:
        with open(a.bin, "rb") as f:
            blob = strip_dfu_suffix(f.read())
        if len(blob) > BL_SIZE:
            print(f"BL 镜像 {len(blob)} 字节 > BL 区 {BL_SIZE} 字节，拒绝。", file=sys.stderr)
            return 2
        tmp = os.path.join(ROOT, ".pio", "bl_dfu_image.bin")
        os.makedirs(os.path.dirname(tmp), exist_ok=True)
        with open(tmp, "wb") as f:
            f.write(blob)
        if v:
            print(f"[BL] {len(blob)} 字节 → 0x{BL_BASE:08X}（需要板子在芯片内置 DFU 模式）")
        flav = dfu_flavor(tool)
        if flav == "mine":
            print("[BL] 在线的是**我的 BL**，它按设计会拒绝任何写 0x08000000 的请求。\n"
                  "     重刷 BL 必须断电 + 按住 BOOT0 上电，回到芯片内置 DFU 再来。", file=sys.stderr)
            return 5
        code, txt = run_dfu(tool, ["-a", "0", "-s", f"0x{BL_BASE:08X}", "-D", tmp], v)
        if code != 0:
            print("[BL] 刷写失败：确认板子按着 BOOT0 上电进了内置 DFU，再试一次。", file=sys.stderr)
            return 1
        print("[BL] 完成。现在起所有刷机都不再需要按键。")
        return 0

    # ---- 应用镜像
    image = build_app_image(a.bin, a.protect)
    if v:
        crc = binascii.crc32(image[:PAYLOAD_MAX]) & 0xFFFFFFFF
        print(f"[img ] {a.bin} → {len(image)} 字节，payload CRC=0x{crc:08X}"
              f"{'  +EPP' if a.protect else ''}")

    tmp = os.path.join(ROOT, ".pio", "app_dfu_image.bin")
    os.makedirs(os.path.dirname(tmp), exist_ok=True)
    with open(tmp, "wb") as f:
        f.write(image)

    if not a.no_enter:
        flav = dfu_flavor(tool)
        if flav:
            if v:
                print(f"[enter] 已在 DFU（{'我的 BL' if flav == 'mine' else '芯片内置 DFU'}），不叫它了")
            if flav == "builtin":
                print("[warn ] 这是 BOOT0 内置 DFU：应用会直接写到 0x08002000，BL 区不受影响，"
                      "但有效性要等 BL 下次启动才判定。")
        else:
            if not enter_dfu(a.port, v):
                print("没能通过 0x19A 让板子进 DFU（应用没响应）。"
                      "如果板子已经停在 DFU，加 --no-enter 再来。", file=sys.stderr)
                return 3
            if not wait_dfu(tool, True, 12.0):
                print("等了 12s 没看到 DFU 设备。可能 BL 没刷进去，或者驱动没绑上。",
                      file=sys.stderr)
                return 3

    if a.enter_only:
        print("[done] 已在 DFU 模式。")
        return 0

    t0 = time.time()
    # 【不要给 -s 地址】我的 BL 是"单区 DFU 1.1 设备"，不是 ST 的 DfuSe 设备：
    # DFU 1.1 里 DNLOAD/UPLOAD 的 wValue 是**块号**，地址由设备自己按
    # base + block * wTransferSize 算（base 已经钉死在 bl_layout.h 的 0x08002000）。
    # dfu-util 对非 DfuSe 设备传 -s 0x08002000 会把它换算成块号 0x20008 发过来，
    # BL 那边就变成 0x08002000 + 0x20008*64 —— 早就跑出应用区，被地址守卫一票拒绝。
    # （-s 只在 --bootloader 那条路用，那边是芯片内置的 DfuSe，语义完全不同。）
    code, txt = run_dfu(tool, ["-a", "0", "-t", str(DFU_XFER), "-D", tmp], v)
    # dfu-util 对 dfuMANIFEST-WAIT-RESET 的处理各版本不一，退出码不能当结论用。
    # 真正说话的是下面那次 DFU_UPLOAD 回读：它逐字节比过了，才算刷成功。
    if code != 0 and v:
        print("[flash] dfu-util 退出码非 0（多半是 manifest 后等设备复位），继续用回读判定。")
    if v:
        print(f"[flash] 下载完成，用时 {time.time() - t0:.1f}s")

    if not a.no_verify:
        up = os.path.join(ROOT, ".pio", "app_readback.bin")
        # dfu-util 的 -U 不肯覆盖已存在的文件（"File exists: Cannot open ... for
        # writing"，然后直接退出）。上一次失败留下的空文件会让本轮校验必挂，
        # 而且报的是"打不开文件"，很容易被误读成设备没应答 —— 先删掉。
        if os.path.exists(up):
            os.remove(up)
        # 长度写在 -s 里（"地址:长度"），dfu-util 没有 -l 这个选项 —— 写错它会直接吐
        # 一大段 usage 然后退出，看起来像"设备没应答"，其实是命令行参数非法。
        # 同上：回读也不给地址，从块 0 顺序读；BL 在越界时回 0 长度包，
        # 于是 dfu-util 自然在读满整个 104KB 应用区时收尾。
        code, txt = run_dfu(tool, ["-a", "0", "-t", str(DFU_XFER), "-U", up], v)
        if code != 0 or not os.path.exists(up):
            print("[verify] 回读失败（BL 的 DFU_UPLOAD 没应答？）", file=sys.stderr)
            return 1
        with open(up, "rb") as f:
            got = f.read()
        if len(got) < APP_SIZE:
            got = got + b"\xff" * (APP_SIZE - len(got))
        if got[:APP_SIZE] != image:
            bad = next(i for i in range(APP_SIZE) if got[i] != image[i])
            print(f"[verify] 回读不一致，第一个差异在偏移 0x{bad:05X}"
                  f"（期望 0x{image[bad]:02X} 读回 0x{got[bad]:02X}）", file=sys.stderr)
            return 1
        if v:
            print("[verify] 回读 104KB 逐字节一致 ✓")

    # BL 会自动复位进新应用；CDC 口回来 = 新固件跑起来了
    p = wait_app_back(a.port, 15.0)
    if p:
        print(f"[done] 新固件已启动，应用回到 {p}")
        # 最后一道确认：直接问运行中的固件"你是哪一版"。刷机工具说成功只说明
        # flash 内容对了；寄存器答对了才算"代码真的跑起来了"。
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import fw_version
            d = fw_version.query(p)
            if d:
                print(f"[fw  ] 运行中的固件：{fw_version.describe(d)}")
            else:
                print("[fw  ] 读不到 0x19C~0x19E：这是加身份寄存器之前的旧固件", file=sys.stderr)
        except Exception as e:
            print(f"[fw  ] 版本查询失败（不影响刷机结果）：{e}", file=sys.stderr)
    else:
        print("[done] 刷机+校验通过，但 15s 内没看到应用 CDC 口回来。"
              "可能是应用本身启不来（BL 会在连续 3 次失败后停在 DFU），"
              "按 BOOT0 上电用内置 DFU 兜底，或看 docs/bootloader.md。", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
