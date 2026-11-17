#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核对 BL 镜像里的 DFU 描述符字节是否合 DFU 1.1 规范（dfu-util 就按这些字节行事）。

    py -3 tools/check_dfu_desc.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, ".pio", "build", "bootloader", "firmware.bin")

blob = open(BIN, "rb").read()
# configuration descriptor 的头是 09 02（bLength=9, bType=CONFIG），不是 1B 02 ——
# 27 这个总长在 wTotalLength 里。按 1B 02 去找永远找不到。
i = blob.find(bytes([0x09, 0x02, 0x1B, 0x00]))
if i < 0:
    i = blob.find(bytes([0x09, 0x02]))
if i < 0:
    print("找不到 configuration descriptor（09 02）")
    sys.exit(1)
c = blob[i:i + 27]
print("configuration descriptor @ 镜像偏移 0x%X" % i)
print("  " + " ".join("%02X" % x for x in c))

dev = blob.find(bytes([0x12, 0x01]))
if dev >= 0:
    d = blob[dev:dev + 18]
    print("device descriptor @ 0x%X : VID=0x%04X PID=0x%04X bcdUSB=0x%04X ep0=%d"
          % (dev, d[8] | d[9] << 8, d[10] | d[11] << 8, d[2] | d[3] << 8, d[7]))

tl = c[2] | c[3] << 8
xfer = c[23] | c[24] << 8
bcd = c[25] | c[26] << 8
checks = [
    ("wTotalLength == 27", tl == 27),
    ("接口 bLength/type = 09 04", (c[9], c[10]) == (9, 4)),
    ("接口 class/sub/proto = FE/01/02", (c[14], c[15], c[16]) == (0xFE, 0x01, 0x02)),
    ("接口端点数 = 0（EP0-only）", c[13] == 0),
    ("iInterface = 0（不给字符串，主机就不会去要 index 4）", c[17] == 0),
    ("functional bLength == 9", c[18] == 9),
    ("functional bDescriptorType == 0x21", c[19] == 0x21),
    ("bmAttributes bit7|bit6 (dnload|upload)", (c[20] & 0xC0) == 0xC0),
    ("wTransferSize == 64（和 DFU_XFER_SIZE 一致）", xfer == 64),
    ("bcdDFU == 0x0110（1.1 而非 1.1a，避免 dfu-util 走 DfuSe 分支）", bcd == 0x0110),
]
print("functional: type=0x%02X attr=0x%02X timeout=%dms xfer=%d bcdDFU=0x%04X"
      % (c[19], c[20], c[21] | c[22] << 8, xfer, bcd))
bad = 0
for name, ok in checks:
    print(("  [ok] " if ok else "  [FAIL] ") + name)
    bad += 0 if ok else 1
print("全部通过" if not bad else "%d 项不合规范" % bad)
sys.exit(1 if bad else 0)
