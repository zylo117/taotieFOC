#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查固件身份：板上正在跑哪一版 / 某个镜像里是哪一版。

    py -3 tools/fw_version.py COM9                问正在跑的固件（寄存器 0x19C~0x19E）
    py -3 tools/fw_version.py --scan <bin>        扫镜像里的 "TTFW1;" 魔数
    py -3 tools/fw_version.py --scan-all          扫 .pio 里的 app/bl 镜像 + backup/ 全片备份

为什么两个模式都要：
  * 寄存器回答"代码真的在跑"，刷机工具的成功只证明 flash 内容对了；
  * 魔数回答"这片 flash / 这个文件里到底是哪一版"，设备连不上、或者只有 dump 的时候用得上。
"""

from __future__ import annotations

import argparse
import datetime
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

REG_GIT = 0x19C
REG_BUILD = 0x19D
REG_FLAGS = 0x19E
MAGIC = b"TTFW1;"


def query(port: str):
    """从运行中的固件读身份。读不到（旧固件没有这几个寄存器）返回 None。"""
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    from tt_host import TTHost                      # noqa: E402

    h = TTHost(port, verbose=False)
    try:
        git = h.read(REG_GIT)
        bld = h.read(REG_BUILD)
        flg = h.read(REG_FLAGS)
    finally:
        h.close()
    if git is None or bld is None:
        return None
    return {"git": git & 0xFFFFFFFF, "build": bld & 0xFFFFFFFF,
            "flags": (flg or 0) & 0xFFFFFFFF}


def describe(d: dict) -> str:
    when = datetime.datetime.fromtimestamp(d["build"]).strftime("%Y-%m-%d %H:%M:%S")
    dirty = "  **工作区脏：仓库里没有对应提交**" if d["flags"] & 1 else ""
    return f"git %07x  编译于 {when}{dirty}" % d["git"]


def scan(path: str):
    with open(path, "rb") as f:
        blob = f.read()
    out = []
    for m in re.finditer(re.escape(MAGIC), blob):
        end = blob.find(b"\x00", m.start(), m.start() + 96)
        if end < 0:
            end = m.start() + 96
        try:
            txt = blob[m.start():end].decode("utf-8", "replace")
        except Exception:
            txt = "?"
        out.append((m.start(), txt))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM9")
    ap.add_argument("--scan", default=None, help="在指定 bin/hex dump 里找 TTFW1; 标记")
    ap.add_argument("--scan-all", action="store_true")
    a = ap.parse_args()

    if a.scan_all:
        targets = sorted(glob.glob(os.path.join(ROOT, ".pio", "build", "*", "firmware.bin"))
                         + glob.glob(os.path.join(ROOT, "backup", "*.bin"))
                         + glob.glob(os.path.join(ROOT, ".pio", "*dfu_image.bin")))
        if not targets:
            print("没有可扫的镜像")
            return 1
        for t in targets:
            hits = scan(t)
            rel = os.path.relpath(t, ROOT)
            if not hits:
                print(f"{rel:56s}  无标记（旧固件 / 非本项目镜像）")
            for off, txt in hits:
                print(f"{rel:56s}  @0x{off:06X}  {txt}")
        return 0

    if a.scan:
        hits = scan(a.scan)
        if not hits:
            print(f"{a.scan}: 没有找到 {MAGIC!r} —— 这是加版本戳之前编的镜像")
            return 1
        for off, txt in hits:
            print(f"@0x{off:06X}  {txt}")
        return 0

    d = query(a.port)
    if d is None:
        print(f"{a.port}: 读不到 0x19C~0x19E —— 板上是加身份寄存器**之前**的固件")
        return 1
    print(f"{a.port}: {describe(d)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
