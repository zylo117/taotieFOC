# -*- coding: utf-8 -*-
"""
extra_version.py —— 每次编译把 git 版本 + 编译时刻写进 include/fw_build.h。

为什么要它：现在调试的第一难题是"板上跑的到底是哪一版、到底刷进去没有"。
光靠文件时间戳和肉眼比对不解决，必须让固件自己把身份报出来
（寄存器 0x19C/0x19D/0x19E、开机 UART 横幅、镜像里的 TTFW1; 魔数）。

为什么用"生成头文件"而不是 CPPDEFINES：
    -DFW_VERSION_STR=... 会让**所有**目标文件的编译命令行变化 → 每次编译全量重编。
    生成头只有 version_build.c 一个文件依赖它 → 增量编译只重编 1 个文件 + 重新链接。
"""

import datetime
import os
import subprocess

Import("env")  # noqa: F821

ROOT = env.subst("$PROJECT_DIR")  # noqa: F821


def _git(*args):
    try:
        r = subprocess.run(("git",) + args, cwd=ROOT, capture_output=True, timeout=15)
        return r.stdout.decode("utf-8", "replace").strip()
    except Exception:
        return ""


def main():
    sha = _git("rev-parse", "--short", "HEAD") or "nogit"
    branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "?"
    dirty = bool(_git("status", "--porcelain"))
    # 工作区脏不脏必须显式带出来：不干净就意味着"这块板子上的代码在仓库里没有对应提交"
    tag = sha + ("-dirty" if dirty else "")
    now = datetime.datetime.now().astimezone()
    iso = now.strftime("%Y-%m-%d %H:%M:%S %z")
    epoch = int(now.timestamp())
    try:
        git_num = int(sha[:7], 16) & 0xFFFFFFFF   # 7 个十六进制字符 = 28 位，塞得进 32 位寄存器
    except ValueError:
        git_num = 0

    ver = "%s @ %s (%s)" % (tag, iso, branch)
    body = """/** 由 extra_version.py 每次编译自动生成 —— 手改无意义，别提交冲突。 */
#ifndef FW_BUILD_H
#define FW_BUILD_H

#define FW_GIT_HASH      "%s"
#define FW_GIT_BRANCH    "%s"
#define FW_GIT_DIRTY     %d
#define FW_GIT_NUM       0x%08Xu
#define FW_BUILD_ISO     "%s"
#define FW_BUILD_EPOCH   %uLu
#define FW_VERSION_STR   "%s"

#endif
""" % (sha, branch, 1 if dirty else 0, git_num, iso, epoch, ver)

    dst = os.path.join(ROOT, "include", "fw_build.h")
    old = None
    if os.path.exists(dst):
        with open(dst, "r", encoding="utf-8") as f:
            old = f.read()
    if old != body:                    # 内容没变就不重写，省掉一次无谓的重编依赖传播
        with open(dst, "w", encoding="utf-8") as f:
            f.write(body)
    print("[version] %s" % ver)


main()
