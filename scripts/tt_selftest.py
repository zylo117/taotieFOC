"""taotieFOC 上台自检：行程比例 / 起步斜坡 / 匀速抖动 / 堵转保护。

用法：
    python scripts/tt_selftest.py ratio      # 1 圈开环 -> 编码器实际角度（查细分口径）
    python scripts/tt_selftest.py ramp       # 20->200RPM 梯形，打印速度包络
    python scripts/tt_selftest.py steady     # 100RPM 匀速 3s，统计速度纹波/跟随误差
    python scripts/tt_selftest.py stall      # 堵转保护（会让电机顶住，先别用）

COM14 的调试日志会被同时抓下来（带时间戳），关键行是 [SETTLE] / [STALL] / [SAFETY]。
"""

from __future__ import annotations

import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial

from tt_host import (
    MODE_POS_FWD,
    MODE_VEL_FWD,
    R_FOLLOW_ERR,
    R_MOTION,
    R_SPEED_RPM,
    TTHost,
)


class LogTap:
    """COM14 日志抓取线程：把每行带时间戳存起来，测试结束时打印。"""

    def __init__(self, port="COM14"):
        self.lines = []
        self._stop = False
        self._t0 = time.time()
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                n = self.ser.in_waiting
                if n:
                    buf += self.ser.read(n)
            except Exception:
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip()
                if text:
                    self.lines.append((time.time() - self._t0, text))

    def stop(self):
        self._stop = True
        time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass

    def dump(self, keys=("[SETTLE]", "[STALL]", "[SAFETY]", "[PARAM]", "[RAMP]", "[LOAD", "[LAG")):
        for t, text in self.lines:
            if any(k in text for k in keys):
                print("  LOG %6.2fs  %s" % (t, text))


def capture(host, seconds, tag=""):
    """采样遥测，返回 [(t, speed_rpm, err_mdeg, motion)]。"""
    rows = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        for _ in host._pump(0.01):
            pass
        rows.append((time.time() - t0,
                     host.latest.get(R_SPEED_RPM),
                     host.latest.get(R_FOLLOW_ERR),
                     host.latest.get(R_MOTION)))
    return rows


def print_envelope(rows, step_s=0.02):
    """按时间桶打印速度包络（同一桶里取最大/最小，避免刷屏）。"""
    if not rows:
        return
    buckets = {}
    for t, spd, err, run in rows:
        key = int(t / step_s)
        spd = 0 if spd is None else (spd - 0x100000000 if spd & 0x80000000 else spd)
        err = 0 if err is None else (err - 0x100000000 if err & 0x80000000 else err)
        lo, hi, e, r = buckets.get(key, (spd, spd, err, run))
        buckets[key] = (min(lo, spd), max(hi, spd), err, run)
    for key in sorted(buckets):
        lo, hi, err, run = buckets[key]
        print("  t=%5.3f  speed %5d..%-5d rpm  err %8.1f mdeg  run=%s"
              % (key * step_s, lo, hi, err / 1000.0, run))


class Wall:
    def __init__(self, name):
        self.name = name
        self.t = time.time()

    def mark(self, text):
        print("  [%s +%5.2fs] %s" % (self.name, time.time() - self.t, text))


def cmd_ratio(host):
    spr = 200 * host.microsteps()
    print("== 行程比例：开环 1 圈 = %d 微步 ==" % spr)
    before = host.read(0x116)
    host.enable()
    host.write(0x107, 0)          # 关补偿，纯开环
    res = host.run_motion(20, 120, 600, spr, MODE_POS_FWD, timeout=20)
    time.sleep(0.3)
    after = host.read(0x116)
    host.disable()
    if before is None or after is None:
        print("  读不到编码器角度")
        return
    delta = (after - before) / 1000.0
    if delta < 0:
        delta += 360.0
    print("  脉冲 %d，编码器转过 %.3f°（期望 360°）" % (spr, delta))
    print("  比例 = %.4f   -> 实际微步/圈 ≈ %.0f" % (delta / 360.0, spr * 360.0 / max(delta, 1e-6)))
    print("  评分 %.3f°，样本 %d" % (res["score_mdeg"] / 1000.0, res["samples"]))


def cmd_ramp(host):
    spr = 200 * host.microsteps()
    print("== 起步斜坡：20 -> 200RPM，accel=300RPM/s，0.5 圈 ==")
    host.enable()
    host.write(0x107, 1)
    host.write(0x11A, 20)
    host.write(0x11B, 200)
    host.write(0x11C, 300)
    host.write(0x11D, int(spr * 0.5))
    host.write(0x11E, MODE_POS_FWD)
    host.latest.clear()
    host.write(0x11F, 1)
    rows = capture(host, 2.5)
    host.write(0x11F, 0)
    time.sleep(0.3)
    host.disable()
    print_envelope(rows)


def cmd_steady(host):
    spr = 200 * host.microsteps()
    print("== 匀速 100RPM（速度模式，连续跑）：速度纹波 / 跟随误差 ==")
    host.enable()
    host.write(0x107, 1)
    host.write(0x11A, 60)
    host.write(0x11B, 100)
    host.write(0x11C, 1200)
    host.write(0x11D, 0)          # 0 = 无限跑
    host.write(0x11E, MODE_VEL_FWD)
    host.latest.clear()
    host.write(0x11F, 1)
    time.sleep(0.8)               # 让它加速到稳态
    rows = capture(host, 3.0)
    host.write(0x11F, 0)
    time.sleep(0.3)
    host.disable()
    speeds = []
    errs = []
    for _, spd, err, run in rows:
        if spd is None or err is None:
            continue
        spd = spd - 0x100000000 if spd & 0x80000000 else spd
        err = err - 0x100000000 if err & 0x80000000 else err
        if run == 1:
            speeds.append(spd)
            errs.append(err / 1000.0)
    if not speeds:
        print("  没采到运行中的数据")
        return
    print("  转速 n=%d 均值 %.2f 标准差 %.2f min %d max %d 峰峰 %d rpm"
          % (len(speeds), statistics.mean(speeds), statistics.pstdev(speeds),
             min(speeds), max(speeds), max(speeds) - min(speeds)))
    print("  跟随误差 均值 %+.3f° 标准差 %.3f° 峰峰 %.3f°"
          % (statistics.mean(errs), statistics.pstdev(errs), max(errs) - min(errs)))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "ramp"
    tap = LogTap()
    host = TTHost("COM9")
    print("微步=%d，PID=%s，故障=0x%02X" % (host.microsteps(), host.get_pid(), host.fault_flags()))
    try:
        host.clear_stall()
        if what == "ratio":
            cmd_ratio(host)
        elif what == "ramp":
            cmd_ramp(host)
        elif what == "steady":
            cmd_steady(host)
        else:
            print("未知测试：%s" % what)
            return 2
    finally:
        try:
            host.write(0x11F, 0)
            host.disable()
        except Exception:
            pass
        host.close()
        time.sleep(0.4)
        tap.dump()
        tap.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
