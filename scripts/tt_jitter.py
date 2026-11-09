"""闭环抖动 / 短行程过冲 诊断。

用法：
    python scripts/tt_jitter.py jitter   # 恒速段 开环 vs 闭环 的编码器抖动
    python scripts/tt_jitter.py residual # 1/2/4 圈的末端残余误差

恒速段用速度模式（pulse_count=0 表示无限跑），开环/闭环只切 0x107。
抖动用 0x116（编码器角度）与 0x121（跟随误差）在 ~40Hz 采样上算标准差/峰峰值。
注意：0x116 本身有 ±0.03° 的传感器噪声，所以两种模式要对比着看。
"""

from __future__ import annotations

import os
import statistics
import sys
import time

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tt_host import R_ENCODER_ANGLE_MDEG, R_FOLLOW_ERR, TTHost  # noqa: E402


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def capture(host, seconds):
    """采样编码器角度与跟随误差。角度必须**解卷绕**：0x116 是 0~360°，
    匀速转几圈就会绕好几次，直接用会算出 100° 的假抖动。"""
    rows = []
    t0 = time.time()
    unwrapped = None
    prev = None
    while time.time() - t0 < seconds:
        for _ in host._pump(0.008):
            pass
        ang = host.latest.get(R_ENCODER_ANGLE_MDEG)
        err = host.latest.get(R_FOLLOW_ERR)
        if ang is not None and err is not None:
            deg = s32(ang) / 1000.0
            if prev is None:
                unwrapped = deg
            else:
                d = deg - prev
                while d > 180.0:
                    d -= 360.0
                while d < -180.0:
                    d += 360.0
                unwrapped += d
            prev = deg
            rows.append((time.time() - t0, unwrapped, s32(err) / 1000.0))
    return rows


def stats(values):
    if len(values) < 5:
        return (0.0, 0.0, 0.0)
    return (statistics.mean(values), statistics.pstdev(values), max(values) - min(values))


def jitter(host, rpm, comp, seconds=3.0):
    host.clear_stall()
    host.write(0x124, 1)
    host.write(0x107, comp)
    host.write(0x11A, max(10, rpm // 2))
    host.write(0x11B, rpm)
    host.write(0x11C, 1200)
    host.write(0x11D, 0)
    host.write(0x11E, 2)          # VELOCITY_FORWARD
    host.latest.clear()
    host.write(0x11F, 1)
    time.sleep(0.9)               # 加速到稳态
    rows = capture(host, seconds)
    host.write(0x11F, 0)
    time.sleep(0.15)
    host.write(0x125, 1)
    time.sleep(0.15)
    if len(rows) < 10:
        return None
    # 去掉首尾 10% 避免残余加速段
    rows = rows[len(rows) // 10: -len(rows) // 10 or None]
    ang = [r[1] for r in rows]
    err = [r[2] for r in rows]
    # 角度去趋势（匀速时角度线性增长），看偏离直线的抖动
    t = [r[0] for r in rows]
    n = len(t)
    mt = sum(t) / n
    ma = sum(ang) / n
    cov = sum((t[i] - mt) * (ang[i] - ma) for i in range(n))
    var = sum((x - mt) ** 2 for x in t)
    slope = cov / var if var else 0.0
    resid = [ang[i] - (ma + slope * (t[i] - mt)) for i in range(n)]
    return {
        "rows": len(rows),
        "rpm_est": slope * 60.0,
        "pos_std": statistics.pstdev(resid),
        "pos_pp": max(resid) - min(resid),
        "err_std": statistics.pstdev(err),
        "err_pp": max(err) - min(err),
        "err_mean": statistics.mean(err),
    }


def residual(host, rpm, turns, repeats=3):
    out = []
    for _ in range(repeats):
        host.clear_stall()
        host.write(0x124, 1)
        host.write(0x107, 1)
        r = host.run_motion(max(20, rpm // 2), rpm, 900, int(12800 * turns), 0, timeout=25)
        e = host.read(0x121)
        out.append(s32(e) / 1000.0 if e is not None else float("nan"))
        host.write(0x11F, 0)
        time.sleep(0.12)
        host.write(0x125, 1)
        time.sleep(0.2)
    return out, r


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "jitter"
    host = TTHost("COM9")
    try:
        print("微步=%d  当前PID=%s  滞后角前馈=%s" % (host.microsteps(), host.get_pid(), host.read(0x152)))
        if what == "jitter":
            for rpm in (100, 200):
                for comp in (0, 1):
                    r = jitter(host, rpm, comp)
                    if r:
                        print("  %3dRPM comp=%d : 实测%.1fRPM  位置抖动 std=%.4f° pp=%.4f° | "
                              "跟随误差 mean=%+.3f° std=%.3f° pp=%.3f°"
                              % (rpm, comp, r["rpm_est"], r["pos_std"], r["pos_pp"],
                                 r["err_mean"], r["err_std"], r["err_pp"]))
        elif what == "residual":
            for rpm in (60, 120, 200):
                for turns in (1, 2, 4):
                    vals, r = residual(host, rpm, turns)
                    print("  %3dRPM %d圈 : 末端残余 %s °（评分 %.3f°）"
                          % (rpm, turns, " / ".join("%+.3f" % v for v in vals),
                             r["score_mdeg"] / 1000.0))
        else:
            print("未知命令")
            return 2
    finally:
        try:
            host.write(0x11F, 0)
            host.disable()
        except Exception:
            pass
        host.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
