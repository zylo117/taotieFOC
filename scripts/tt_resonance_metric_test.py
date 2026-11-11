"""共振指标对照实测（2026-11-11，定位网页扫描误判根因用）。

网页扫描暴露两个硬伤（用户实机观察）：
  A. 250RPM 明明平滑却报 4.16x「剧烈」，基线 60RPM σ 就有 2.76°——
     嫌疑：角度的时间戳来自 JS/上位机轮询，与 ~100Hz 遥测刷新之间存在
     相位抖动；匀速段角度 2160°/s(60RPM)~23400°/s(650RPM)，±1.3ms 抖动
     就能造出 σ=2.8°/11.5° —— 和实测吻合，量到的主要是抖动不是机械振动。
  B. 425~500RPM 转子打滑/转不动被报「平静」——去趋势把斜坡当速度减掉了，
     残差当然小；斜率才是失步证据，旧算法把斜率扔了。

新指标（本脚本验证对象）：跟随误差 FE(0x114) = 理论行程-编码器行程。
  * 匀速跟随：FE≈常数（固定跟随滞后），轮询时间抖动不产生残差；
  * 共振：转子围着命令位置大幅摆动，FE 残差 σ/pp 如实放大；
  * 失步打滑：FE 以 (1-实际/命令)×命令速率 爬升，|斜率| 即失步证据。
每点同时算 旧σ(角度)/新σ(FE)/角度估计转速/FE斜率/实测转速(0x120)，
用真机数据判哪种指标和手感一致。
"""
from __future__ import annotations

import os
import statistics
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tt_host import R_ENCODER_ANGLE_MDEG, TTHost  # noqa: E402

R_MOTION = 0x11F
R_START_RPM = 0x11A
R_MAX_RPM = 0x11B
R_ACCEL = 0x11C
R_PULSE_COUNT = 0x11D
R_MODE = 0x11E
R_ENABLE = 0x124
R_DISABLE = 0x125
R_QUICK_STOP = 0x163
R_STALL_CLEAR = 0x15F
R_STALL_DETECTED = 0x15D
R_SPEED_RPM = 0x120
R_FOLLOW_ERROR_DEG = 0x121   # 跟随误差 mdeg（POSITION_DEG 名字是历史的，值=编码器行程-命令行程，
                             # 快帧 ~100Hz）。0x114 是死寄存器（follow_error_ 从未赋值恒 0），勿用。
R_MICROSTEPS = 0x100
R_RESONANCE_BYPASS = 0x171
R_CLOSED_LOOP = 0x107

BASE_RPM = 60
POINTS = [250, 275, 300, 325, 350, 375, 400, 425, 450, 500, 525, 600]
ACCEL = 1500
WINDOW_S = 1.2
MAX_STALLS = 2


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def detrend(rows):
    """rows=[(t_deg_or_fe)]：返回 (σ, pp, slope_per_s)。"""
    if len(rows) < 10:
        return (float("nan"),) * 2 + (float("nan"),)
    trim = max(1, len(rows) // 10)
    rows = rows[trim: len(rows) - trim] or rows
    n = len(rows)
    mt = sum(r[0] for r in rows) / n
    ma = sum(r[1] for r in rows) / n
    cov = sum((r[0] - mt) * (r[1] - ma) for r in rows)
    var = sum((r[0] - mt) ** 2 for r in rows)
    slope = cov / var if var else 0.0
    res = [r[1] - (ma + slope * (r[0] - mt)) for r in rows]
    return statistics.pstdev(res), max(res) - min(res), slope


def sample(host, seconds):
    """同时采 角度(解卷绕→估计转速) 与 FE 0x121(解卷绕→纹波+斜率)。
    时间戳用墙钟轮询 —— 角度指标因此被 ±1ms 级抖动污染（正是要证明的），
    FE 指标对抖动免疫（匀速段 FE 斜率≈0，误差=斜率×δt≈0）。"""
    ang = []
    fe = []
    speeds = []
    t0 = time.time()
    prev_a = None
    unw_a = None
    prev_f = None
    unw_f = None
    while time.time() - t0 < seconds:
        for _ in host._pump(0.006):
            pass
        t = time.time() - t0
        a = host.latest.get(R_ENCODER_ANGLE_MDEG)
        f = host.latest.get(R_FOLLOW_ERROR_DEG)
        sp = host.latest.get(R_SPEED_RPM)
        if a is not None:
            deg = s32(a) / 1000.0
            if prev_a is None:
                unw_a = deg
            else:
                d = deg - prev_a
                while d > 180.0:
                    d -= 360.0
                while d < -180.0:
                    d += 360.0
                unw_a += d
            prev_a = deg
            ang.append((t, unw_a))
        if f is not None:
            fdeg = s32(f) / 1000.0
            if prev_f is None:
                unw_f = fdeg
            else:
                d = fdeg - prev_f
                while d > 180.0:
                    d -= 360.0
                while d < -180.0:
                    d += 360.0
                unw_f += d
            prev_f = fdeg
            fe.append((t, unw_f))
        if sp is not None:
            speeds.append(abs(s32(sp)))
    return ang, fe, speeds


def run_point(host, rpm):
    host.write(R_START_RPM, max(10, min(rpm, 120)))
    host.write(R_MAX_RPM, rpm)
    host.write(R_ACCEL, ACCEL)
    host.write(R_PULSE_COUNT, 0)
    host.write(R_MODE, 2)
    host.write(R_MOTION, 1)
    tol = max(15, rpm * 0.15)
    deadline = time.time() + min(6.0, 2.0 + rpm / ACCEL * 2.0)
    settled = False
    while time.time() < deadline:
        for _ in host._pump(0.05):
            pass
        sp = host.latest.get(R_SPEED_RPM)
        if sp is not None and abs(abs(s32(sp)) - rpm) <= tol:
            settled = True
            break
        if host.latest.get(R_STALL_DETECTED):
            break
    ang, fe, speeds = sample(host, WINDOW_S)
    stalled = bool(host.latest.get(R_STALL_DETECTED))
    if stalled:
        host.write(R_MOTION, 0)
        host.write(R_STALL_CLEAR, 1)
        time.sleep(0.2)
    else:
        host.write(R_QUICK_STOP, 1)
        time.sleep(0.3 + rpm / ACCEL)
    est_rpm = detrend(ang)[2] / 6.0 if len(ang) >= 10 else float("nan")
    med_speed = statistics.median(speeds) if speeds else float("nan")
    return detrend(ang), detrend(fe), est_rpm, med_speed, stalled, settled


step_deg = 0.0

def main():
    global step_deg
    host = TTHost("COM9", verbose=False)
    time.sleep(0.5)
    for _ in host._pump(0.5):
        pass
    usteps = host.read(R_MICROSTEPS) or 64
    step_deg = 360.0 / (200 * usteps)
    print(f"微步 {usteps} → 1 步 = {step_deg:.5f}°")
    host.write(R_CLOSED_LOOP, 0)
    host.write(R_STALL_CLEAR, 1)
    host.write(R_ENABLE, 1)
    host.write(R_RESONANCE_BYPASS, 1)
    stalls = 0
    try:
        (b_old_s, _, _), (b_fe_s, b_fe_pp, _), b_est, b_spd, b_st, b_ok = run_point(host, BASE_RPM)
        print(f"基线 {BASE_RPM}RPM：旧σ={b_old_s:.4f}°  新σ={b_fe_s:.4f}° (pp={b_fe_pp:.3f}°) "
              f"估计转速={b_est:.0f} 实测中位={b_spd:.0f} settled={b_ok}")
        print(f"\n{'RPM':>5} {'估计':>6} {'实测':>6} {'旧σ':>8} {'旧比':>6} | {'新σ':>8} {'新pp':>8} {'新比':>6} "
              f"{'FE斜率°/s':>10}  判定")
        results = []
        for rpm in POINTS:
            (o_s, o_pp, _), (f_s, f_pp, f_slope), est, spd, st, ok = run_point(host, rpm)
            if st:
                stalls += 1
            cmd_deg_s = rpm * 6.0
            slip = (est == est and est < rpm * 0.7) \
                or (f_slope == f_slope and abs(f_slope) > cmd_deg_s * 0.25) \
                or (spd == spd and spd < rpm * 0.6)
            ratio_old = o_s / b_old_s if b_old_s > 0 else float("nan")
            ratio_new = f_s / b_fe_s if b_fe_s > 0 else float("nan")
            verdict = "堵转" if st else ("失步/打滑" if slip else
                      ("剧烈" if ratio_new >= 3 else ("偏高" if ratio_new > 1.6 else "平静")))
            print(f"{rpm:>5} {est:>6.0f} {spd:>6.0f} {o_s:>8.4f} {ratio_old:>5.1f}x | "
                  f"{f_s:>8.4f} {f_pp:>8.3f} {ratio_new:>5.1f}x {f_slope:>10.1f}  {verdict}")
            results.append((rpm, ratio_new, slip or st))
            if stalls >= MAX_STALLS:
                print("!! 堵转次数到达上限，中止")
                break
        hot = [r for r in results if (r[1] == r[1] and r[1] >= 3.0) or r[2]]
        if hot:
            print(f"\n新指标热点：{[r[0] for r in hot]}")
        else:
            print("\n新指标未发现 ≥3x 热点")
    finally:
        try:
            host.write(R_MOTION, 0)
            host.write(R_RESONANCE_BYPASS, 0)
            host.write(R_DISABLE, 1)
        except Exception:
            pass
        host.close()


if __name__ == "__main__":
    main()
