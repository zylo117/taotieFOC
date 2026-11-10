"""共振速度区间自动测试（用户规格 2026-11）。

原理：
  1) 超低速基准（默认 60 RPM）采一段编码器角度，去趋势后算"纹波"（σ 与峰峰值）。
  2) 从（共振带下限 - 边缘）到（上限 + 边缘）逐点升速（默认带 ±50 RPM，步长 25 RPM），
     每点等稳态后采同样长度的窗口，纹波与基准比较。
  3) 共振表现为**突然开始剧烈抖动 / 突然结束**：用比例阈值找跳变 ——
     连续 2 个点 ratio >= START_RATIO(3x) 记为带起点，之后回落 <= END_RATIO(1.6x)
     连续 2 个点记为带终点。
  4) 剧烈共振有不小概率导致堵转：每点都盯 0x15D/故障位图，堵转就立即停车、
     清锁存、把该速度标记进禁区并跳过 ±skip 内的邻点继续测；连炸 3 次带结论中止。

安全措施：
  * 测试前必须显式确认（--confirm 或交互输入 RESONANCE），并再次核对现场。
  * 写 0x171=1 旁路固件共振钳位（否则带内目标会被钳到带下限，测不到共振本身），
    结束（含异常）必恢复 0x171=0。
  * 每个点测完先"缓停"（0x163 减速斜坡）再升速，不来回猛冲。
  * --apply 才把结果写 0x16F/0x170；默认只出报告。
  * 结束时留在内部规划模式（同 tt_autotune 约定），并失能驱动器。

用法：
  python scripts/tt_resonance.py --port COM9 [--lo 300 --hi 600] [--edge 50]
                                 [--step 25] [--seconds 1.2] [--apply] [--confirm RESONANCE]
"""

from __future__ import annotations

import argparse
import json
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

from tt_host import (R_ENCODER_ANGLE_MDEG, R_SPEED_RPM, TTHost)  # noqa: E402

R_MOTION = 0x11F
R_START_RPM = 0x11A
R_MAX_RPM = 0x11B
R_ACCEL = 0x11C
R_PULSE_COUNT = 0x11D
R_MODE = 0x11E
R_ENABLE = 0x124
R_DISABLE = 0x125
R_QUICK_STOP = 0x163
R_STALL_DETECTED = 0x15D
R_STALL_CLEAR = 0x15F
R_FAULT = 0x108
R_RESONANCE_LO = 0x16F
R_RESONANCE_HI = 0x170
R_RESONANCE_BYPASS = 0x171
R_MOTION_SOURCE = 0x16D
R_EMERGENCY_STOP = 0x16E

START_RATIO = 3.0     # 连续 2 点纹波 >= 3x 基准 => 认为共振开始
END_RATIO = 1.6       # 之后连续 2 点回落到 <= 1.6x => 共振结束
MAX_STALLS = 3        # 整轮堵转超过 3 次：带结论中止


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def capture_unwrapped(host, seconds):
    """采 0x116 编码器角度（mdeg），环绕解卷绕。同 tt_jitter.capture。"""
    rows = []
    t0 = time.time()
    unwrapped = None
    prev = None
    while time.time() - t0 < seconds:
        for _ in host._pump(0.008):
            pass
        ang = host.latest.get(R_ENCODER_ANGLE_MDEG)
        if ang is None:
            continue
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
        rows.append((time.time() - t0, unwrapped))
    return rows


def detrend_ripple(rows):
    """对角度去线性趋势（匀速段角度≈直线），返回 (resid_std_deg, resid_pp_deg, slope_rpm)。"""
    if len(rows) < 10:
        return (float("nan"), float("nan"), float("nan"))
    trim = max(1, len(rows) // 10)
    rows = rows[trim: len(rows) - trim] or rows
    n = len(rows)
    mt = sum(r[0] for r in rows) / n
    ma = sum(r[1] for r in rows) / n
    cov = sum((r[0] - mt) * (r[1] - ma) for r in rows)
    var = sum((r[0] - mt) ** 2 for r in rows)
    slope = cov / var if var else 0.0     # deg/s
    resid = [r[1] - (ma + slope * (r[0] - mt)) for r in rows]
    return (statistics.pstdev(resid), max(resid) - min(resid), slope * 60.0)


def stall_active(host):
    stall = host.read(R_STALL_DETECTED)
    fault = host.read(R_FAULT)
    return (stall or 0) & 1 or (fault or 0) & 0x08


def run_point(host, rpm, seconds, accel):
    """速度模式无限跑 rpm，等稳态，采窗口，缓停。返回 (std, pp, est_rpm, stalled)。"""
    host.write(R_START_RPM, max(10, min(rpm, 120)))
    host.write(R_MAX_RPM, rpm)
    host.write(R_ACCEL, accel)
    host.write(R_PULSE_COUNT, 0)      # 无限行程
    host.write(R_MODE, 2)             # VELOCITY_FORWARD
    host.latest.clear()
    host.write(R_MOTION, 1)
    # 等稳态：加速时间 + 裕量（上限 4s）
    settle = min(4.0, abs(rpm) / max(accel, 1.0) * 1.6 + 0.8)
    t0 = time.time()
    while time.time() - t0 < settle:
        for _ in host._pump(0.05):
            pass
        sp = host.latest.get(R_SPEED_RPM)
        if sp is not None and abs(abs(s32(sp)) - rpm) <= max(15.0, rpm * 0.15):
            break
    rows = capture_unwrapped(host, seconds)
    std, pp, est = detrend_ripple(rows)
    stalled = stall_active(host)
    if not stalled:
        host.write(R_QUICK_STOP, 1)   # 缓停（减速斜坡）
    else:
        host.write(R_MOTION, 0)       # 已堵转：立即停 + 清锁存
        time.sleep(0.2)
        host.write(R_STALL_CLEAR, 1)
    time.sleep(0.25)
    return std, pp, est, stalled


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--lo", type=int, default=None, help="共振带下限（默认读固件 0x16F）")
    ap.add_argument("--hi", type=int, default=None, help="共振带上限（默认读固件 0x170）")
    ap.add_argument("--edge", type=int, default=50, help="带外扩展扫描边距 RPM（默认 50）")
    ap.add_argument("--step", type=int, default=25, help="扫速步长 RPM（默认 25）")
    ap.add_argument("--seconds", type=float, default=1.2, help="每点采样窗口秒数")
    ap.add_argument("--base-rpm", type=int, default=60, help="超低速纹波基准转速（默认 60）")
    ap.add_argument("--accel", type=int, default=1500, help="测试加速度 RPM/s（默认 1500）")
    ap.add_argument("--apply", action="store_true", help="测完把结果写 0x16F/0x170（默认只报告）")
    ap.add_argument("--confirm", default=None, help="现场安全确认后输入 RESONANCE（或在交互提示中输入）")
    ap.add_argument("--out", default="resonance_result.json")
    args = ap.parse_args()

    print("== taotieFOC 共振速度区间自动测试 ==")
    host = TTHost(args.port)
    bypass_on = False
    try:
        src = host.read(R_MOTION_SOURCE)
        if src is not None and (src & 1):
            if not host.write(R_MOTION_SOURCE, 0):
                print("!! 无法切换到内部规划模式（0x16D），退出")
                return 2
            print("运动源：外部 -> 内部规划（结束留在内部模式）")

        lo = args.lo if args.lo is not None else host.read(R_RESONANCE_LO)
        hi = args.hi if args.hi is not None else host.read(R_RESONANCE_HI)
        if lo is None or hi is None:
            print("!! 读不到共振带寄存器（0x16F/0x170）——固件版本过旧？")
            return 2
        if not (0 < lo < hi):
            print("!! 共振带配置无效 lo=%s hi=%s（应为 0<lo<hi，或显式传参）" % (lo, hi))
            return 2

        sweep = list(range(max(20, int(lo) - args.edge), int(hi) + args.edge + 1, args.step))
        print("固件共振带 [%d, %d] RPM；扫速 %d ~ %d，步长 %d，每点窗口 %.1fs"
              % (lo, hi, sweep[0], sweep[-1], args.step, args.seconds))
        print("注意：测试期间固件共振钳位被旁路（0x171=1），电机会**真的进入**共振区，")
        print("      剧烈抖动/啸叫/堵转都是预期现象。请确认电机空载或安全固定、周围没人碰机构。")
        conf = args.confirm if (args.confirm or "").upper() == "RESONANCE" else None
        if conf is None:
            conf = input("输入 RESONANCE 确认开始（其他任意输入取消）: ").strip().upper()
        if conf != "RESONANCE":
            print("已取消。")
            return 1

        host.write(R_STALL_CLEAR, 1)
        host.write(R_ENABLE, 1)
        # 开环扫：纹波反映机械/电机本体 + 步距量化的共振，不被 PID 阻尼/激励污染，
        # 基准与扫描同一口径，比值才有意义。
        host.write(0x107, 0)
        host.write(R_RESONANCE_BYPASS, 1)
        bypass_on = True

        # ---- 超低速纹波基准 ----
        print("\n基准：超低速 %d RPM ..." % args.base_rpm)
        b_std, b_pp, b_est, b_stalled = run_point(host, args.base_rpm, args.seconds, args.accel)
        if b_stalled or not (b_std == b_std) or b_std <= 0.0:
            print("!! 超低速基准都堵转/采不到有效纹波（std=%s），先解决基础问题再测共振" % b_std)
            return 2
        print("  基准纹波：std=%.4f° pp=%.4f°（实测 %.0f RPM）" % (b_std, b_pp, b_est))

        # ---- 扫速 ----
        points = []
        stalls = 0
        skip_until = -1.0
        print("\n速度    实测    纹波std    纹波pp    比例    状态")
        for rpm in sweep:
            if rpm <= skip_until:
                print("%5d  （堵转禁区邻近，跳过）" % rpm)
                continue
            std, pp, est, stalled = run_point(host, rpm, args.seconds, args.accel)
            ratio = (std / b_std) if (std == std and b_std > 0) else float("nan")
            state = "堵转" if stalled else ("剧烈" if ratio >= START_RATIO else
                                            ("偏高" if ratio > END_RATIO else "平静"))
            print("%5d  %6.0f  %8.4f  %8.4f  %6.1fx  %s" % (rpm, est, std, pp, ratio, state))
            points.append({"rpm": rpm, "est_rpm": est, "std_deg": std, "pp_deg": pp,
                           "ratio": ratio, "stalled": bool(stalled)})
            if stalled:
                stalls += 1
                skip_until = rpm + max(2 * args.step, 50)   # 禁区附近不硬闯
                print("       堵转：已清锁存，跳过 <= %d RPM 的邻近点继续" % skip_until)
                if stalls >= MAX_STALLS:
                    print("!! 堵转次数达到 %d，中止扫描，用已有数据出结论" % MAX_STALLS)
                    break

        # ---- 分离骤起骤止区间 ----
        band_lo = band_hi = None
        hot = 0
        for p in points:
            if p["stalled"]:
                hot = max(hot, 2)   # 堵转点直接算剧烈（比测出来的数更有说服力）
            if p["ratio"] == p["ratio"] and p["ratio"] >= START_RATIO:
                hot += 1
            else:
                hot = 0
            if hot >= 2 and band_lo is None:
                # 起点 = 连续两点中的第一个（回看上一条）
                idx = points.index(p)
                band_lo = points[idx - 1]["rpm"] if idx > 0 else p["rpm"]
        if band_lo is not None:
            for p in points:
                if p["rpm"] < band_lo:
                    continue
                if p["stalled"]:
                    band_hi = p["rpm"]   # 最后一处确定证据
                    continue
                if p["ratio"] == p["ratio"] and p["ratio"] <= END_RATIO and p["rpm"] > (band_lo + args.step):
                    band_hi = p["rpm"] - args.step
                    break
            if band_hi is None and points:
                last = points[-1]
                if last["stalled"] or (last["ratio"] == last["ratio"] and last["ratio"] > END_RATIO):
                    band_hi = last["rpm"]
        if band_lo is not None and (band_hi is None or band_hi <= band_lo):
            band_hi = band_lo + args.step

        print("\n== 结论 ==")
        if band_lo is None:
            print("本次扫描未发现 >= %.1fx 基准的骤发剧烈抖动区间；保持现设置或加严阈值/加密步长。"
                  % START_RATIO)
        else:
            print("检测到共振区间约 [%d, %d] RPM（固件现值 [%d, %d]）" % (band_lo, band_hi, lo, hi))
            if args.apply:
                ok_lo = host.write(R_RESONANCE_LO, int(band_lo))
                ok_hi = host.write(R_RESONANCE_HI, int(band_hi))
                print("写入 0x16F/0x170：%s（如需掉电保持请在网页点『写入参数到 Flash』或写 0x154）"
                      % ("成功" if (ok_lo and ok_hi) else "失败"))
            else:
                print("（未写固件；确认结果后加 --apply 重跑，或手动在网页改共振带）")

        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"timestamp": time.strftime("%Y-%m-%d %H:%M:%S"), "port": args.port,
                       "baseline": {"rpm": args.base_rpm, "std_deg": b_std, "pp_deg": b_pp},
                       "band_before": {"lo": lo, "hi": hi},
                       "band_detected": {"lo": band_lo, "hi": band_hi},
                       "applied": bool(args.apply and band_lo is not None),
                       "stalls": stalls, "points": points}, f, ensure_ascii=False, indent=2)
        print("报告：%s" % os.path.abspath(args.out))
        return 0
    finally:
        try:
            host.write(R_MOTION, 0)
        except Exception:
            pass
        if bypass_on:
            try:
                host.write(R_RESONANCE_BYPASS, 0)   # 钳位必须恢复，不然共振保护就没了
            except Exception:
                print("!! 0x171 旁路恢复失败 —— 请手动写 0x171=0，否则固件共振保护不生效！")
        try:
            host.disable()
        except Exception:
            pass
        host.close()


if __name__ == "__main__":
    raise SystemExit(main())
