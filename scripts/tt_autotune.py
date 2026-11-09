"""taotieFOC 自动 PID 整定（串口版，COM9 = USB CDC）。

整定对象：位置环 + 速度环，三种运动模式（规划位置 / 往复急转弯 / 急加速）共用一组参数。
评价指标：固件在 20kHz 控制环里算的评分（0x132），它已经是
    RMS(滞后角补偿后误差) + 0.35*峰值 + 后半程增长惩罚
所以既惩罚抖动，也惩罚超调/掉步，还惩罚"越跑越偏"。

安全约束（全部硬编码，不允许脚本参数绕过）：
  - 转速上限 250RPM（本机 300~800RPM 有剧烈共振，先低速整定）
  - 每个试验 = 使能 -> 跑完 -> 等停 -> 失能；绝不提前失能（会丢尾脉冲）
  - 每个试验后读 0x108/0x15D：堵转/故障立即中止，不用坏数据评分
  - 试验之间冷却 250ms，避免连续大电流积热

用法：
    python scripts/tt_autotune.py                     # 默认 COM9
    python scripts/tt_autotune.py --port COM9 --quick # 少跑几组
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tt_host import (
    MODE_ALT_FWD,
    MODE_POS_FWD,
    MODE_POS_REV,
    R_TUNE_PEAK,
    R_TUNE_SCORE,
    R_TUNE_SAMPLES,
    TTHost,
)

# 搜索参数（实际增益口径）。上下限与固件 maxPidGain() 一致。
GAIN_KEYS = ("pos_kp", "pos_ki", "pos_kd", "vel_kp", "vel_ki", "vel_kd")
GAIN_LIMITS = {
    "pos_kp": (0.0, 1000.0),
    "pos_ki": (0.0, 1000.0),
    "pos_kd": (0.0, 100.0),
    "vel_kp": (0.0, 10.0),
    "vel_ki": (0.0, 10.0),
    "vel_kd": (0.0, 1.0),
}
# 每一轮按这个顺序试：先速度环（抖动/超调的主因），再位置环，最后积分项。
SEARCH_ORDER = ("vel_kp", "pos_kp", "pos_kd", "vel_kd", "pos_ki", "vel_ki")

MAX_TUNE_RPM = 250.0
COOLDOWN_S = 0.25


class Profile:
    def __init__(self, name, mode, start_rpm, max_rpm, accel_rpm_s, turns, weight=1.0):
        self.name = name
        self.mode = mode
        self.start_rpm = start_rpm
        self.max_rpm = max_rpm
        self.accel_rpm_s = accel_rpm_s
        self.turns = turns
        self.weight = weight


def build_profiles(max_rpm: float, quick: bool):
    """按"慢速 / 中速 / 高速 / 反向 / 急加速 / 急转弯"六类工况组织。

    max_rpm 会被硬钉在 250 以内（用户要求先低速整定，避开 300~800RPM 共振）。
    """
    m = min(max_rpm, MAX_TUNE_RPM)
    p = [
        Profile("慢速正向", MODE_POS_FWD, max(5, m * 0.15), max(10, m * 0.25), max(60, m * 1.2), 1.0),
        Profile("中速正向", MODE_POS_FWD, max(10, m * 0.35), max(20, m * 0.65), max(120, m * 2.4), 1.5),
        Profile("高速正向", MODE_POS_FWD, max(20, m * 0.5), m, max(200, m * 5.0), 2.0),
        Profile("中速反向", MODE_POS_REV, max(10, m * 0.35), max(20, m * 0.65), max(120, m * 2.4), 1.5),
        Profile("急加速", MODE_POS_FWD, max(10, m * 0.30), max(20, m * 0.85), max(600, m * 14.0), 2.0),
        Profile("急转弯往返", MODE_ALT_FWD, max(10, m * 0.35), max(20, m * 0.75), max(600, m * 14.0), 1.0),
    ]
    if quick:
        keep = {"中速正向", "高速正向", "急加速"}
        p = [x for x in p if x.name in keep]
    return p


def round_gain(value: float, key: str) -> float:
    lo, hi = GAIN_LIMITS[key]
    v = max(lo, min(hi, value))
    if v >= 10.0:
        return round(v, 2)
    if v >= 1.0:
        return round(v, 3)
    return round(v, 4)


def gains_tuple(g: dict) -> tuple:
    return (g["pos_kp"], g["pos_ki"], g["pos_kd"], g["vel_kp"], g["vel_ki"], g["vel_kd"])


def fmt_gains(g: dict) -> str:
    return ("位置 Kp=%.3f Ki=%.3f Kd=%.3f | 速度 Kp=%.4f Ki=%.4f Kd=%.5f"
            % (g["pos_kp"], g["pos_ki"], g["pos_kd"], g["vel_kp"], g["vel_ki"], g["vel_kd"]))


class Tuner:
    def __init__(self, host: TTHost, profiles, log=print):
        self.host = host
        self.profiles = profiles
        self.log = log
        self.spr = 200 * host.microsteps()
        self.trials = 0
        self.history = []

    # ---------------- 单次试验 ----------------
    def run_profile(self, profile: Profile, gains: dict) -> dict:
        h = self.host
        self.trials += 1
        h.set_pid(gains["pos_kp"], gains["pos_ki"], gains["pos_kd"],
                  gains["vel_kp"], gains["vel_ki"], gains["vel_kd"])
        h.enable()
        pulse_count = max(2, int(round(profile.turns * self.spr)))
        result = h.run_motion(profile.start_rpm, profile.max_rpm, profile.accel_rpm_s,
                              pulse_count, profile.mode)
        # 【阶段收尾】等运动真的结束之后再失能。run_motion 已经等到 0x11F=0（沉降完成）。
        h.write(0x11F, 0)
        time.sleep(0.12)
        h.disable()
        time.sleep(COOLDOWN_S)

        stall = h.stall()
        faults = h.fault_flags()
        result["stall"] = stall
        result["faults"] = faults
        result["profile"] = profile.name
        if stall or (faults & 0x01) or (faults & 0x02):
            raise RuntimeError(f"{profile.name}: 固件报故障（stall={stall}, fault=0x{faults:02X}），中止整定")
        if not result["finished"]:
            raise RuntimeError(f"{profile.name}: 运动未在超时内结束（started={result['started']}）")
        if result["score_mdeg"] is None or result["score_mdeg"] >= 0xFFFFFFFF:
            raise RuntimeError(f"{profile.name}: 固件评分无效（samples={result['samples']}）")
        if result["samples"] < 10:
            raise RuntimeError(f"{profile.name}: 样本太少（{result['samples']}）")
        return result

    # ---------------- 一组参数的综合评分 ----------------
    def evaluate(self, gains: dict, profiles=None) -> dict:
        profiles = profiles or self.profiles
        scores = []
        details = []
        for p in profiles:
            r = self.run_profile(p, gains)
            score = r["score_mdeg"] / 1000.0
            scores.append((score, p.weight))
            details.append({
                "profile": p.name,
                "score_deg": score,
                "peak_deg": r["peak_mdeg"] / 1000.0,
                "samples": r["samples"],
                "elapsed_s": round(r["elapsed_s"], 2),
            })
            self.log("      %-8s 评分 %6.3f°  峰值 %6.3f°  样本 %5d  %4.2fs"
                     % (p.name, score, r["peak_mdeg"] / 1000.0, r["samples"], r["elapsed_s"]))
        total_w = sum(w for _, w in scores) or 1.0
        mean = sum(s * w for s, w in scores) / total_w
        worst = max(s for s, _ in scores)
        # 综合分 = 加权平均 + 0.25*最差项：只压平均会让某一工况（通常是急转弯）烂掉。
        combined = mean + 0.25 * worst
        entry = {"gains": dict(gains), "mean_deg": mean, "worst_deg": worst,
                 "combined": combined, "details": details}
        self.history.append(entry)
        return entry

    # ---------------- 搜索 ----------------
    def search(self, start_gains: dict, rounds: int, step: float) -> dict:
        best = dict(start_gains)
        self.log("  基准：%s" % fmt_gains(best))
        base = self.evaluate(best)
        best_score = base["combined"]
        self.log("  基准综合分 %.3f°（平均 %.3f° / 最差 %.3f°）"
                 % (best_score, base["mean_deg"], base["worst_deg"]))

        for r in range(rounds):
            improved = False
            self.log("  ── 第 %d 轮（步进 ×%.2f）──" % (r + 1, step))
            for key in SEARCH_ORDER:
                center = best[key]
                lo, hi = GAIN_LIMITS[key]
                factor = step
                candidates = []
                up = center * factor if center > 1e-9 else 0.05
                down = center / factor if center > 1e-9 else 0.0
                if up <= hi:
                    candidates.append(up)
                if down >= lo and abs(down - center) > 1e-9:
                    candidates.append(down)
                for cand in candidates:
                    trial = dict(best)
                    trial[key] = round_gain(cand, key)
                    if abs(trial[key] - center) < 1e-9:
                        continue
                    self.log("    %s: %.4f -> %.4f" % (key, center, trial[key]))
                    try:
                        res = self.evaluate(trial)
                    except RuntimeError as exc:
                        self.log("      跳过：%s" % exc)
                        continue
                    if res["combined"] < best_score - 1e-4:
                        best_score = res["combined"]
                        best = trial
                        improved = True
                        self.log("      ✔ 新最佳 %.3f°（平均 %.3f° / 最差 %.3f°）%s"
                                 % (best_score, res["mean_deg"], res["worst_deg"], fmt_gains(best)))
                    else:
                        self.log("      ✘ %.3f°（平均 %.3f° / 最差 %.3f°）"
                                 % (res["combined"], res["mean_deg"], res["worst_deg"]))
            if not improved:
                self.log("  本轮无改善，搜索结束")
                break
        return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--max-rpm", type=float, default=240.0)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default="autotune_result.json")
    args = ap.parse_args()

    print("== taotieFOC 自动 PID 整定 ==")
    print("端口 %s，转速上限 %.0fRPM（硬上限 %.0f）" % (args.port, min(args.max_rpm, MAX_TUNE_RPM), MAX_TUNE_RPM))
    host = TTHost(args.port)
    try:
        microsteps = host.microsteps()
        faults = host.fault_flags()
        pid = host.get_pid()
        print("微步=%d（%d 微步/圈），故障位图=0x%02X，当前 PID=%s" % (microsteps, 200 * microsteps, faults, pid))
        if pid is None:
            print("!! 读不到 PID 参数，检查固件/端口")
            return 2
        if faults & 0x04:
            print("!! 输出已被封锁（fault bit2）。先排查故障，或写 0x15F=1 清堵转锁存。")
            return 2
        host.clear_stall()
        host.set_pid(*pid)

        # 编码器活性检查：遥测里的角度必须在变（手转/上次运动残留均可）
        before = host.latest.get(0x116)
        time.sleep(0.6)
        after = host.latest.get(0x116)
        if before is None or after is None:
            print("!! 没收到编码器遥测（0x116），检查 USB CDC 连接")
            return 2

        profiles = build_profiles(args.max_rpm, args.quick)
        print("工况：%s" % "、".join("%s(%.0f→%.0fRPM, %0.fRPM/s)" %
                                    (p.name, p.start_rpm, p.max_rpm, p.accel_rpm_s) for p in profiles))

        tuner = Tuner(host, profiles)
        start = {"pos_kp": pid[0], "pos_ki": pid[1], "pos_kd": pid[2],
                 "vel_kp": pid[3], "vel_ki": pid[4], "vel_kd": pid[5]}
        best = tuner.search(start, args.rounds, args.step)

        print("\n== 全工况验证 ==")
        final = tuner.evaluate(best)
        print("最终：%s" % fmt_gains(best))
        print("综合分 %.3f°（平均 %.3f° / 最差 %.3f°）" % (final["combined"], final["mean_deg"], final["worst_deg"]))

        # 写入 + 存 Flash
        host.set_pid(best["pos_kp"], best["pos_ki"], best["pos_kd"],
                     best["vel_kp"], best["vel_ki"], best["vel_kd"])
        time.sleep(0.2)
        readback = host.get_pid()
        print("回读 PID=%s" % (readback,))
        saved = host.save_params()
        print("写入 Flash：%s" % ("成功" if saved else "失败"))

        report = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "port": args.port,
            "microsteps": microsteps,
            "max_rpm": min(args.max_rpm, MAX_TUNE_RPM),
            "profiles": [p.__dict__ for p in profiles],
            "start_gains": start,
            "best_gains": best,
            "final": final,
            "trials": tuner.trials,
            "saved_to_flash": bool(saved),
            "history": tuner.history,
        }
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("报告已写入 %s" % os.path.abspath(args.out))
        return 0
    finally:
        try:
            host.write(0x11F, 0)
            host.disable()
        except Exception:
            pass
        host.close()


if __name__ == "__main__":
    raise SystemExit(main())
