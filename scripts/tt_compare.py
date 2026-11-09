"""A/B 比较两组候选增益：全工况、多次重复、取平均。

整定器最后两名候选常常差在噪声里（0.879 vs 0.881）。用 3 次重复 + 全部 6 个工况
再比一次，选更稳的那组写进 Flash。用法：

    python scripts/tt_compare.py --repeats 3
"""

import argparse
import json
import os
import sys
import time

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tt_autotune import Tuner, build_profiles, fmt_gains  # noqa: E402
from tt_host import TTHost  # noqa: E402

CANDIDATES = {
    "A_120_20_0__0.3_0_0": {"pos_kp": 120.0, "pos_ki": 20.0, "pos_kd": 0.0,
                            "vel_kp": 0.3, "vel_ki": 0.0, "vel_kd": 0.0},
    "B_80_20_0.05__0.2_0_0": {"pos_kp": 80.0, "pos_ki": 20.0, "pos_kd": 0.05,
                              "vel_kp": 0.2, "vel_ki": 0.0, "vel_kd": 0.0},
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--max-rpm", type=float, default=240.0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--out", default="autotune_compare.json")
    args = ap.parse_args()

    host = TTHost(args.port)
    results = {}
    try:
        host.clear_stall()
        host.write(0x152, 50)
        profiles = build_profiles(args.max_rpm, False)
        tuner = Tuner(host, profiles)
        tuner.repeats = args.repeats
        for name, gains in CANDIDATES.items():
            print("\n== %s : %s ==" % (name, fmt_gains(gains)))
            res = tuner.evaluate(gains)
            results[name] = res
            print("  平均 %.3f° / 最差 %.3f° / 综合 %.3f°"
                  % (res["mean_deg"], res["worst_deg"], res["combined"]))
    finally:
        host.write(0x11F, 0)
        host.disable()
        host.close()

    best_name = min(results, key=lambda k: results[k]["combined"])
    best = CANDIDATES[best_name]
    print("\n胜出：%s（综合 %.3f°）%s"
          % (best_name, results[best_name]["combined"], fmt_gains(best)))

    host = TTHost(args.port)
    try:
        host.set_pid(best["pos_kp"], best["pos_ki"], best["pos_kd"],
                     best["vel_kp"], best["vel_ki"], best["vel_kd"])
        time.sleep(0.2)
        print("回读 PID =", host.get_pid())
        print("写入 Flash：%s" % ("成功" if host.save_params() else "失败"))
    finally:
        host.write(0x11F, 0)
        host.disable()
        host.close()

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "repeats": args.repeats,
                   "winner": best_name,
                   "winner_gains": best,
                   "results": results}, f, ensure_ascii=False, indent=2)
    print("报告：%s" % os.path.abspath(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
