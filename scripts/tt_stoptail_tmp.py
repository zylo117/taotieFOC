"""「缓停抖一下」定位：把减速到停之后 1.5s 内的每一拍都打出来。

只回答三个问题：
  1) 抖的那一下发生在什么时刻 —— 相对"最后一个脉冲"和相对"0x11F 归 0"分别是多久；
  2) 那一下是谁驱动的 —— 0x165(补偿输出) 还是 0x183(补尾爬行计数) 在动；
  3) 那一下多大 —— 单个采样间隔内的角度跳变（编码器噪声约 ±0.02°，1 微步 = 0.056°）。

用法：
  python scripts/tt_stoptail_tmp.py                # 250RPM / 6400 步 / 600RPM/s 自然走完
  python scripts/tt_stoptail_tmp.py 250 64000 600  指定参数
  python scripts/tt_stoptail_tmp.py 250 64000 600 soft   跑到一半按 0x163 缓停
结束还原 0x107 / 0x167 / 0x16D。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import (TTHost, R_FOLLOW_ERR, R_MOTION, R_SPEED_RPM,
                     R_ENCODER_ANGLE_MDEG, R_MICROSTEPS)

R_COMP = 0x165          # compensated_rps × 1000
R_CREEP_EVENTS = 0x183  # 补尾爬行进入次数
R_CREEP_DEFICIT = 0x184  # 最后一次爬行时的剩余步数
R_ENGINE_REINIT = 0x185  # 本段运动完整重装 TMR2 的次数（修好后应为 1）
R_ENGINE_FLOOR = 0x186   # 减速末端补偿把脉冲率压到地板的拍数
R_STALL = 0x15D
R_CLOSED = 0x107
R_DEADBAND = 0x167
R_SOURCE = 0x16D
R_ACCEL = 0x11C
R_MAX_RPM = 0x11B
R_START_RPM = 0x11A
R_PULSES = 0x11D
R_MODE = 0x11E
R_SOFT_STOP = 0x163

RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 250
PULSES = int(sys.argv[2]) if len(sys.argv) > 2 else 6400
ACC = int(sys.argv[3]) if len(sys.argv) > 3 else 600
SOFT = len(sys.argv) > 4 and sys.argv[4] == 'soft'


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


h = TTHost('COM9')
spr = 200 * (h.read(R_MICROSTEPS) or 32)
saved = {r: h.read(r) for r in (R_CLOSED, R_DEADBAND, R_SOURCE)}
creep0 = h.read(R_CREEP_EVENTS) or 0
print(f'细分 spr={spr}  1步={360.0/spr:.5f}° | 起始 '
      f'0x107={saved[R_CLOSED]} 0x167={saved[R_DEADBAND]} 0x16D={saved[R_SOURCE]} '
      f'creep={creep0}')
print(f'工况：{RPM}RPM / {PULSES}步({PULSES/spr:.2f}圈) / {ACC}RPM/s / '
      f'{"缓停 0x163" if SOFT else "自然走完"}')

h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(R_SOURCE, 0)
time.sleep(0.2)
base_angle = h.read(R_ENCODER_ANGLE_MDEG)


def unwrap_delta(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


try:
    h.write(R_START_RPM, min(120, RPM))
    h.write(R_MAX_RPM, RPM)
    h.write(R_ACCEL, ACC)
    h.write(R_PULSES, PULSES)
    h.write(R_MODE, 0)          # POS_FWD
    t_launch = time.time()
    h.write(R_MOTION, 1)

    # 预计走完的时刻：加速 + 巡航 + 减速（不够长就按三角算）
    ramp_s = RPM / max(ACC, 1)
    ramp_steps = spr * (RPM / 60.0) * ramp_s / 2.0
    total_s = (2 * ramp_s) if PULSES <= 2 * ramp_steps else (2 * ramp_s +
               (PULSES - 2 * ramp_steps) / (spr * RPM / 60.0))
    if SOFT:
        t_end = t_launch + min(0.8, total_s * 0.5)
    else:
        t_end = t_launch + total_s

    trace = []
    t_stop_flag = None
    t_pulses_zero = None
    while time.time() < t_end + 1.5:
        if SOFT and t_stop_flag is None and time.time() >= t_end:
            h.write(R_SOFT_STOP, 1, need_ack=False)
        a = h.read(R_ENCODER_ANGLE_MDEG)
        c = h.read(R_COMP)
        now = time.time()
        fe = s32(h.latest.get(R_FOLLOW_ERR))
        rpm = h.latest.get(R_SPEED_RPM)
        run = h.latest.get(R_MOTION)
        trace.append((now, a, s32(c), fe, rpm, run))
        if run == 0 and t_stop_flag is None:
            t_stop_flag = now
        if t_pulses_zero is None and rpm is not None and now > t_end - 1.0 and abs(rpm) < 3:
            t_pulses_zero = now

    # ---------------- 分析 ----------------
    diffs = []
    for i in range(1, len(trace)):
        if trace[i][1] is None or trace[i - 1][1] is None:
            diffs.append((trace[i][0], 0))
            continue
        diffs.append((trace[i][0], unwrap_delta(trace[i][1] - trace[i - 1][1])))

    # 基准时刻取固件自己说"运动结束"的那一帧（0x11F 归 0，遥测约 34Hz）。
    # 抖动就发生在这附近，用它当零点对齐比用"最后一次大幅跳变"准得多。
    done = [t for t, a, c, fe, rpm, run in trace if run == 0 and t > t_launch + 0.3]
    t_done = done[0] if done else None
    moving = [t for t, d in diffs if abs(d) > 200]
    t_big = moving[-1] if moving else trace[-1][0]
    t_stop = t_done if t_done else t_big
    print(f'\n0x11F 归零时刻 {"抓到" if t_done else "没抓到"}（相对最后一次 >0.2° 跳变 '
          f'{((t_stop - t_big) * 1000):+.0f}ms）')

    rate = len(diffs) / max(1e-6, trace[-1][0] - trace[0][0])
    # 累计解绕出转子的真实总行程（采样 269Hz，250RPM 下每帧约 1.5°，解绕安全）
    total = 0
    prev_a = None
    for t, a, c, fe, rpm, run in trace:
        if a is None:
            continue
        if prev_a is not None:
            total += unwrap_delta(a - prev_a)
        prev_a = a
    plan_mdeg = 360000.0 * PULSES / spr
    print(f'采样 {len(trace)} 帧 ≈ {rate:.0f}Hz')
    print(f'行程对账：计划 {plan_mdeg/1000.0:.3f}°   编码器累计(解绕) {total/1000.0:.3f}°   '
          f'差 {(total-plan_mdeg)/1000.0:+.3f}°（{(total-plan_mdeg)*spr/360000.0:+.0f} 微步，'
          f'{100.0*(total-plan_mdeg)/plan_mdeg:+.2f}%）')

    # 「抖一下」的定量口径：以 t_stop 为界，两边各自的累计游走。
    # 单帧阈值抓不到 300pps 的补尾爬行（每帧只有 0.006°），必须用累计和。
    def wander(lo, hi):
        cum = peak = 0
        t_peak = lo
        for t, d in diffs:
            if lo <= t <= hi:
                cum += d
                if abs(cum) > abs(peak):
                    peak = cum
                    t_peak = t
        return cum, peak, t_peak

    tail_cum, tail_peak, _ = wander(t_stop - 0.30, t_stop)
    post_cum, post_peak, t_post = wander(t_stop, trace[-1][0])
    us = 360000.0 / spr
    print(f'停前 300ms 累计走 {tail_cum/1000.0:+.3f}°（{tail_cum/us:+.1f} 微步）'
          f'  ← 补尾爬行会把这段拉出一截')
    print(f'停后累计游走 {post_cum/1000.0:+.3f}°，峰值 {post_peak/1000.0:+.3f}°'
          f'（{post_peak/us:+.1f} 微步，停后 {(t_post-t_stop)*1000:.0f}ms）'
          + ('  ← 明显超噪声，就是"抖一下"' if abs(post_peak) > 3 * us else '  ← 在噪声内，没抖'))

    print('\n以 0x11F 归零为 0ms，前后各 16 帧（Δ = 这一帧相对上一帧的角度变化）：')
    idx = max(1, min(range(len(trace)), key=lambda i: abs(trace[i][0] - t_stop)))
    for i in range(max(1, idx - 16), min(len(trace), idx + 17)):
        t, a, c, fe, rpm, run = trace[i]
        d = diffs[i - 1][1] if i - 1 < len(diffs) else 0
        if a is None:
            continue
        print(f'{(t - t_stop)*1000:+8.0f}ms | Δ{d/1000.0:+7.3f}° | {a:7d} mdeg | '
              f'FE {(fe/1000.0 if fe is not None else float("nan")):+7.3f}° | '
              f'0x165 {(c/1000.0 if c is not None else float("nan")):+7.3f} | '
              f'{rpm if rpm is not None else -999:+5}rpm | run={run}')

    creep1 = h.read(R_CREEP_EVENTS) or 0
    reinit = h.read(R_ENGINE_REINIT)
    floor = h.read(R_ENGINE_FLOOR)
    print(f'\n---- 判读 ----\n补尾爬行 0x183 {creep0} → {creep1}'
          f'（{"触发" if creep1 > creep0 else "没触发"}）0x184 剩余 {h.read(R_CREEP_DEFICIT)} '
          f'| 0x15D 堵转 {h.read(R_STALL)}')
    print(f'连续引擎 0x185 重装定时器 {reinit} 次（应为 1 = 只有起步那次；>1 说明换档还在重装）'
          f' | 0x186 补偿触底 {floor} 拍（非 0 = 补偿大到能掐断脉冲流）')
    comp_run = [c for t, a, c, fe, rpm, run in trace if t < t_stop and c is not None]
    comp_tail = [c for t, a, c, fe, rpm, run in trace if t > t_stop and c is not None]
    if comp_run:
        print(f'运动段 0x165：均值 {sum(comp_run)/len(comp_run)/1000.0:+.4f} '
              f'最大 {max(comp_run, key=abs)/1000.0:+.4f} rps')
    if comp_tail:
        print(f'停机后 0x165：最大 {max(comp_tail, key=abs)/1000.0:+.4f} rps '
              + ('← 补偿还在动，跳变是位置环驱动的'
                 if max(abs(x) for x in comp_tail) > 20 else '← 补偿为 0，跳变不是补偿驱动的'))
    comp_run = [c for t, a, c, fe, rpm, run in trace if t < t_stop and c is not None]
    comp_tail = [c for t, a, c, fe, rpm, run in trace if t > t_stop and c is not None]
    if comp_run:
        print(f'运动段 0x165：均值 {sum(comp_run)/len(comp_run)/1000.0:+.4f} '
              f'最大 {max(comp_run, key=abs)/1000.0:+.4f} rps')
    if comp_tail:
        print(f'停机后 0x165：最大 {max(comp_tail, key=abs)/1000.0:+.4f} rps '
              + ('← 补偿还在动，跳变是位置环驱动的'
                 if max(abs(x) for x in comp_tail) > 20 else '← 补偿为 0，跳变不是补偿驱动的'))
finally:
    h.write(R_MOTION, 0)
    for r, v in saved.items():
        if v is not None:
            h.write(r, v)
    print(f'\n还原：0x107={h.read(R_CLOSED)} 0x167={h.read(R_DEADBAND)} 0x16D={h.read(R_SOURCE)}')
    h.close()
