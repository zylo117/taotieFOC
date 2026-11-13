"""实测内部规划运动的起步斜坡：0x120 时间序列，判断有没有加速段。

用法：py -X utf8 scripts/tt_ramp_probe_tmp.py [mode]
  mode = vel  速度模式(0x11E=2, 无限跑)
  mode = pos  位置模式(0x11E=0, 定量脉冲)
  省略 = 两个都测
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPR = 25600          # 64 细分 * 400 整步（雷赛06 0.9°）
START_RPM = 60
MAX_RPM = 600
ACCEL = 300          # RPM/s -> 爬到 600 应当要 (600-60)/300 = 1.8s


def s32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


def sample_trace(h, seconds, label):
    """按 20ms 采样遥测 0x120（带符号整数 RPM），返回 [(t_s, rpm), ...]"""
    trace = []
    t0 = time.time()
    nxt = t0
    while time.time() - t0 < seconds:
        for _ in h._pump(0.02):
            pass
        now = time.time()
        if now >= nxt:
            raw = h.latest.get(0x120)
            trace.append((now - t0, float(s32(raw)) if raw is not None else 0.0))
            nxt += 0.02
    return trace


def judge(trace, target=MAX_RPM, accel=ACCEL):
    """看速度是不是逐步爬升：统计越过 30%/60%/90% 目标速度的时刻。"""
    ramp_t = (target - START_RPM) / accel
    marks = {}
    for frac in (0.3, 0.6, 0.9):
        level = target * frac
        for t, rpm in trace:
            if rpm >= level:
                marks[frac] = t
                break
    peak = max((r for _, r in trace), default=0.0)
    return marks, peak, ramp_t


def run(h, mode, start_rpm=START_RPM, max_rpm=MAX_RPM, accel=ACCEL):
    print(f'\n===== {mode} 模式：start={start_rpm} max={max_rpm} accel={accel} RPM/s =====', flush=True)
    h.write(0x16D, 0)            # 内部规划
    h.enable()
    time.sleep(0.2)
    h.write(0x123, 1000)         # 脉宽 1us
    h.write(0x11A, start_rpm)
    h.write(0x11B, max_rpm)
    h.write(0x11C, accel)
    if mode == 'vel':
        h.write(0x11D, 0)        # 无限跑
        h.write(0x11E, 2)
    else:
        h.write(0x11D, 40 * SPR)  # 40 圈，够跑出梯形
        h.write(0x11E, 0)
    print(f'回读 0x11A={h.read(0x11A)} 0x11B={h.read(0x11B)} 0x11C={h.read(0x11C)} 0x11D={h.read(0x11D)}', flush=True)
    h.write(0x11F, 1)
    trace = sample_trace(h, 4.0, mode)
    h.write(0x11F, 0)
    time.sleep(0.3)

    marks, peak, ramp_t = judge(trace, max_rpm, accel)
    print(f'理论加速时长 = {ramp_t:.2f}s（{start_rpm}->{max_rpm} @ {accel}RPM/s）')
    if marks:
        for frac, t in sorted(marks.items()):
            print(f'  越过 {int(frac*100)}% 目标速({max_rpm*frac:.0f}RPM) 用时 {t:.3f}s')
    else:
        print('  没有任何采样达到 30% 目标速（可能遥测缺失或没转）')
    print(f'  峰值 {peak:.1f} RPM，采样 {len(trace)} 点')
    head = [f'{t:.2f}s:{r:.0f}' for t, r in trace if t <= 1.2]
    print('  起步轨迹: ' + '  '.join(head[:60]))
    return marks, peak


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else 'ab'
    h = TTHost('COM9')
    try:
        if which == 'ab':
            # A/B：同一个 max/accel，只比起始速 0 vs 120。
            # 旧固件语义错误时：start=0 会直接跳到 max（无加速），start=120 有斜坡。
            run(h, 'vel', start_rpm=0)
            time.sleep(0.6)
            run(h, 'vel', start_rpm=120)
        elif which == 'pos0':
            # 位置模式（定量脉冲）起始速 0：既要看斜坡，也要看是否走满行程不失步
            h.write(0x16D, 0)
            h.enable()
            time.sleep(0.2)
            h.write(0x11A, 0)
            h.write(0x11B, MAX_RPM)
            h.write(0x11C, ACCEL)
            h.write(0x11D, 20 * SPR)
            h.write(0x11E, 0)
            a0 = h.read(0x116)
            h.write(0x11F, 1)
            trace = sample_trace(h, 5.0, 'pos')
            h.wait_settled(20.0)
            time.sleep(1.0)
            marks, peak, ramp_t = judge(trace, MAX_RPM, ACCEL)
            print(f'pos 模式 start=0 max={MAX_RPM} accel={ACCEL}：20 圈 = {20*SPR} 脉冲')
            for frac, t in sorted(marks.items()):
                print(f'  越过 {int(frac*100)}% 目标速 用时 {t:.3f}s')
            print(f'  峰值 {peak:.1f} RPM（理论 {MAX_RPM}）')
            head = [f'{t:.2f}s:{r:.0f}' for t, r in trace if t <= 1.2]
            print('  起步轨迹: ' + '  '.join(head[:60]))
            mdeg = h.read(0x121)
            run_s = h.read(0x11F)
            print(f'  收尾：0x11F running={run_s} 跟随误差 0x121={mdeg} mdeg')
        else:
            run(h, which)
    finally:
        h.write(0x11F, 0)
        h.write(0x125, 1)        # 测完失能
        h.write(0x16D, 1)        # 交还外部跟随语义
        h.close()


if __name__ == '__main__':
    main()
