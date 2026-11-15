"""尾幅 vs 巡航转速：一刀切开「滞后角释放」与「整步跳」。

滞后角 ∝ 转速（本仓库注释记的固有滞后 ~1.5°），所以：
  · 尾巴随 RPM 成比例（30→0.2°、250→1.8°）  => 账本把磁场停在转子前面，收尾松手 = 滞后释放
  · 尾巴恒定 ≈1.8°（= 1 个整步），与 RPM 无关  => 停机瞬间的电平/齿位跳，不是滞后
  · 只在某个 RPM 之上才出现               => 共振/失步，另案

每档都：使能静置 1.2s -> 取基线角 -> 跑 6 圈自然收尾 -> 账本到 N 后再采 1.5s（尾巴）
        -> 停稳 2.5s -> 读落点。0x121 同步采样用于判「软件是否真发了脉冲」。
全程 <=250RPM、减速度 300RPM/s，不碰高速区。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost, R_MOTION, R_SCHEDULED, R_ENCODER_ANGLE_MDEG, R_FOLLOW_ERR

RPM_LIST = [30, 60, 120, 250]
PULSES = 38400            # 6 圈：250RPM 下减速段要 1.04 圈，留足余量
ACCEL = 300


def unwrap(x):
    return x - 360000 if x > 180000 else (x + 360000 if x < -180000 else x)


def latest(h, reg, timeout=0.5):
    end = time.time() + timeout
    while time.time() < end:
        for _ in h._pump(0.01):
            pass
        if h.latest.get(reg) is not None:
            return h.latest[reg]
    return h.latest.get(reg)


h = TTHost('COM9')
print(f'口径核对 0x100={h.read(0x100)} 0x182={h.read(0x182)}（期望 32 / 14062）')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(0x107, 0)                     # 开环

rows = []
try:
    for rpm in RPM_LIST:
        base = latest(h, R_ENCODER_ANGLE_MDEG)
        h.write(0x11A, max(10, min(rpm, 60)))
        h.write(0x11B, rpm)
        h.write(0x11C, ACCEL)
        h.write(0x11D, PULSES)
        h.write(0x11E, 0)
        h.write(0x11F, 1)
        # 等账本到 N
        t_ledger = None
        t0 = time.time()
        while time.time() - t0 < 60:
            if (h.latest.get(R_SCHEDULED) or 0) >= PULSES:
                t_ledger = time.time()
                break
            for _ in h._pump(0.005):
                pass
        if t_ledger is None:
            print(f'{rpm} RPM：没等到账本到 N，跳过')
            h.write(0x163, 1)
            time.sleep(2.0)
            continue
        # 尾巴窗口：账本到 N 之后 1.5s
        tail = 0.0
        fe_first = fe_last = None
        marks = []
        prev = base
        t_end = time.time() + 1.5
        while time.time() < t_end:
            for _ in h._pump(0.005):
                pass
            a = h.latest.get(R_ENCODER_ANGLE_MDEG)
            f = h.latest.get(R_FOLLOW_ERR)
            if a is None or prev is None:
                continue
            d = unwrap(a - prev) / 1000.0
            tail += d
            if abs(d) > 0.05:
                marks.append(f'{(time.time() - t_ledger) * 1000:.0f}ms:{d:+.3f}')
            if f is not None:
                fe_first = fe_first if fe_first is not None else f
                fe_last = f
            prev = a
        time.sleep(2.5)                              # 停稳
        final = h.read(R_ENCODER_ANGLE_MDEG)
        dfe = unwrap(fe_last - fe_first) / 1000.0 if (fe_last and fe_first) else float('nan')
        land = unwrap(final - base) / 1000.0 if final else float('nan')
        rows.append((rpm, tail, dfe, land))
        print(f'{rpm:4d} RPM | 账本到 N 后尾巴 = {tail:+7.3f}° | 同窗 Δ0x121 = {dfe:+7.3f}° '
              f'| 落点-基线 = {land:+7.3f}° | 跳变点 {marks if marks else "无"}')
        time.sleep(0.3)
finally:
    h.write(0x11F, 0)
    h.write(0x107, 1)                  # 恢复闭环开关（别再犯）
    h.write(0x16D, 1)
    h.write(0x125, 1)
    h.close()

print('\n判读：')
if len(rows) >= 2:
    lo, hi = rows[0], rows[-1]
    if abs(hi[1]) > 2 * max(0.15, abs(lo[1])):
        print(f'  尾巴随转速显著变化（{lo[0]}RPM:{lo[1]:.3f}° -> {hi[0]}RPM:{hi[1]:.3f}°）'
              f'=> 滞后角释放：账本到底时磁场仍超前转子，停机才松手。修在规划终点，不是账本。')
    elif abs(abs(hi[1]) - abs(lo[1])) < 0.25:
        print(f'  尾巴几乎不随转速变化（≈{abs(hi[1]):.2f}°≈1 个整步）'
              f'=> 停机瞬间的电平/齿位跳，查 stepper_stop_motion_timer 的引脚归位。')
    else:
        print('  介于两者之间：需要再看 0x121 是否同幅变化。')
for rpm, tail, dfe, land in rows:
    verdict = '转子自己跳（软件没发脉冲）' if abs(abs(dfe) - abs(tail)) < 0.15 else '软件发了脉冲'
    print(f'  {rpm:4d} RPM：Δ角 {tail:+.3f}° vs ΔFE {dfe:+.3f}° -> {verdict}')
