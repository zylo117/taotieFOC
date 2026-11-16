"""三条回归的板载 A/B 对照：靠 0x187 运行时开关直接比，不用一版版重刷。

  0x187 bit0 = 1  禁用无缝换档（换档退回「完整重装定时器」）
  0x187 bit2 = 1  补尾退回旧锯齿（速度归零那一拍 bump +32pps）

用法：
  python scripts/tt_ab_tmp.py land  <tuning>          开环落点系统偏（多走 / 少走几个微步）
  python scripts/tt_ab_tmp.py angle <tuning>          角度模式阶跃响应（跟手延迟 / 过冲 / 来回抖几下）
  tuning 取 0（默认全开）/ 1 / 4 / 5

land  ：0x107=0 纯开环，同一台子跑 400 / 3200 / 64000 步各 3 趟，报每趟"实际走的角度 −
         计划角度"（微步）。系统性正偏 = 引擎多发了脉冲，看它跟 0x185 重装次数是否成比例，
         就能判定 F2 那笔「每次重装白送一个沿」的账到底该不该记。
angle ：闭环角度模式，写 0x128 = 当前角 ±30°，量"写下去到第一下真动"的延迟、过冲、
         以及末端来回反向几下（用户说的"到达后抖两下才停"）。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import (TTHost, R_MOTION, R_SPEED_RPM,
                     R_ENCODER_ANGLE_MDEG, R_MICROSTEPS)

R_CLOSED = 0x107
R_START_RPM = 0x11A
R_MAX_RPM = 0x11B
R_ACCEL = 0x11C
R_PULSES = 0x11D
R_MODE = 0x11E
R_SOFT_STOP = 0x163
R_DEADBAND = 0x167
R_SOURCE = 0x16D
R_TARGET_ANGLE = 0x128
R_ANGLE_MAX_RPM = 0x12F
R_ANGLE_ACCEL = 0x130
R_ENGINE_REINIT = 0x185
R_ENGINE_TIER = 0x186
R_ENGINE_TUNING = 0x187

CASE = sys.argv[1] if len(sys.argv) > 1 else 'land'
TUNING = int(sys.argv[2]) if len(sys.argv) > 2 else 0
# 余下参数：'cl' = 带闭环补偿跑（默认纯开环）；数字 = 步数列表
_args = sys.argv[3:]
CLOSED = 'cl' in _args
SIZES = [int(x) for x in _args if x.isdigit()] or [3200, 6400, 32000, 64000]


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
spr = 200 * (h.read(R_MICROSTEPS) or 32)
STEP_DEG = 360.0 / spr
saved = {r: h.read(r) for r in (R_CLOSED, R_DEADBAND, R_SOURCE,
                                R_ANGLE_MAX_RPM, R_ANGLE_ACCEL)}
print(f'细分 spr={spr} 1步={STEP_DEG:.5f}° | tuning={TUNING} 用例={CASE} | '
      f'起始 0x107={saved[R_CLOSED]} 0x167={saved[R_DEADBAND]}')
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(R_ENGINE_TUNING, TUNING)
time.sleep(0.2)
print(f'回读 0x187 = {h.read(R_ENGINE_TUNING)}')

try:
    if CASE == 'land':
        h.write(R_CLOSED, 1 if CLOSED else 0)
        h.write(R_SOURCE, 0)
        h.write(R_START_RPM, 120)
        h.write(R_MAX_RPM, 250)
        h.write(R_ACCEL, 600)
        print(f'状态：0x107={"1 带闭环补偿" if CLOSED else "0 纯开环"} '
              f'0x167={h.read(R_DEADBAND)}')
        for steps in SIZES:
            print(f'\n--- {steps} 步（{steps * STEP_DEG:.2f}° = {steps / spr:.2f}圈）x3 ---')
            biases = []
            for rep in range(3):
                while h.latest.get(R_MOTION):
                    time.sleep(0.05)
                time.sleep(0.35)
                a0 = h.read(R_ENCODER_ANGLE_MDEG)
                # 预计走完：够长=梯形，不够长=三角电梯
                ramp_s = 250.0 / 600.0
                ramp_steps = spr * (250.0 / 60.0) * ramp_s / 2.0
                total_s = (2 * ramp_s) if steps <= 2 * ramp_steps else (
                    2 * ramp_s + (steps - 2 * ramp_steps) / (spr * 250.0 / 60.0))
                h.write(R_PULSES, steps)
                h.write(R_MODE, 0)
                # 【写完必回读】0x11D=0 会被固件当成"无限速度模式"，丢写绝不能放行，
                # 否则量到的全是脏数据（之前那次卡死就是这么来的）。
                back = h.read(R_PULSES)
                if back != steps:
                    raise RuntimeError(f'0x11D 回读 {back} ≠ {steps}：写没进去，'
                                       f'多半是网页 UI 在抢 COM9，这趟作废')
                t0 = time.time()
                h.write(R_MOTION, 1)
                travel = 0
                prev = a0
                t_end_guess = t0 + total_s + 3.0
                while time.time() < t_end_guess:
                    a = h.read(R_ENCODER_ANGLE_MDEG)
                    if a is not None:
                        travel += unwrap(a - prev)
                        prev = a
                    if h.latest.get(R_MOTION) == 0:
                        t_end_guess = time.time() + 0.60
                    elif time.time() - t0 > 15.0:
                        print('  !! 超时未停')
                        break
                time.sleep(0.35)
                got = travel / (STEP_DEG * 1000.0)
                bias = got - steps
                biases.append(bias)
                fe = s32(h.read(0x121))
                comp = s32(h.read(0x165))
                print(f'  第{rep + 1}趟 实走 {got:9.1f} 步  偏 {bias:+7.1f} 微步'
                      f'（{bias * STEP_DEG:+.4f}°）  落点FE {(fe or 0) / 1000.0:+7.3f}°  '
                      f'0x165={(comp or 0) / 1000.0:+.3f} '
                      f'0x185={h.read(R_ENGINE_REINIT)} 0x186={h.read(R_ENGINE_TIER)}')
            avg = sum(biases) / len(biases)
            print(f'  平均系统偏 {avg:+7.1f} 微步（{avg * STEP_DEG:+.4f}°）'
                  f'，折算 {1e6 * avg / max(steps, 1):+.1f}ppm')
        print('\n判读：各档 ppm 接近 = 速率/账本错（走得越多偏得越多）；'
              '\n      各档微步数接近同一常数 = 每次运动固定多发若干脉冲（起步/换档沿）；'
              '\n      闭环态偏差明显大于开环态 = 偏差主要来自补偿输出，不是脉冲引擎。')

    else:
        h.write(R_CLOSED, 1)
        h.write(R_DEADBAND, 400)
        h.write(R_ANGLE_MAX_RPM, 120)
        h.write(R_ANGLE_ACCEL, 600)
        for jump in (30.0, -30.0):
            print(f'\n--- 角度模式 阶跃 {jump:+.0f}° x3 ---')
            for rep in range(3):
                while h.latest.get(R_MOTION):
                    time.sleep(0.05)
                time.sleep(0.4)
                a0 = h.read(R_ENCODER_ANGLE_MDEG)
                if a0 is None:
                    print('  读不到角度')
                    break
                target = a0 + jump * 1000.0
                h.write(R_TARGET_ANGLE, int(target) & 0xFFFFFFFF)
                t_cmd = time.time()
                trace = []
                prev = a0
                while time.time() < t_cmd + 2.2:
                    a = h.read(R_ENCODER_ANGLE_MDEG)
                    if a is None:
                        continue
                    trace.append((time.time(), unwrap(a - prev)))
                    prev = a
                h.write(R_SOFT_STOP, 1, need_ack=False)   # 角度模式下 = 干净退出
                time.sleep(0.15)
                t_move = t_settle = None
                cum = 0.0
                peak = 0.0
                signs = []
                for t, d in trace:
                    cum += d
                    if abs(cum) > abs(peak):
                        peak = cum
                    if t_move is None and abs(d) > 200:
                        t_move = t - t_cmd
                    if abs(d) > 200:
                        t_settle = t - t_cmd
                        signs.append(1 if d > 0 else -1)
                revs = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
                over = (abs(peak) - abs(jump * 1000.0)) / (STEP_DEG * 1000.0)
                print(f'  第{rep + 1}次 首动延迟 {1000 * (t_move or -1):6.0f}ms  '
                      f'最后一动 {1000 * (t_settle or -1):6.0f}ms  '
                      f'过冲 {over:+6.1f} 微步  中途反向 {revs} 次  '
                      f'0x185={h.read(R_ENGINE_REINIT)} 0x186={h.read(R_ENGINE_TIER)}')
finally:
    h.write(R_SOFT_STOP, 1, need_ack=False)
    time.sleep(0.1)
    for reg in (R_CLOSED, R_DEADBAND, R_SOURCE, R_ANGLE_MAX_RPM, R_ANGLE_ACCEL):
        if saved[reg] is not None:
            h.write(reg, saved[reg])
    h.write(R_ENGINE_TUNING, 0)
    print(f'还原 0x107={saved[R_CLOSED]} 0x167={saved[R_DEADBAND]} '
          f'0x16D={saved[R_SOURCE]} 0x12F={saved[R_ANGLE_MAX_RPM]} '
          f'0x130={saved[R_ANGLE_ACCEL]} 0x187=0')
