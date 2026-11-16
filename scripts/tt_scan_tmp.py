"""在用户的真实工况下扫步数 + A/B 换档方式，定位"每趟固定多走"注在哪。

  python scripts/tt_scan_tmp.py <起步RPM> <最高RPM> <加速度RPM/s> <步数...> [nos]
  nos = 0x187 bit0=1（关闭无缝换档，换档退回完整重装定时器）
开环 0x107=0。每趟报 实走-计划 微步；末尾给平均。跑完还原参数并 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

STEP_UM = 0.05625
R_CLOSED, R_STATE = 0x107, 0x162
R_START, R_MAX, R_ACC = 0x11A, 0x11B, 0x11C
R_PULSES, R_MODE, R_MOTION = 0x11D, 0x11E, 0x11F
R_ANG, R_FE = 0x116, 0x121
R_REINIT, R_TIER, R_TUNING, R_CREEP = 0x185, 0x186, 0x187, 0x183


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


nums = [int(x) for x in sys.argv[1:] if x.isdigit()]
NOS = 'nos' in sys.argv
assert len(nums) >= 4, '用法: 起步RPM 最高RPM 加速度 步数...'
START, MAXRPM, ACC = nums[0], nums[1], nums[2]
sizes = nums[3:]

h = TTHost('COM9')
if h.read(0x16D) != 0:
    print('0x16D 外部 → 强制内部')
    h.write(0x16D, 0)
    time.sleep(0.2)
saved = {r: h.read(r) for r in (R_CLOSED, R_START, R_MAX, R_ACC, R_TUNING)}
h.write(0x124, 1)
time.sleep(0.8)
h.write(0x107, 0)
h.write(0x11A, START)
h.write(0x11B, MAXRPM)
h.write(0x11C, ACC)
h.write(0x187, 1 if NOS else 0)
print(f'开环 start={START} max={MAXRPM} acc={ACC} 0x187={1 if NOS else 0}'
      f'{"（无缝换档关闭）" if NOS else ""}  0x162={h.read(R_STATE)}')
try:
    allb = []
    for steps in sizes:
        biases = []
        for rep in range(3):
            while h.read(R_MOTION):
                time.sleep(0.05)
            time.sleep(0.3)
            h.write(R_PULSES, steps)
            h.write(R_MODE, 0)
            if h.read(R_PULSES) != steps:
                raise RuntimeError(f'0x11D 丢写 {h.read(R_PULSES)} ≠ {steps}')
            prev = h.read(R_ANG)
            acc = 0
            seen = False
            t_stop = None
            t0 = time.time()
            h.write(R_MOTION, 1)
            while time.time() - t0 < 20:
                a = h.read(R_ANG)
                if a is not None:
                    acc += unwrap(a - prev)
                    prev = a
                r = h.latest.get(R_MOTION)
                if r == 1:
                    seen = True
                elif seen and t_stop is None:
                    t_stop = time.time() - t0
                    break
            if not seen:
                raise RuntimeError('未进入运动')
            time.sleep(0.35)
            bias = acc / 56.25 - steps
            biases.append(bias)
            print(f'{steps:6d} 步 第{rep + 1}趟 耗时 {(t_stop or -1):6.3f}s  '
                  f'实走 {acc / 56.25:9.1f} 偏 {bias:+6.2f} 微步({bias * STEP_UM:+.4f}°)  '
                  f'FE={(s32(h.read(R_FE)) or 0) / 1000.0:+.3f}°  '
                  f'0x185={h.read(R_REINIT)} 0x186={h.read(R_TIER)} 0x183={h.read(R_CREEP)}')
        m = sum(biases) / len(biases)
        allb.append(m)
        print(f'{steps:6d} 步 平均 {m:+6.2f} 微步 ({m * STEP_UM:+.4f}°)  '
              f'每千步 {m / steps * 1000:+.2f} 微步')
    print(f'总平均 {sum(allb) / len(allb):+6.2f} 微步；各长度平均 = '
          f'{" / ".join(f"{x:+.2f}" for x in allb)}'
          f'  →{"常数（每趟固定注入）" if max(allb) - min(allb) < 2.0 else "随步数变化"}')
finally:
    for reg, val in saved.items():
        if val is not None:
            h.write(reg, val)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'参数已还原，已失能 0x162={h.read(R_STATE)}')
