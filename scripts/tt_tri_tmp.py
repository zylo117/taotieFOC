"""短行程减速修正的验收：耗时 + 0x186 换档次数 + 实走偏差，跑完立即失能。

  python scripts/tt_tri_tmp.py            默认 1600/3200/6400/32000 各 2 趟
验收判据（修正后）：3200 步耗时 0.24s → 约 0.33s、0x186 >= 3、多走回到 +2~3 微步本底。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_EN, R_DIS = 0x124, 0x125
R_CLOSED, R_STATE = 0x107, 0x162
R_START, R_MAX, R_ACC = 0x11A, 0x11B, 0x11C
R_PULSES, R_MODE, R_MOTION = 0x11D, 0x11E, 0x11F
R_ANG, R_FE = 0x116, 0x121
R_REINIT, R_TIER = 0x185, 0x186
STEP_UM = 0.05625


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
# 【0x16D 运动来源必须 = 0 内部】= 1(外部) 时 enableAuthorityIsExternalPin() 为真，
# requestDriverEnable(true) 直接 return、startMotion 也被拒：上位机写 0x124/0x11F 全无用，
# 现象就是"板子死住不动"。之前两次白跑几十分钟全卡在这。这里强制切回内部并且不复原，
# 需要外部控制的人请自己从上位机/UI 里切回去。
src = h.read(0x16D)
if src != 0:
    print(f'!! 0x16D 运动来源 = {src}（外部）→ 使能权限在外部 EN 引脚，'
          f'上位机的 0x124/0x11F 会被固件按设计拒绝。本脚本强制改成 0（内部），'
          f'跑完不复原。')
    h.write(0x16D, 0)
    time.sleep(0.2)
START_RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 120
sizes = [int(x) for x in sys.argv[2:]] or [1600, 3200, 6400, 32000]
saved_closed = h.read(R_CLOSED)
h.write(R_EN, 1)
time.sleep(0.8)
print(f'0x162 使能={h.read(R_STATE)}  start={START_RPM}RPM max=250 acc=600 0x107=0（纯开环）')
try:
    h.write(R_CLOSED, 0)
    h.write(R_START, START_RPM)
    h.write(R_MAX, 250)
    h.write(R_ACC, 600)
    for steps in sizes:
        for rep in (1, 2):
            while h.read(R_MOTION):
                time.sleep(0.05)
            time.sleep(0.25)
            h.write(R_PULSES, steps)
            h.write(R_MODE, 0)
            if h.read(R_PULSES) != steps:
                raise RuntimeError(f'0x11D 回读丢写：{h.read(R_PULSES)} ≠ {steps}')
            prev = h.read(R_ANG)
            acc = 0
            seen = False
            t_stop = None
            t0 = time.time()
            h.write(R_MOTION, 1)
            while time.time() - t0 < 12:
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
            time.sleep(0.25)
            if not seen:
                raise RuntimeError('0x11F 从未置起：启动被固件拒绝，别再空跑，查使能链路')
            bias = acc / 56.25 - steps
            fe = (s32(h.read(R_FE)) or 0) / 1000.0
            print(f'{steps:6d} 步 第{rep}趟  耗时 {(t_stop or -1):5.3f}s  '
                  f'实走 {acc / 56.25:9.1f} 微步  偏 {bias:+6.1f} 微步'
                  f'({bias * STEP_UM:+.4f}°)  0x185={h.read(R_REINIT)} '
                  f'0x186={h.read(R_TIER)}  FE={fe:+.3f}°')
finally:
    h.write(R_DIS, 1)
    time.sleep(0.3)
    if saved_closed is not None:
        h.write(R_CLOSED, saved_closed)
    print(f'已失能 0x162={h.read(R_STATE)}  0x107 还原={saved_closed}')
