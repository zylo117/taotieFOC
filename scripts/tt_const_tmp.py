"""固定多走 vs 速率性多走：慢速下跑 8 倍距离对照，跑完立即失能。

  python scripts/tt_const_tmp.py [最大RPM] [加速度RPM/s] [步数列表...]
默认 60RPM / 300RPM/s，跑 640 / 1280 / 5120 步各 2 趟。
慢到 60RPM 时转子根本没有动量可言（末速 <1RPM），落点散布应在 ±1 微步内；
此时若"实走 − 计划"是同一个常数 → 每趟固定多发若干脉冲（起步/收口沿）；
若随步数按比例增长 → 速率/账本错。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

STEP_UM = 0.05625


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
MAXRPM = int(sys.argv[1]) if len(sys.argv) > 1 else 60
ACC = int(sys.argv[2]) if len(sys.argv) > 2 else 300
nums = [int(x) for x in sys.argv[3:] if x.isdigit()] or [640, 1280, 5120]
if h.read(0x16D) != 0:
    print('0x16D 外部 → 强制内部')
    h.write(0x16D, 0)
    time.sleep(0.2)
saved = h.read(0x107)
h.write(0x107, 0)
h.write(0x11A, 6)
h.write(0x11B, MAXRPM)
h.write(0x11C, ACC)
h.write(0x124, 1)
time.sleep(0.8)
print(f'纯开环 0x107=0  start=6RPM max={MAXRPM}RPM acc={ACC}RPM/s  0x162={h.read(0x162)}')
try:
    for steps in nums:
        for rep in (1, 2):
            while h.read(0x11F):
                time.sleep(0.05)
            time.sleep(0.3)
            h.write(0x11D, steps)
            h.write(0x11E, 0)
            if h.read(0x11D) != steps:
                raise RuntimeError('0x11D 丢写')
            prev = h.read(0x116)
            acc = 0
            seen = False
            t_stop = None
            t0 = time.time()
            h.write(0x11F, 1)
            while time.time() - t0 < 30:
                a = h.read(0x116)
                if a is not None:
                    acc += unwrap(a - prev)
                    prev = a
                r = h.latest.get(0x11F)
                if r == 1:
                    seen = True
                elif seen and t_stop is None:
                    t_stop = time.time() - t0
                    break
            if not seen:
                raise RuntimeError('未进入运动')
            time.sleep(0.4)
            bias = acc / 56.25 - steps
            fe = (h.read(0x121) or 0) / 1000.0
            print(f'{steps:5d} 步 第{rep}趟 耗时 {t_stop:6.3f}s  '
                  f'实走 {acc / 56.25:9.1f} 微步  偏 {bias:+6.2f} 微步'
                  f'({bias * STEP_UM:+.4f}°)  FE={fe:+.3f}°  '
                  f'0x185={h.read(0x185)} 0x186={h.read(0x186)} 0x183={h.read(0x183)}')
finally:
    h.write(0x125, 1)
    time.sleep(0.3)
    if saved is not None:
        h.write(0x107, saved)
    print(f'已失能 0x162={h.read(0x162)}  0x107 还原={saved}')
