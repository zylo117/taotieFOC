"""停止之后位置还在不在动？——分辨"多发脉冲"与"停止后继续沉降/滞后衰减"。

  python scripts/tt_settle_tmp.py        半圈 0x126 × 3 趟

每趟：从运动前开始积分编码器，分别在【0x11F 归零瞬间】、+0.3s、+1.0s、+2.0s 记录
累计位移与 0x116 绝对角，并打印三段"停后还在走的量"。
若停后仍走 ~几微步 → 偏差来自沉降/滞后衰减，不是发脉冲；若几乎不走 → 偏差在脉冲侧。
跑完 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPR_HALF = 3200
R_ANG, R_MOTION, R_STATE = 0x116, 0x11F, 0x162
R_LAG, R_LAGFF = 0x151, 0x152


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
if h.read(0x16D) != 0:
    h.write(0x16D, 0)
    time.sleep(0.2)
h.write(0x124, 1)
time.sleep(1.0)
h.write(0x107, 0)
print(f'0x166 lag_accel={h.read(0x166)}  0x167 deadband={h.read(0x167)}  '
      f'0x151 lag_mdeg={h.read(R_LAG)}  0x152 ff_ratio={h.read(R_LAGFF)}')
try:
    for i in range(3):
        while h.read(R_MOTION):
            time.sleep(0.05)
        time.sleep(0.3)
        prev = h.read(R_ANG)
        acc = 0.0
        seen = False
        marks = {}
        t0 = time.time()
        t_stop = None
        h.write(0x126, 1)
        while time.time() - t0 < 6:
            a = h.read(R_ANG)
            if a is not None:
                acc += unwrap(a - prev)
                prev = a
            now = time.time() - t0
            r = h.latest.get(R_MOTION)
            if r == 1:
                seen = True
            elif seen and t_stop is None:
                t_stop = now
                marks['停'] = acc
            for tag, dt in (('+0.3s', 0.3), ('+1.0s', 1.0), ('+2.0s', 2.0)):
                if t_stop is not None and now >= t_stop + dt and tag not in marks:
                    marks[tag] = acc
            if t_stop is not None and now >= t_stop + 2.1:
                break
        if not seen:
            raise RuntimeError('未进入运动')
        b0 = marks.get('停', acc) / 56.25 - SPR_HALF
        print(f'第{i + 1}趟 停点 {marks.get("停", 0) / 56.25:9.2f} 微步 偏 {b0:+6.2f} | '
              f'停后再走：+0.3s {(marks.get("+0.3s", marks["停"]) - marks["停"]) / 56.25:+6.2f}  '
              f'+1.0s {(marks.get("+1.0s", marks["停"]) - marks["停"]) / 56.25:+6.2f}  '
              f'+2.0s {(marks.get("+2.0s", marks["停"]) - marks["停"]) / 56.25:+6.2f} 微步 | '
              f'lag={h.read(R_LAG)}')
finally:
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'已失能 0x162={h.read(R_STATE)}')
