"""定论用：账本(0x134 MOTION_SCHEDULED_STEPS) 峰值 vs 编码器实走。

  python scripts/tt_ledger_tmp.py          半圈 0x126 × 5 趟（板上当前参数）

运动过程中每拍抓 0x134，取本次运动见过的最大值 = 固件认为自己该发/已发的步数。
  账本恒 = 3200 而编码器 3196~3206  → 脉冲没错，散布出在编码器读数/转子侧
  账本本身就在 3194~3206 跳        → 散布出在规划/账本侧，回引擎里查
跑完 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPR_HALF = 3200
R_ANG, R_MOTION, R_STATE, R_LEDGER = 0x116, 0x11F, 0x162, 0x134


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
print(f'tuning={h.read(0x187)} start={h.read(0x11A)} max={h.read(0x11B)} acc={h.read(0x11C)}')
leds, encs = [], []
try:
    for i in range(5):
        while h.read(R_MOTION):
            time.sleep(0.05)
        time.sleep(0.3)
        prev = h.read(R_ANG)
        acc = 0.0
        seen = False
        led = 0
        t0 = time.time()
        h.write(0x126, 1)
        while time.time() - t0 < 4:
            a = h.read(R_ANG)
            if a is not None:
                acc += unwrap(a - prev)
                prev = a
            v = h.read(R_LEDGER)   # 0x134 不在遥测里推，必须显式读
            if isinstance(v, int) and v > led:
                led = v
            r = h.latest.get(R_MOTION)
            if r == 1:
                seen = True
            elif seen and time.time() - t0 > 0.1:
                break
        if not seen:
            raise RuntimeError('未进入运动')
        time.sleep(0.35)
        e = acc / 56.25
        leds.append(led)
        encs.append(e)
        print(f'第{i + 1}趟  账本峰值 {led:6d}  编码器 {e:9.2f}  差 {e - led:+6.2f} 微步'
              f' ({(e - led) * 0.05625:+.4f}°)')
    print(f'>>> 账本取值集合 {sorted(set(leds))}   编码器均值 {sum(encs) / len(encs):.2f} '
          f'散布 {max(encs) - min(encs):.2f} 微步')
finally:
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'已失能 0x162={h.read(R_STATE)}')
