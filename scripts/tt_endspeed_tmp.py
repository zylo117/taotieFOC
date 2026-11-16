"""末速探针：把一段运动最后 150ms 的每帧角度增量打出来，直接看有没有减到 0。

  python scripts/tt_endspeed_tmp.py [步数] [趟数]
判读：真减到 0 时，最后几帧的 Δ 应该逐帧缩小到 <0.05°（≈1 微步/帧）；
      若最后仍是每帧零点几度然后直接归零 = 带速硬停。跑完立即失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_ANG, R_MOTION, R_FE = 0x116, 0x11F, 0x121
R_TIER, R_REINIT = 0x186, 0x185

STEPS = int(sys.argv[1]) if len(sys.argv) > 1 else 3200
REPS = int(sys.argv[2]) if len(sys.argv) > 2 else 3


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
if h.read(0x16D) != 0:
    print('0x16D 运动来源 = 外部 → 强制切内部（否则 0x124/0x11F 会被拒绝）')
    h.write(0x16D, 0)
    time.sleep(0.2)
h.write(0x107, 0)
h.write(0x11A, 120)
h.write(0x11B, 250)
h.write(0x11C, 600)
h.write(0x124, 1)
time.sleep(0.8)
print(f'0x162 使能={h.read(0x162)}  {STEPS} 步 × {REPS} 趟，纯开环')
try:
    for rep in range(REPS):
        while h.read(R_MOTION):
            time.sleep(0.05)
        time.sleep(0.3)
        h.write(0x11D, STEPS)
        h.write(0x11E, 0)
        if h.read(0x11D) != STEPS:
            raise RuntimeError('0x11D 丢写')
        prev = h.read(R_ANG)
        rec = []
        seen = False
        t_stop = None
        t0 = time.time()
        h.write(R_MOTION, 1)
        while time.time() - t0 < 8:
            a = h.read(R_ANG)
            if a is not None:
                rec.append((time.time() - t0, unwrap(a - prev) / 1000.0))
                prev = a
            r = h.latest.get(R_MOTION)
            if r == 1:
                seen = True
            elif seen and t_stop is None:
                t_stop = time.time() - t0
                break
        if not seen:
            raise RuntimeError('未进入运动')
        print(f'\n第 {rep + 1} 趟  耗时 {t_stop:.3f}s  0x185={h.read(R_REINIT)} '
              f'0x186={h.read(R_TIER)}  FE后={(h.read(R_FE) or 0) / 1000.0:+.3f}°')
        tail = [x for x in rec if x[0] >= t_stop - 0.15]
        line = []
        for t, d in tail[-26:]:
            line.append(f'{(t - t_stop) * 1000:+.0f}:{d:+.3f}')
        print('   末 150ms 每帧Δ(°):', ' '.join(line))
finally:
    h.write(0x125, 1)
    time.sleep(0.3)
    print(f'\n已失能 0x162={h.read(0x162)}')
