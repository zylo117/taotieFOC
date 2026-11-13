"""逐点读片上共振抽头：每个转速巡航时，报主频 / 幅值 / 震度 / 频点掩码。

抽头窗口 0.512s，所以每点读 4~5 次（0.6s 节拍），取主频众数 + 震度均值。
判读：主频随转速走 → 空间阶次锁定（抖动/跳频有救）；主频钉住不动 → 时间锁定
（斩波帧频那类），抖动无效。掩码只亮 1 个 bit = 单一频率共振。
用法：py -X utf8 scripts/tt_tap_scan_tmp.py
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPEEDS = [300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 900, 1000]
DWELL = 3.0
ACCEL = 300


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x123, 1000)
h.write(0x107, 0)          # 开环
h.write(0x11A, 120)
h.write(0x11C, ACCEL)
h.write(0x11D, 0xFFFFFFFF)


def soft_stop():
    h.write(0x163, 1)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        for _ in h._pump(0.02):
            pass
        if abs(s32(h.latest.get(0x120) or 0)) < 25:
            break
    h.write(0x11F, 0)
    time.sleep(0.3)


print('  目标RPM  主频Hz(众数)  主峰mdeg  震度RMS_mdeg  掩码   逐次主频')
for tgt in SPEEDS:
    h.write(0x15F, 1)
    h.write(0x124, 1)
    time.sleep(0.35)
    h.write(0x11B, tgt)
    h.write(0x11E, 2)
    h.write(0x11F, 1)
    print(f'>>>  >>>   现 在 {tgt} RPM   >>>  >>>', flush=True)
    ramp = 0.5 + max(0.0, (tgt - 120) / float(ACCEL)) + 0.35
    t0 = time.time()
    while time.time() - t0 < ramp:
        for _ in h._pump(0.02):
            pass
    freqs, amps, rms, masks = [], [], [], []
    t1 = time.time()
    nxt = t1
    dead = False
    while time.time() - t1 < DWELL:
        for _ in h._pump(0.02):
            pass
        if time.time() >= nxt:
            nxt += 0.6
            f = h.read(0x17A)
            a = h.read(0x17B)
            r = h.read(0x17C)
            m = h.read(0x17D)
            if f is not None:
                freqs.append(f)
                amps.append(a or 0)
                rms.append(r or 0)
                masks.append(m or 0)
            if h.read(0x15D) == 1:
                dead = True
                break
    if not freqs:
        print(f'  {tgt:6d}  无数据', flush=True)
        soft_stop()
        continue
    # 众数（并列取较大的那个频率，避免低频占优的假象）
    cnt = {}
    for f in freqs:
        cnt[f] = cnt.get(f, 0) + 1
    best = sorted(cnt.items(), key=lambda kv: (-kv[1], -kv[0]))[0][0]
    or_mask = 0
    for m in masks:
        or_mask |= m
    print(f'  {tgt:6d} {best:10d} {max(amps):10d} '
          f'{sum(rms) // max(1, len(rms)):13d}  0x{or_mask:02X}   {freqs}'
          f'{"  [失步]" if dead else ""}', flush=True)
    soft_stop()

h.write(0x125, 1)
h.write(0x16D, 1)
print('\n完成；已失能、运动源交还外部跟随')
h.close()
