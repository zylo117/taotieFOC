"""两件事一次做掉：
A) 探多高的加速度能不丢步地穿过 400~800 带（这本身就是可用缓解手段）。
B) 用能穿过去的加速度冲到带顶之上，再逐级"下滑"过带，从稳定侧量带内抽头。
用法：py -X utf8 scripts/tt_cross_probe_tmp.py
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

ACCELS = [600, 1200, 2000, 3000]
TOP = 1000
DESC = [900, 850, 800, 750, 700, 650, 600, 550, 500, 450, 400]


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x123, 1000)
h.write(0x107, 0)
h.write(0x11A, 120)
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


def fresh(accel, tgt):
    h.write(0x15F, 1)
    h.write(0x124, 1)
    time.sleep(0.35)
    h.write(0x11C, accel)
    h.write(0x11B, tgt)
    h.write(0x11E, 2)
    h.write(0x11F, 1)


print('A) 上行穿越能力探测（不扶电机）', flush=True)
good = []
for a in ACCELS:
    fresh(a, TOP)
    t0 = time.time()
    res = '超时'
    while time.time() - t0 < 8.0:
        for _ in h._pump(0.01):
            pass
        v = s32(h.latest.get(0x120) or 0)
        if v >= TOP * 0.95:
            res = f'穿过去了（实测 {v}）'
            good.append(a)
            break
        if h.read(0x15D) == 1:
            res = f'失步（崩前峰值约 {v}）'
            break
    print(f'  accel={a:5d} RPM/s : {res}', flush=True)
    soft_stop()

if not good:
    print('\n四种加速度都没能空手穿带，B) 需要你先加阻尼或抬 VREF。')
    h.write(0x125, 1)
    h.write(0x16D, 1)
    h.close()
    sys.exit(0)

a_use = good[0]
print(f'\nB) 用 accel={a_use} 冲到 {TOP} 后逐级下滑过带（每级读一组抽头）', flush=True)
print('  目标  实测RPM | 主频Hz  主峰mdeg  震度mdeg  掩码', flush=True)
fresh(a_use, TOP)
t0 = time.time()
while time.time() - t0 < 8.0:
    for _ in h._pump(0.01):
        pass
    if s32(h.latest.get(0x120) or 0) >= TOP * 0.95:
        break
time.sleep(1.5)
for tgt in DESC:
    h.write(0x11B, tgt)
    time.sleep(0.85)
    f = h.read(0x17A)
    am = h.read(0x17B)
    r = h.read(0x17C)
    m = h.read(0x17D)
    st = h.read(0x15D)
    v = s32(h.latest.get(0x120) or 0)
    print(f'  {tgt:5d} {v:7d} | {(f or 0):6d} {(am or 0):8d} {(r or 0):8d}   0x{(m or 0):02X}'
          f'{"   [失步]" if st == 1 else ""}', flush=True)
    if st == 1:
        break
soft_stop()
h.write(0x125, 1)
h.write(0x16D, 1)
print('已失能、运动源交还外部跟随', flush=True)
h.close()
