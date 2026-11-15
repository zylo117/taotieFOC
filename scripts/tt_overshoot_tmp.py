"""开环位置模式过冲实测：250RPM × 128000 步，用编码器累计行程对账。
同时回答：固件 microsteps 与板上实际细分是否一致（命令圈数 vs 实测圈数）。
用法：py -X utf8 scripts/tt_overshoot_tmp.py [微步数] [转速] [加速度]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

PULSES = int(sys.argv[1]) if len(sys.argv) > 1 else 128000
RPM = int(sys.argv[2]) if len(sys.argv) > 2 else 250
ACCEL = int(sys.argv[3]) if len(sys.argv) > 3 else 1000


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(0.35)
h.write(0x107, 0)              # 开环
spr = h.read(0x130) or 0       # 只试探：读不到就留 0，下面用命令圈数对比即可
print(f'读到的 microsteps/spr 相关：0x130={spr}')
h.write(0x11A, min(120, RPM))
h.write(0x11C, ACCEL)
h.write(0x11B, RPM)
h.write(0x11D, PULSES)
h.write(0x11E, 0)              # 位置正转
s0 = h.read(0x134) or 0
ev0 = h.read(0x183) or 0
df0 = h.read(0x184) or 0
cu0 = h.read(0x178) or 0
# 编码器绝对角：连续采样做环绕展开，累计真实行程
prev = h.read(0x116)
trav_mdeg = 0.0
h.write(0x11F, 1)
t0 = time.time()
last_s = s0
same = 0
while time.time() - t0 < 60.0:
    for _ in h._pump(0.02):
        pass
    a = h.latest.get(0x116)
    if a is not None and prev is not None:
        d = a - prev
        if d > 180000:
            d -= 360000
        elif d < -180000:
            d += 360000
        trav_mdeg += d
        prev = a
    if time.time() - t0 > 3.0:
        s = h.read(0x134)
        if s is not None:
            if s == last_s:
                same += 1
            else:
                same = 0
            last_s = s
            if same >= 3 or (s - s0) >= PULSES:
                break
    if (h.read(0x15D) or 0) == 1:
        print('  失步！')
        break
t_run = time.time() - t0
s1 = h.read(0x134) or 0
h.write(0x163, 1)
time.sleep(1.2)
h.write(0x11F, 0)
ev1 = h.read(0x183) or 0
df1 = h.read(0x184) or 0
cu1 = h.read(0x178) or 0
h.write(0x125, 1)
h.write(0x16D, 1)

moved = s1 - s0
print(f'\n命令：{PULSES} 脉冲，{RPM} RPM，加速 {ACCEL}，耗时 {t_run:.2f}s')
print(f'账本 0x134：{s0} -> {s1}，发出 {moved}（应等于 {PULSES}）')
print(f'编码器累计行程：{trav_mdeg / 1000.0:.3f}°')
for guess in (6400, 12800):
    cmd_rev = PULSES / guess
    cmd_deg = cmd_rev * 360.0
    act_deg = trav_mdeg / 1000.0
    print(f'  若 spr={guess}: 命令 {cmd_deg:.3f}°（{cmd_rev:.3f} 圈） '
          f'实测 {act_deg:.3f}°  差 {act_deg - cmd_deg:+.3f}° '
          f'({(act_deg - cmd_deg) / cmd_deg * 100.0:+.3f}%)')
print(f'蠕行事件 0x183：{ev0} -> {ev1}   差额 0x184：{df0} -> {df1}')
print(f'catchup 事件 0x178：{cu0} -> {cu1}')
h.close()
