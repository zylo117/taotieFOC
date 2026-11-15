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
COMP = int(sys.argv[4]) if len(sys.argv) > 4 else 0     # 0=开环 1=闭环补偿


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
# 使能瞬间转子会被拽向激励平衡位置（用户说的"抖一下"）。这一段单独积分并等 1.2s，
# 否则使能抖动会被算进运动误差里，测出来的"少走/多走"就不是运动本身的账。
prev = h.read(0x116)
jerk = 0.0
tg = time.time()
while time.time() - tg < 1.2:
    for _ in h._pump(0.02):
        pass
    a = h.latest.get(0x116)
    if a is not None and prev is not None:
        d = a - prev
        if d > 180000:
            d -= 360000
        elif d < -180000:
            d += 360000
        jerk += d
        prev = a
print(f'使能抖动（使能后 1.2s 内编码器累计）：{jerk / 1000.0:+.3f}°')
# 关键基线：20 整圈（128000/6400）跑完静置后，绝对角应回到这里，误差才是真误差。
a_base = h.read(0x116)
h.write(0x107, COMP)
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
h.write(0x11F, 0)
# 静置 2.5s：等最终齿位附近的振铃衰减完，再读"绝对角"。20 整圈的命令位移是
# 360° 的整数倍，所以静置后的绝对角应当等于使能基线；这个差才是真正静态落点误差，
# 不受"我在振铃的哪个相位收尾"影响。
for _ in range(250):
    for __ in h._pump(0.01):
        pass
a_end = h.read(0x116)
if a_end is not None and a_base is not None:
    dstatic = (a_end - a_base + 180000) % 360000 - 180000
    print(f'【静置 2.5s 后】绝对角基线 {a_base} mdeg -> 终点 {a_end} mdeg，'
          f'静态落点误差 = {dstatic / 1000.0:+.3f}°  '
          f'（{dstatic / 1000.0 / 360.0 * 6400:+.1f} 微步 @32细分）')
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
