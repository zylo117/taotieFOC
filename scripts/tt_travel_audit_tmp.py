"""对账：命令行程 vs 编码器实际行程，分清 FE 发散是假账还是真丢步。

0x116 = 绝对角(mdeg, 0..360000)，自己解环绕累加，不经固件的 FE 通路。
用法：py -X utf8 scripts/tt_travel_audit_tmp.py [max_rpm] [accel] [revs] [comp 0|1]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPR = 25600          # 400 整步 * 64 细分
REV_MDEG = 360000


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


MAX_RPM = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
ACCEL = int(sys.argv[2]) if len(sys.argv) > 2 else 600
REVS = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
COMP = (len(sys.argv) > 4 and sys.argv[4] == '1')

STEPS = int(REVS * SPR)
h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(0.4)
h.write(0x123, 1000)
h.write(0x107, 1 if COMP else 0)   # TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE = 闭环补偿
h.write(0x11A, 120)
h.write(0x11B, MAX_RPM)
h.write(0x11C, ACCEL)
h.write(0x11D, STEPS)
h.write(0x11E, 0)
print(f'命令：{REVS} 圈 = {STEPS} 微步，120->{MAX_RPM} a{ACCEL}，补偿={"ON" if COMP else "OFF"}', flush=True)

h.write(0x11F, 1)
t0 = time.time()
prev = None
travel_mdeg = 0.0
nxt = t0
STALL_T = None
while time.time() - t0 < 30:
    for _ in h._pump(0.01):
        pass
    a = h.read(0x116)
    if a is not None:
        cur = float(a)
        if prev is not None:
            d = cur - prev
            if d > REV_MDEG / 2:
                d -= REV_MDEG
            elif d < -REV_MDEG / 2:
                d += REV_MDEG
            travel_mdeg += d
        prev = cur
    now = time.time()
    if now >= nxt:
        nxt += 0.5
        print(f'  t={now-t0:5.2f}s 0x120={s32(h.latest.get(0x120) or 0):6d} '
              f'0x121={s32(h.read(0x121) or 0)/1000.0:8.2f}° '
              f'实测累计={travel_mdeg/1000.0:9.2f}° ({travel_mdeg/REV_MDEG:5.2f}圈) '
              f'堵转={h.read(0x15D)}', flush=True)
    if h.read(0x15D) == 1:
        STALL_T = now - t0
        print(f'  >>> STALL @ {STALL_T:.2f}s', flush=True)
        break
    if h.latest.get(0x11F) == 0 and now - t0 > 1.0:
        print(f'  >>> 结束 @ {now-t0:.2f}s', flush=True)
        break
time.sleep(1.0)
print(f'\n对账：命令 {REVS:.2f} 圈 / 编码器实测 {travel_mdeg/REV_MDEG:.3f} 圈 '
      f'/ 差 {(travel_mdeg/REV_MDEG) - REVS:+.3f} 圈'
      f'（{((travel_mdeg/REV_MDEG)-REVS)*360:+.1f}°）')
print(f'      收尾 FE 0x121={s32(h.read(0x121) or 0)/1000.0:.2f}°  '
      f'故障字=0x{h.read(0x108) or 0:X}  堵转={h.read(0x15D)}  {"STALL" if STALL_T else "无堵转"}')
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
