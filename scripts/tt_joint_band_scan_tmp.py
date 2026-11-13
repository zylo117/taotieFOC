"""逐点独立起步的开环带扫：重点量「加速穿越时在哪个转速失步」，只统计纯巡航段。

相比上一版的修正：
  * 每点独立起步（先 0x15F 清堵转 + 0x124 使能，再 0x11F=1）——上一版整场只起步一次，
    一点失步后面全是死点；实测证明堵转锁存期间单写 0x11F 起不来。
  * 点间用 0x163 缓停（减速停），不硬停。
  * 加速段也采速，失步时报出「崩在哪个转速」，这是本次最值钱的指标。
  * 巡航统计只用 0x135(命令) vs 0x120(实测) 同窗对比；主动 read 只在 0.5s 节拍出。
用法：py -X utf8 scripts/tt_joint_band_scan_tmp.py
"""
import sys, time, math
sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPEEDS = [250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 900, 1000]
DWELL = 3.0
ACCEL = 300
LOG = 'band_scan_joint.log'


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def stats(xs):
    n = len(xs)
    if n < 3:
        return 0.0, 0.0, 0.0
    m = sum(xs) / n
    return m, math.sqrt(sum((x - m) ** 2 for x in xs) / n), max(xs) - min(xs)


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
log = open(LOG, 'w', encoding='utf-8')


def emit(line):
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x123, 1000)
h.write(0x107, 0)          # 开环（源头条件）
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


def fresh_start(tgt):
    h.write(0x15F, 1)      # 清堵转（锁存状态下单写 0x11F 起不来）
    h.write(0x124, 1)      # 重新使能
    time.sleep(0.35)
    h.write(0x11B, tgt)
    h.write(0x11E, 2)
    h.write(0x11F, 1)


emit('逐点独立起步扫描。序列：' + ' '.join(str(s) for s in SPEEDS))
emit(f'加速 {ACCEL} RPM/s，每点等进入巡航后统计 {DWELL}s，点间缓停。开环（补偿关）。')
emit('请重点听：每次重新起步往上冲的过程中，大概什么转速开始震/崩。')
emit('\n 目标  命令RPM  实测RPM  差%   | FE均值°  FE RMS°  FE P-P° | 失步转速 / 结果')

for tgt in SPEEDS:
    emit('')
    emit(f'>>>  >>>  >>>   现 在 {tgt} RPM   >>>  >>>  >>>')
    fresh_start(tgt)
    # 【不再用"稳定200ms"门禁】上一版那个门禁会把振荡本身当无效数据扔掉——
    # 实测读数摆到目标的 120~126%，正是我要量的东西。改为按固定时间等加速段结束：
    # t = 500ms presync + (tgt-120)/accel 斜坡 + 0.3s 余量，然后无条件统计。
    ramp = 0.5 + max(0.0, (tgt - 120) / float(ACCEL)) + 0.3
    t0 = time.time()
    while time.time() - t0 < ramp:
        for _ in h._pump(0.01):
            pass
    t0 = time.time()
    nxt_chk = t0 + 0.25
    tr_fe, tr_sp = [], []
    last_v = None
    stall_speed = None
    while time.time() - t0 < DWELL:
        for _ in h._pump(0.008):
            pass
        v = s32(h.latest.get(0x120) or 0)
        if v != last_v:
            last_v = v
            tr_sp.append((time.time() - t0, v))
            fe = h.latest.get(0x121)
            if fe is not None:
                tr_fe.append((time.time() - t0, s32(fe) / 1000.0))
        if time.time() >= nxt_chk:
            nxt_chk += 0.25
            if h.read(0x15D) == 1:
                stall_speed = max((v for _, v in tr_sp), default=0)
                break
    with open(f'trace_{tgt}.csv', 'w', encoding='utf-8') as f:
        f.write('t,rpm,fe_deg\n')
        for (t, v), (_, e) in zip(tr_sp, tr_fe):
            f.write(f'{t:.4f},{v},{e:.4f}\n')
    sp = [v for _, v in tr_sp]
    fev = [e for _, e in tr_fe]
    if stall_speed is not None:
        s_mean, s_rms, s_pp = stats(sp)
        emit(f'  {tgt:5d} {"--":>8} {s_mean:8.0f} {"--":>5} | {"--":>8} {s_rms:7.2f} {s_pp:7.2f} '
             f'| 失步@≈{stall_speed} RPM  n={len(sp)}')
        soft_stop()
        continue
    s_mean, s_rms, s_pp = stats(sp)
    f_mean, f_rms, f_pp = stats(fev)
    emit(f'  {tgt:5d} {tgt:7d} {s_mean:8.0f} {(s_mean - tgt) / tgt * 100.0:+5.1f} |'
         f' {f_mean:7.2f} {f_rms:7.2f} {f_pp:7.2f} | 转速RMS {s_rms:6.1f} P-P {s_pp:5.0f}  n={len(sp)}')
    soft_stop()

h.write(0x125, 1)
h.write(0x16D, 1)
emit('\n完成，日志已存 band_scan_joint.log；已失能、运动源交还外部跟随')
log.close()
h.close()
