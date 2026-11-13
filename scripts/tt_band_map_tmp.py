"""共振/震动带测绘：逐点驻留，量 FE 与转速的波动（RMS / 峰峰 / 粗估主频）。

开环（0x107=0）跑，因为要的是"源头"行为；堵转就清掉重使能并记下该点撑不住。
用法：py -X utf8 scripts/tt_band_map_tmp.py [accel]
"""
import sys, time, math
sys.path.insert(0, 'scripts')
from tt_host import TTHost

SPEEDS = [150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 900, 1000]
DWELL = 3.0
ACCEL = int(sys.argv[1]) if len(sys.argv) > 1 else 400


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def stats(xs):
    n = len(xs)
    if n < 3:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    rms = math.sqrt(var)
    return m, rms, max(xs) - min(xs), max(xs), min(xs)


def dom_freq(xs, fs):
    """粗估主频：去均值后数上穿零次数 / 时长（只到奈奎斯特 fs/2）。"""
    n = len(xs)
    if n < 12:
        return 0.0
    m = sum(xs) / n
    y = [x - m for x in xs]
    cross = 0
    for i in range(1, n):
        if y[i - 1] <= 0.0 < y[i]:
            cross += 1
    return cross * fs / max(1.0, n)


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
h.write(0x107, 0)           # 闭环补偿 OFF = 开环
h.write(0x11C, ACCEL)
h.write(0x11D, 0xFFFFFFFF)  # 速度模式无限跑
print(f'开环驻留测绘：accel={ACCEL} RPM/s，每点驻留 {DWELL}s', flush=True)
print('\n 目标RPM  实测均RPM  转速RMS  转速P-P |  FE均值°  FE RMS°  FE P-P° | 主频Hz  结果', flush=True)

for tgt in SPEEDS:
    h.write(0x15F, 1)
    h.write(0x124, 1)
    time.sleep(0.25)
    h.write(0x11A, 120)
    h.write(0x11B, tgt)
    h.write(0x11E, 2)
    h.write(0x11F, 1)
    time.sleep(0.9)          # 先让它爬到附近再开始统计
    # 【采样口径】0x120 是 20ms 平滑的显示值，看不见 12Hz 以上振荡，不能用来定频；
    # 这里改为直接吃固件遥测帧里的 0x121（20kHz 计算的位置误差，约 100Hz 上报），
    # 主动 read 会把总线塞满（tt_host 注释里吃过亏），所以只在 latest 变化时收。
    fe, sp = [], []
    last_fe = None
    t0 = time.time()
    nxt_stall = t0 + 0.5     # 0x15D 每 0.5s 查一次：主动 read 会抢总线，会污染采样
    stalled = False
    while time.time() - t0 < DWELL:
        for _ in h._pump(0.004):
            pass
        cur = h.latest.get(0x121)
        if cur is not None and cur != last_fe:
            last_fe = cur
            fe.append(s32(cur) / 1000.0)
            sp.append(float(s32(h.latest.get(0x120) or 0)))
        if time.time() >= nxt_stall:
            nxt_stall += 0.5
            if h.read(0x15D) == 1:
                stalled = True
                break
    h.write(0x11F, 0)
    _, s_rms, s_pp, _, _ = stats(sp)
    f_mean, f_rms, f_pp, _, _ = stats(fe)
    dur = max(0.3, time.time() - t0 - 0.9)
    fs = len(fe) / dur
    fd = dom_freq(fe, fs)
    print(f'  {tgt:6d} {stats(sp)[0]:9.0f} {s_rms:8.1f} {s_pp:8.1f} |'
          f' {f_mean:7.2f} {f_rms:7.2f} {f_pp:7.2f} | {fd:6.1f} fs={fs:5.0f} '
          f'{"STALL@%.1fs" % (time.time()-t0) if stalled else "OK"}', flush=True)
    time.sleep(0.35)

h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
