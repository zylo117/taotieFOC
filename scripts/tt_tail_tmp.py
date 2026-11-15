"""收尾抖一刀切判据：那 1.83° 到底是「软件真发了脉冲」还是「转子自己跳」。

原理（不需要改固件）：0x116 = 编码器绝对角，0x121 = 跟随误差 = 命令位置 − 编码器位置。
  · 若是软件多发了脉冲  -> 命令位置同步前进 -> 0x116 动、0x121 基本不动
  · 若是转子自己跳(引脚/齿位) -> 命令位置不变 -> 0x116 动、0x121 反向变化同样大小
两个量来自同一批遥测帧，配对后作差即可判定。

时间标记两个：
  t_ledger  账本 0x134 到 N（软件认为发完了）
  t_done    0x11F 由 1 变 0（= SETTLE 结束、stopMotion() 真正执行的那一刻）
沉降参数 STEP_SETTLE_MIN_US=15ms / TIMEOUT=200ms，所以 t_done 落在脉冲停止后 15~200ms，
正好是「减速到 0，过一会儿抖一下」的位置。看这一刀跟哪个标记对齐。

用法： python scripts/tt_tail_tmp.py          （250RPM / 300RPM/s / 10 圈，开环自然收尾）
       python scripts/tt_tail_tmp.py quick    （同样参数，但中途 0x163 缓停）
"""
import re
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost, R_MOTION, R_SCHEDULED, R_ENCODER_ANGLE_MDEG, R_FOLLOW_ERR

QUICK = len(sys.argv) > 1 and sys.argv[1] == 'quick'
RPM, ACCEL, PULSES = 250, 300, 64000
TAIL_S = 1.5

PRINTABLE = re.compile(rb'[\x20-\x7e\r\n]{14,}')


class Sniff(TTHost):
    """顺带把串口上的 ASCII 日志（[SETTLE]/[PUL] 等）捞出来，不改父组帧逻辑。"""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.lines = []
        self._seen = set()

    def _parse(self):
        for m in PRINTABLE.finditer(bytes(self._rx)):
            s = m.group().decode('ascii', 'ignore').strip()
            if s and s not in self._seen:
                self._seen.add(s)
                self.lines.append(s)
        return super()._parse()


def unwrap(x):
    return x - 360000 if x > 180000 else (x + 360000 if x < -180000 else x)


h = Sniff('COM9')
print(f'口径核对 0x100={h.read(0x100)} 0x182={h.read(0x182)}（期望 32 / 14062）')

h.write(0x16D, 0)          # 运动源交给内部规划
h.write(0x15F, 1)          # 清堵转
h.write(0x124, 1)          # 使能
time.sleep(1.2)            # 使能拉入的 ~0.19° 抖完再取基线
for _ in h._pump(0.15):
    pass
base = h.latest.get(R_ENCODER_ANGLE_MDEG)
if base is None:
    print('没收到遥测 0x116，退出'); h.close(); sys.exit(1)
print(f'基线角 {base / 1000.0:.3f}°   模式={"缓停" if QUICK else "自然收尾"}')

h.write(0x107, 0)          # 开环（结尾必须恢复，见 finally）
h.write(0x11A, 120)
h.write(0x11B, RPM)
h.write(0x11C, ACCEL)
h.write(0x11D, PULSES)
h.write(0x11E, 0)

events = []                # (t, reg, val)
t_start = time.time()
t_ledger = t_done = None
quick_done = False
try:
    h.write(0x11F, 1)
    running = False
    while time.time() - t_start < 40:
        for cmd, reg, val in h._pump(0.01):
            if cmd != 0x10:
                continue
            t = time.time()
            events.append((t, reg, val))
            if reg == R_MOTION:
                if val == 1:
                    running = True
                elif running and val == 0 and t_done is None:
                    t_done = t
            if reg == R_SCHEDULED and val >= PULSES and t_ledger is None:
                t_ledger = t
        if QUICK and not quick_done and (h.latest.get(R_SCHEDULED) or 0) >= PULSES // 2:
            h.write(0x163, 1)
            quick_done = True
            print(f'已写 0x163 缓停（账本 {h.latest.get(R_SCHEDULED)}）')
        if t_done is not None and time.time() - t_done > TAIL_S:
            break
        if not running and time.time() - t_start > 8:
            break
finally:
    h.write(0x11F, 0)
    time.sleep(2.5)                       # 停稳后再读落点
    final = h.read(R_ENCODER_ANGLE_MDEG)
    print(f'\n恢复：0x107={h.write(0x107, 1)}  0x16D={h.write(0x16D, 1)}  '
          f'0x125={h.write(0x125, 1)}')
    h.close()

# ---------------- 报告 ----------------
if t_ledger is None:
    t_ledger = t_done
if t_ledger is None:
    print('没抓到结束标记，只有原始事件：', len(events)); sys.exit(1)

fe = {}
for t, reg, val in events:
    if reg == R_FOLLOW_ERR:
        fe[t] = val


def fe_near(t):
    best = None
    for tt, vv in fe.items():
        if abs(tt - t) < 0.045 and (best is None or abs(tt - t) < abs(best[0] - t)):
            best = (tt, vv)
    return best


angles = sorted([(t, v) for t, reg, v in events if reg == R_ENCODER_ANGLE_MDEG
                 and t >= t_ledger - 0.05])
if t_done is not None:
    print(f'\nt_done(0x11F→0 = stopMotion 执行时刻) 比 t_ledger 晚 '
          f'{(t_done - t_ledger) * 1000:.1f} ms')
else:
    print('\nt_done 没抓到（只用 t_ledger 对齐）')
print('\n时刻(ms)   Δ角(°)    0x121 跟随误差      ΔFE(°)')
a0 = f0 = None
run = 0.0
for t, a in angles:
    nf = fe_near(t)
    if a0 is None:
        a0 = a
        f0 = nf[1] if nf else None
    da = unwrap(a - a0) / 1000.0
    df = unwrap(nf[1] - f0) / 1000.0 if (nf and f0 is not None) else float('nan')
    run += da
    flag = '  <-- 抖一刀' if abs(da) > 0.3 else ''
    fe_now = f'{nf[1] / 1000.0:+7.3f}' if nf else '    n/a'
    print(f'{(t - t_ledger) * 1000:7.0f}  {da:+7.3f}   {fe_now}   {df:+7.3f}{flag}')

print(f'\n账本到 N 之后的编码器总位移 = {run:.3f}°')
print(f'落点核对：基线 {base / 1000.0:.3f}° -> 终值 {final / 1000.0 if final else float("nan"):.3f}°'
      f'   差 {unwrap(final - base) / 1000.0 if final else float("nan"):+.3f}°（{PULSES} 步 = 10 整圈，应为 ~0）')
print(f'判据：Δ角 与 ΔFE 同幅值(≈{run:.2f}°) => 转子自己跳（软件没发脉冲，引脚/齿位问题）')
print(f'      ΔFE ≈ 0 而角动了 => 软件真发了脉冲，回到账本/发波路径')
print(f'取证 0x183={h.latest.get(0x183, "?")}(蠕行事件)')
if h.lines:
    print('\n抓到的固件日志：')
    for s in h.lines[-14:]:
        print('  ', s.replace('\r', ' ').replace('\n', ' '))
