"""分析 trace_*.csv：振荡主频、是否随转速走、能量落在哪。"""
import glob, math, os


def read_csv(p):
    ts, fe, rpm = [], [], []
    with open(p, encoding='utf-8') as f:
        next(f, None)
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != 3:
                continue
            ts.append(float(parts[0]))
            rpm.append(float(parts[1]))
            fe.append(float(parts[2]))
    return ts, rpm, fe


def dft_peak(xs, fs, f_lo=2.0, f_hi=None):
    """去均值后在 [f_lo, f_hi] 扫频，返回 (峰值频率, 峰值幅, 总AC能量)。"""
    n = len(xs)
    if n < 16:
        return 0.0, 0.0, 0.0
    m = sum(xs) / n
    y = [x - m for x in xs]
    energy = sum(v * v for v in y)
    if f_hi is None:
        f_hi = fs * 0.45
    best_f, best_a = 0.0, 0.0
    f = f_lo
    while f <= f_hi:
        re = im = 0.0
        w = 2.0 * math.pi * f / fs
        for i, v in enumerate(y):
            re += v * math.cos(w * i)
            im += v * math.sin(w * i)
        a = 2.0 * math.hypot(re, im) / n
        if a > best_a:
            best_a, best_f = a, f
        f += 0.5
    return best_f, best_a, energy


print('  转速  fs(Hz)  n  | FE主频Hz  FE主频幅°  FE AC-RMS° | 转速主频Hz  转速主频幅RPM')
for p in sorted(glob.glob('trace_*.csv'), key=lambda s: int(s.split('_')[1].split('.')[0])):
    tgt = int(os.path.basename(p).split('_')[1].split('.')[0])
    ts, rpm, fe = read_csv(p)
    if len(ts) < 16:
        print(f'  {tgt:5d}  样本太少({len(ts)})，跳过')
        continue
    dur = ts[-1] - ts[0]
    fs = (len(ts) - 1) / dur if dur > 0 else 0.0
    if fs < 20:
        print(f'  {tgt:5d}  fs过低({fs:.1f}Hz)，跳过')
        continue
    f1, a1, e1 = dft_peak(fe, fs)
    f2, a2, _ = dft_peak(rpm, fs)
    ac_rms = math.sqrt(e1 / len(fe))
    print(f'  {tgt:5d} {fs:7.1f} {len(fe):3d} | {f1:8.1f} {a1:9.2f} {ac_rms:10.2f} |'
          f' {f2:10.1f} {a2:12.1f}')
print(f'\n奈奎斯特 = fs/2 ≈ {28}Hz 附近：高于它的振荡会被折叠，只能看到 <25Hz 的成分。')
