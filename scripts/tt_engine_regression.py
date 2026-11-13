"""统一引擎回归总表：
T1 位置正转 120→1000 a600 640000   T2 a150 同行程
T3 位置反转 120→800 a300 256000    T4 速度 2000RPM 10s 漂移
T5 角度模式 +90°往返               T6 缓停(0x163)@1000
T7 往复模式 640000 总行程          全部缓停+测完失能"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

def wait_idle(h, tmo):
    t0 = time.time()
    while time.time() - t0 < tmo:
        for _ in h._pump(0.03): pass
        if h.latest.get(0x11F) == 0 and time.time() - t0 > 0.5:
            return True
        if h.read(0x15D) == 1:
            return 'STALL'
    return False

def run_counted(h, mode, srpm, mrpm, accel, count, tmo):
    # 【测试间恢复】堵转守卫的动作是“停机+关驱动器输出”，只清 0x15F 不重新使能的话，
    # 下一条测试是在电机没通电的状态下跑的 —— 编码器不动 → 又判堵转，整条链全假。
    # （实测级联：T1 一锁，T2/T3/T7 全部 STALL，而会自己 requestDriverEnable(true)
    #   的角度模式 T5 反而正常，正是这个特征的指纹。）
    h.write(0x15F, 1)
    h.write(0x124, 1)
    time.sleep(0.25)
    h.write(0x11A, srpm); h.write(0x11B, mrpm); h.write(0x11C, accel)
    h.write(0x11D, count); h.write(0x11E, mode); h.write(0x11F, 1)
    t0 = time.time(); peak = 0.0
    while time.time() - t0 < tmo:
        for _ in h._pump(0.03): pass
        f = s32(h.read(0x121)) / 1000.0
        if abs(f) > abs(peak): peak = f
        if h.latest.get(0x11F) == 0 and time.time() - t0 > 1.0:
            break
        if h.read(0x15D) == 1:
            return peak, None, 'STALL'
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    return peak, s32(h.read(0x121)) / 1000.0, None

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x123, 1000)
h.write(0x124, 1); time.sleep(0.4)

pk, fe, err = run_counted(h, 0, 120, 1000, 600, 640000, 30)
print(f'T1 位置 fwd a600: 峰值{pk:.1f}° 末值{fe if fe is None else round(fe,2)}° {err or "OK"}')
time.sleep(0.5)
pk, fe, err = run_counted(h, 0, 120, 1000, 150, 640000, 40)
print(f'T2 位置 fwd a150: 峰值{pk:.1f}° 末值{fe if fe is None else round(fe,2)}° {err or "OK"}')
time.sleep(0.5)
# T3 反转：运动中必须有负速度样本（物理反转确认），且不误报堵转
h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.25)
h.write(0x11A, 120); h.write(0x11B, 800); h.write(0x11C, 300)
h.write(0x11D, 224000); h.write(0x11E, 1); h.write(0x11F, 1)
t0 = time.time(); sps = []; st3 = None; fe3 = 0.0
while time.time() - t0 < 20:
    for _ in h._pump(0.03): pass
    v = s32(h.read(0x120)); sps.append(v)
    f = s32(h.read(0x121)) / 1000.0; fe3 = min(fe3, f)
    if h.read(0x15D) == 1: st3 = 'STALL'; break
    if h.latest.get(0x11F) == 0 and time.time() - t0 > 1.0: break
neg = sum(1 for v in sps if v < -100)
time.sleep(0.8)
for _ in h._pump(0.3): pass
fe3e = s32(h.read(0x121)) / 1000.0
print(f'T3 位置 REV a300: {"STALL" if st3 else "完成"} 负速样本{neg}/{len(sps)} '
      f'峰值FE{fe3:.1f}° 末值{fe3e:.2f}° {"OK" if (not st3 and neg > 3) else "FAIL"}')
time.sleep(0.5)

# T4 速度 2000 无限计数 10s：漂移
h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.25)
h.write(0x11A, 120); h.write(0x11B, 2000); h.write(0x11C, 1000)
h.write(0x11D, 0xFFFFFFFF); h.write(0x11E, 2); h.write(0x11F, 1)
t0 = time.time(); pts = []
while time.time() - t0 < 10:
    for _ in h._pump(0.05): pass
    if h.read(0x15D) == 1: break
    if time.time() - t0 > 2.5 and (not pts or time.time() - pts[-1][0] >= 1.0):
        pts.append((time.time() - t0, s32(h.read(0x121)) / 1000.0))
h.write(0x163, 1)
t1 = time.time()
while time.time() - t1 < 8:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
n = len(pts); slope = 0.0
if n >= 3:
    mt = sum(p[0] for p in pts)/n; mf = sum(p[1] for p in pts)/n
    den = sum((t-mt)**2 for t, _ in pts)
    slope = sum((t-mt)*(f-mf) for t, f in pts)/den if den else 0.0
print(f'T4 速度2000 10s: 漂移{slope:+.2f}°/s FE {[round(f,1) for _,f in pts]}')
h.write(0x15F, 1); time.sleep(0.8)

# T5 角度模式 +90/-90（目标取模到合法绝对角）
for _ in h._pump(0.3): pass
base = (s32(h.read(0x116)) / 1000.0) % 360.0
for tgt_off in (90.0, 0.0):
    tgt = (base + tgt_off) % 360.0
    h.write(0x128, int(tgt * 1000))
    r = wait_idle(h, 15)
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    now = (s32(h.read(0x116)) / 1000.0) % 360.0
    d = ((now - tgt + 180.0) % 360.0) - 180.0
    print(f'T5 角度→{tgt:.1f}°: {"STALL" if r == "STALL" else "到位" if r is True else "超时"} 实测{now:.1f}° 误差{d:+.1f}°')
    if r == 'STALL':
        h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.5)
h.write(0x107, 0)  # 角度模式带过的补偿开关复原
time.sleep(0.5)

# T6 缓停
h.write(0x11A, 120); h.write(0x11B, 1000); h.write(0x11C, 600)
h.write(0x11D, 0xFFFFFFFF); h.write(0x11E, 2); h.write(0x11F, 1)
time.sleep(3.0)
h.write(0x163, 1)
t1 = time.time(); ok = False
while time.time() - t1 < 6:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: ok = True; break
print(f'T6 1000RPM 缓停: {"平滑停止" if ok else "超时!!"} 用时{time.time()-t1:.1f}s')
h.write(0x15F, 1); time.sleep(0.8)

# T7 往复模式：总 640000，去320000回320000
pk, fe, err = run_counted(h, 6, 120, 600, 600, 640000, 60)
print(f'T7 往复640000: 峰值{pk:.1f}° 末值{fe if fe is None else round(fe,2)}° {err or "OK"}')

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能，运动源已交还外部跟随')
