"""双向角度模式回归（01f4bac）。角度模式到位后持续伺服保持、不自动停，
所以每腿跑固定时长后测稳态误差，并统计速度符号证明真反转 + 无发散。"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

def signed_err(target, now):
    return ((now - target + 180.0) % 360.0) - 180.0

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x15F, 1); h.write(0x123, 1000); h.write(0x124, 1); time.sleep(0.4)
h.write(0x12F, 200)   # 角度限速 200RPM（温和）
h.write(0x130, 800)   # 角度限加速度

def leg(label, target_deg, dur):
    h.write(0x128, int(target_deg * 1000))
    t0 = time.time(); neg = 0; pos = 0; peak = 0.0
    while time.time() - t0 < dur:
        for _ in h._pump(0.03): pass
        sp = s32(h.read(0x120))
        if sp < -20: neg += 1
        elif sp > 20: pos += 1
        if h.read(0x15D) == 1:
            print(f'{label}: STALL 误报!'); h.write(0x15F, 1); h.write(0x124, 1); return
    # 末段稳态：最后 0.4s 采样
    time.sleep(0.4)
    for _ in h._pump(0.3): pass
    now = (s32(h.read(0x116)) / 1000.0) % 360.0
    e = signed_err(target_deg, now)
    hold = abs(s32(h.read(0x120)))
    print(f'{label} 目标{target_deg:.1f}°: 实测{now:.1f}° 稳态误差{e:+.2f}° '
          f'正向拍{pos} 反向拍{neg} 保持速{hold}RPM '
          f'[{"发散!" if hold>25 else "收住"}]')

base = (s32(h.read(0x116)) / 1000.0) % 360.0
print(f'起始角 {base:.1f}°')
leg('A 正转', (base + 90.0) % 360.0, 1.5)
leg('B 反转回位', base, 1.5)
leg('C 反向90', (base - 90.0) % 360.0, 1.5)
leg('D 回位', base, 1.5)
# 绕缝：从 base 走到 base+350（正向一大圈，跨 0/360）
leg('E 绕缝+350', (base + 350.0) % 360.0, 3.0)
leg('F 绕缝回', base, 3.0)

h.write(0x163, 1); t1 = time.time()
while time.time() - t1 < 5:
    for _ in h._pump(0.02): pass
    if abs(s32(h.read(0x120))) < 15: break
h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能，运动源交还外部跟随')
