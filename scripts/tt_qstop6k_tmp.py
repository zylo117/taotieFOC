"""缓停抖动量化：250RPM 跑 64000 步，发到 32000 步时写 0x163 缓停。
用 0x116 绝对角逐窗口积分，找两件事：
  ① 反向位移（负增量）＝转子往回弹，就是"抖一下"
  ② 速度尖峰（超过缓停前的巡航速率）＝多发脉冲/活流未收
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

h = TTHost('COM9')
time.sleep(0.4)
list(h._pump(0.3))
print(f'口径核对 0x100={h.read(0x100)}  0x182={h.read(0x182)}  （期望 32 / 14062）')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)                       # 使能后静置，避开拉入抖动
h.write(0x107, 0)                     # 开环，排除补偿变量
h.write(0x11A, 120)
h.write(0x11B, 250)
h.write(0x11C, 6000)
h.write(0x11D, 64000)                 # 10 圈
h.write(0x11E, 0)
h.write(0x11F, 1)

prev = None
while True:                           # 等发到 32000 步
    for _ in h._pump(0.02):
        pass
    if (h.read(0x134) or 0) >= 32000:
        break
    if time.time() > 0 and (h._last_t0 if False else 0):
        pass

cruise = h.read(0x134)
h.write(0x163, 1)                     # ← 真正的缓停（上一版漏了这一行）
print(f'\n已发 {cruise} 步，已写 0x163 缓停')
print('每 100ms 一窗：位移增量(°)  等效转速(RPM)   账本 0x134')
base = h.latest.get(0x116)
prev = base
rev_deg = 0.0
spikes = []
for k in range(18):
    t0 = time.time()
    d = 0.0
    while time.time() - t0 < 0.1:
        for _ in h._pump(0.01):
            pass
        a = h.latest.get(0x116)
        if a is not None and prev is not None:
            x = a - prev
            if x > 180000:
                x -= 360000
            elif x < -180000:
                x += 360000
            d += x
            prev = a
    deg = d / 1000.0
    rpm = deg / 360.0 / 0.1 * 60.0
    if d < 0:
        rev_deg += deg
        spikes.append(f'第{k}窗 反向 {deg:.3f}°')
    if rpm > 290:
        spikes.append(f'第{k}窗 速度尖峰 {rpm:.0f} RPM')
    print(f'  窗{k:2d}  Δ={deg:+8.3f}°   {rpm:+7.1f} RPM   0x134={h.read(0x134)}')
print(f'\n结论：反向位移合计 {rev_deg:.3f}°   异常窗：{spikes if spikes else "无"}')
print(f'缓停后 0x121={h.read(0x121)} mdeg  0x183={h.read(0x183)}/0x184={h.read(0x184)}  '
      f'0x178={h.read(0x178)}')
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()

