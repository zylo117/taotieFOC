"""补偿通路判定：巡航中读 0x165（compensated_rps_×1000）是否离开 0。
0x165 恒 0 → 补偿块被门掉（0x107 / calibration_stage≠IDLE）；
0x165 在动 → 环路活着，"不补"只剩终点无伺服那个设计缺口。
用法：py -X utf8 scripts/tt_comp_probe_tmp.py [RPM]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

rpm = int(sys.argv[1]) if len(sys.argv) > 1 else 300
h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass
print(f'0x107 补偿使能 = {h.read(0x107)}   0x167 残差死区 = {h.read(0x167)} mdeg   '
      f'0x182 量程 = {h.read(0x182)} RPM')
if (h.read(0x107) or 0) != 1:
    h.write(0x107, 1)
    time.sleep(0.2)
    print(f'  -> 写 0x107=1，读回 {h.read(0x107)}')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(0x11A, min(120, rpm))
h.write(0x11B, rpm)
h.write(0x11C, 600)
h.write(0x11D, 0xFFFFFFFF)
h.write(0x11E, 2)
h.write(0x11F, 1)
time.sleep(2.5)                      # 进入巡航
print('\n巡航中每 0.4s 采一次（0x165=补偿rps×1000，1微步/s=0.156）：')
nz = 0
for i in range(12):
    fe = h.read(0x121)
    comp = h.read(0x165)
    cmd = h.read(0x135)
    enc = h.read(0x120)
    print(f'  FE={fe} mdeg  补偿0x165={comp}  命令0x135={cmd} mRPM  实测0x120={enc} RPM')
    if comp not in (None, 0):
        nz += 1
    time.sleep(0.4)
print(f'\n判定：{nz}/12 次采样 0x165 非零 → '
      f'{"补偿块在工作，问题在终点无伺服/死区" if nz >= 3 else "补偿块被门掉（0x107 读回、calibration_stage、0x167）"}')
h.write(0x163, 1)
time.sleep(2.0)
print(f'停车后：FE={h.read(0x121)} mdeg  0x165={h.read(0x165)}  '
      f'蠕行 0x183={h.read(0x183)}/0x184={h.read(0x184)}  catchup 0x178={h.read(0x178)}')
h.write(0x11F, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
h.close()
