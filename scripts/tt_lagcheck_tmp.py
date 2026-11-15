"""恒速段补偿为什么看着不输出：把三项并排量出来。
补偿进 PID 的量 = 0x121(FE) − 0x151(滞后角表值) − 加速滞后，再扣 0x167 死区。
  |FE − lag| < 死区  => 设计如此：物理滞后已被前馈抵掉，恒速段本就不该有输出
  |FE − lag| >> 死区 => 表不准或门有问题，补偿该动却没动
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

rpm = int(sys.argv[1]) if len(sys.argv) > 1 else 250
DEADBAND = int(sys.argv[2]) if len(sys.argv) > 2 else None


def s32(v):
    return v - 0x100000000 if v and (v & 0x80000000) else v


h = TTHost('COM9')
print(f'0x107={h.read(0x107)}  0x167 死区={h.read(0x167)} mdeg  0x166 加速滞后系数={h.read(0x166)}')
if (h.read(0x107) or 0) != 1:
    h.write(0x107, 1)
    print(f'  -> 写 1，读回 {h.read(0x107)}')
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
time.sleep(3.0)                       # 进巡航 + 越过起步屏蔽/滞后收敛

if DEADBAND is not None:
    h.write(0x167, DEADBAND)
    print(f'[A/B] 0x167 已临时改为 {DEADBAND} mdeg（脚本结束还原 400）')

print('\nRPM实测   FE(°)    滞后表0x151(°)   FE−lag(°)   死区(°)   补偿0x165(rps)')
rows = []
try:
    for i in range(10):
        fe = s32(h.read(0x121)) / 1000.0
        lag = s32(h.read(0x151)) / 1000.0
        db = (h.read(0x167) or 0) / 1000.0
        comp = s32(h.read(0x165)) / 1000.0
        enc = h.read(0x120)
        res = fe - lag
        rows.append((fe, lag, res, comp))
        print(f'  {enc}     {fe:+7.3f}    {lag:+7.3f}      {res:+7.3f}    {db:.3f}     {comp:+.4f}')
        time.sleep(0.4)
finally:
    h.write(0x163, 1)
    time.sleep(2.0)
    if DEADBAND is not None:
        h.write(0x167, 400)
        print(f'[A/B] 0x167 已还原：读回 {h.read(0x167)} mdeg')

avg_res = sum(r[2] for r in rows) / len(rows)
avg_comp = sum(r[3] for r in rows) / len(rows)
print(f'\n平均残差 FE−lag = {avg_res:+.3f}°   平均补偿输出 = {avg_comp:+.4f} rps')
print('判读：残差落在死区内 => 恒速段无输出是设计如此（滞后角已被前馈抵掉），'
      '补偿只在暂态/落点起作用；残差明显超出死区而输出仍≈0 => 门或增益有问题，继续查补偿块。')
h.write(0x11F, 0)
h.write(0x16D, 1)
h.close()
