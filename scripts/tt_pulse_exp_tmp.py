"""决定性实验：脉宽 200ns→1000ns（TMC spec），再 64细分→16细分。
若脉宽修复高速 → 驱动输入级没吃进 200ns 脉冲。
若 16细分能到 4 倍速 → 天花板在脉冲速率路径；若同 RPM 死 → 转矩/电压限。
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9')
time.sleep(0.6)
for _ in h._pump(0.4): pass
h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0:
        break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1)
ms0 = h.read(0x100)
pw0 = h.read(0x123)
print(f'当前 细分={ms0} 脉宽={pw0}ns')

def sweep(rpm_list, tag):
    print(f'--- {tag} ---')
    for rpm in rpm_list:
        h.write(0x11A, 60); h.write(0x11B, rpm); h.write(0x11C, 150); h.write(0x11D, 0)
        h.write(0x11E, 2); h.write(0x11F, 1)
        t0 = time.time(); locked = False
        while time.time() - t0 < (rpm - 60) / 150 + 0.4:
            for _ in h._pump(0.01): pass
            if h.read(0x15D) == 1:
                locked = True; break
        speeds = []
        t1 = time.time()
        while not locked and time.time() - t1 < 1.5:
            for _ in h._pump(0.01): pass
            sp = h.latest.get(0x120)
            if sp is not None: speeds.append(s32(sp))
            if h.read(0x15D) == 1: locked = True
        med = sorted(speeds)[len(speeds) // 2] if speeds else 0
        print(f'  {rpm}: {"堵转" if locked else "OK"} 速度中位={med}')
        h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
        time.sleep(0.6)

sweep([400, 450, 550], '脉宽 1000ns')
h.write(0x123, 1000)
time.sleep(0.3)
sweep([400, 450, 550], '脉宽写 1000 后')
# 注意：脉宽生效需确认 emit 用的是 step_pulse_width_ns_——先看结果再说
h.close()
