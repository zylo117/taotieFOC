"""带内微抖 A/B：用「穿带成功率」当指标——失败也是数据，不需要它一定活下来。

每次尝试前严格清场并验证状态（0x162==1 且 0x15D==0），否则不计入样本。
用法：py -X utf8 scripts/tt_dither_ab_tmp.py [accel] [目标RPM] [每配置次数]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

ACCEL = int(sys.argv[1]) if len(sys.argv) > 1 else 400
TARGET = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
N = int(sys.argv[3]) if len(sys.argv) > 3 else 8
BAND_LO, BAND_HI = 350, 850
# (名称, 使能, 深度%, 频率Hz)
CONFIGS = [('关闭', 0, 4, 7), ('开启 4%/7Hz', 1, 4, 7), ('开启 10%/7Hz', 1, 10, 7)]


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass


def clean_state():
    """清堵转 + 使能，并验证到位；返回 False 表示这次尝试不算数。"""
    for _ in range(3):
        h.write(0x15F, 1)
        h.write(0x124, 1)
        time.sleep(0.3)
        if (h.read(0x15D) or 0) == 0 and (h.read(0x162) or 0) == 1:
            return True
    return False


def soft_stop():
    h.write(0x163, 1)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        for _ in h._pump(0.02):
            pass
        if abs(s32(h.latest.get(0x120) or 0)) < 25:
            break
    h.write(0x11F, 0)
    time.sleep(0.4)


h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x123, 1000)
h.write(0x107, 0)                      # 开环
h.write(0x16F, BAND_LO)
h.write(0x170, BAND_HI)
clean_state()
print(f'穿带 A/B：120 -> {TARGET} RPM，加速 {ACCEL} RPM/s，抖动带 {BAND_LO}~{BAND_HI}，'
      f'每配置 {N} 次', flush=True)

summary = []
for name, en, pct, hz in CONFIGS:
    h.write(0x17F, pct)
    h.write(0x180, hz)
    h.write(0x17E, en)
    time.sleep(0.15)
    ok = 0
    deaths = []
    engaged = False
    for i in range(N):
        if not clean_state():
            print(f'  [{name}] 第{i + 1}次：状态清不掉，跳过', flush=True)
            continue
        h.write(0x11A, 120)
        h.write(0x11C, ACCEL)
        h.write(0x11B, TARGET)
        h.write(0x11D, 0xFFFFFFFF)
        h.write(0x11E, 2)
        h.write(0x11F, 1)
        t0 = time.time()
        peak = 0
        res = '超时'
        while time.time() - t0 < 8.0:
            for _ in h._pump(0.01):
                pass
            v = s32(h.latest.get(0x120) or 0)
            if v > peak:
                peak = v
            if v >= TARGET * 0.95:
                if (h.read(0x181) or 0) == 1:
                    engaged = True
                res = 'OK'
                ok += 1
                break
            if (h.read(0x15D) or 0) == 1:
                res = f'失步@~{peak}'
                deaths.append(peak)
                break
        if (h.read(0x181) or 0) == 1:
            engaged = True
        print(f'  [{name}] 第{i + 1}次：{res}', flush=True)
        soft_stop()
    summary.append((name, ok, N))
    print(f'  >>> {name}：{ok}/{N}  失败转速 {deaths}  抖动实际生效={engaged}\n', flush=True)

h.write(0x17E, 0)
h.write(0x125, 1)
h.write(0x16D, 1)
print('汇总：' + '  '.join(f'{n}={a}/{b}' for n, a, b in summary), flush=True)
print('已关抖动、失能、运动源交还外部跟随', flush=True)
h.close()
