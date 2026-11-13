"""找「有没有任何加速度能干净跳过 ~515RPM 这个尖点」：状态严格验证，成功率当指标。
用法：py -X utf8 scripts/tt_accel_cross_tmp.py [每档次数]
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

ACCELS = [200, 600, 1500, 3000]
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
TARGET = 1000


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


h = TTHost('COM9')
time.sleep(0.4)
for _ in h._pump(0.3):
    pass


def clean():
    for _ in range(3):
        h.write(0x15F, 1)
        h.write(0x124, 1)
        time.sleep(0.3)
        if (h.read(0x15D) or 0) == 0 and (h.read(0x162) or 0) == 1:
            return True
    return False


h.write(0x16D, 0)
time.sleep(0.15)
h.write(0x123, 1000)
h.write(0x107, 0)          # 开环
h.write(0x17E, 0)          # 抖动关
clean()
print(f'加速度扫描穿带（目标 {TARGET}，抖动关，每档 {N} 次）', flush=True)
for a in ACCELS:
    ok, deaths = 0, []
    for i in range(N):
        if not clean():
            print(f'  accel={a}: 状态清不掉，跳过', flush=True)
            continue
        h.write(0x11A, 120)
        h.write(0x11C, a)
        h.write(0x11B, TARGET)
        h.write(0x11D, 0xFFFFFFFF)
        h.write(0x11E, 2)
        h.write(0x11F, 1)
        t0 = time.time()
        peak = 0
        res = '超时'
        while time.time() - t0 < 6.0:
            for _ in h._pump(0.01):
                pass
            v = s32(h.latest.get(0x120) or 0)
            if v > peak:
                peak = v
            if v >= TARGET * 0.95:
                res = 'OK'
                ok += 1
                break
            if (h.read(0x15D) or 0) == 1:
                res = f'失步@~{peak}'
                deaths.append(peak)
                break
        print(f'  accel={a:5d}：{res}', flush=True)
        h.write(0x163, 1)
        t1 = time.time()
        while time.time() - t1 < 5.0:
            for _ in h._pump(0.02):
                pass
            if abs(s32(h.latest.get(0x120) or 0)) < 25:
                break
        h.write(0x11F, 0)
        time.sleep(0.4)
    print(f'  >>> accel={a}：{ok}/{N}  失败转速 {deaths}\n', flush=True)

h.write(0x125, 1)
h.write(0x16D, 1)
print('已失能、运动源交还外部跟随', flush=True)
h.close()
