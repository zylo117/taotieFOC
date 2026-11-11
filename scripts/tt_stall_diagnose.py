"""450 失步定位：同时采样 指令速度(0x14C mRPM) vs 编码器速度(0x120 mRPM) vs 堵转(0x15D)。
判据：
  指令=450000 编码器=0   → 电机真堵转（转矩/共振），非编码器问题
  指令=450000 编码器乱跳 → 编码器高速丢读
  指令本身<450000        → 规划器速度根本没爬上去（代码 bug）
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
time.sleep(0.3)
target = int(sys.argv[1]) if len(sys.argv) > 1 else 450
h.write(0x11A, 60); h.write(0x11B, target); h.write(0x11C, 150); h.write(0x11D, 0)
h.write(0x11E, 2); h.write(0x11F, 1)
print(f'目标 {target} RPM，每 100ms 采样 指令/编码器/堵转：')
t0 = time.time()
prev_cmd = None
while time.time() - t0 < 5.0:
    cmd = h.latest.get(0x14C)
    enc = h.latest.get(0x120)
    stall = h.read(0x15D)
    cmdv = s32(cmd) / 1000.0 if cmd is not None else None
    encv = s32(enc) if enc is not None else None
    if cmdv is not None or encv is not None:
        print(f'  t={time.time()-t0:4.2f}s 指令={cmdv} RPM  编码器={encv} RPM  堵转={stall}')
    if stall == 1:
        print('  → 堵转锁存，停止采样')
        break
    for _ in h._pump(0.1): pass
h.write(0x11F, 0); h.write(0x15F, 1); h.write(0x124, 1)
time.sleep(0.4)
h.write(0x125, 1); h.write(0x16D, 1)
h.close()
