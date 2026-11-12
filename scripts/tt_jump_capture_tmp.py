"""漂移取证：复现 1000RPM 跑并捕获固件 [JUMP] 打印（跳变幅/原因/时刻）。
FE 只在静止时读（端点可信）。3 轮 15s，间隔 4s。"""
import re, sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost

def s32(v): return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9'); time.sleep(0.5)
for _ in h._pump(0.4): pass

# 钩住 _parse：帧解析前先从缓冲区捞 [JUMP] 行（字节留给原解析器丢）
orig_parse = h._parse
t_start = time.time()
def parse_hook(_orig=orig_parse):
    try:
        data = bytes(h._rx)
        for m in re.finditer(rb'\[JUMP\][^\r\n]*', data):
            print(f'  t+{time.time()-t_start:6.2f}  {m.group().decode(errors="replace")[:200]}')
    except Exception:
        pass
    return _orig()
h._parse = parse_hook

h.write(0x16D, 0); time.sleep(0.15)
for _ in range(4):
    if h.read(0x16D) == 0: break
    time.sleep(0.2)
h.write(0x107, 0); h.write(0x15F, 1); h.write(0x124, 1); time.sleep(0.3)

for rnd in range(3):
    for _ in h._pump(0.3): pass
    fe0 = s32(h.read(0x121)) / 1000.0
    h.write(0x11A, 60); h.write(0x11B, 1000); h.write(0x11C, 1000)
    h.write(0x11D, 2000000); h.write(0x11E, 2); h.write(0x11F, 1)
    t0 = time.time(); locked = False
    while time.time() - t0 < 15:
        for _ in h._pump(0.05): pass
        if h.read(0x15D) == 1: locked = True; break
    h.write(0x163, 1); t1 = time.time()
    while time.time() - t1 < 3.5:
        for _ in h._pump(0.02): pass
        if abs(s32(h.read(0x120))) < 15: break
    time.sleep(0.8)
    for _ in h._pump(0.3): pass
    fe1 = s32(h.read(0x121)) / 1000.0
    print(f'轮{rnd+1}: FE {fe0:.2f} → {fe1:.2f}  Δ={fe1-fe0:+.2f}° {"[堵转]" if locked else ""}')
    h.write(0x15F, 1)
    for _ in h._pump(3.5): pass

h.write(0x125, 1); h.write(0x16D, 1); h.close()
print('已失能')
