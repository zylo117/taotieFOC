"""角度模式首动延迟插桩：那 75~83ms 里环在命令什么？

  python scripts/tt_angloop_tmp.py [角度增量°] [趟数]
判读（对照 0x18C 状态位）：
  · 死区内 out/pps 一直 0，bit0 迟迟不松  → 环自己掐了命令（A 类：状态/门限）
  · out/pps 非 0 而编码器不动             → 引擎吞命令（B 类：装载/档位/方向门）
  · prof 与 out 大小关系 → 前馈和 PID 谁在出力
优先用固件主动推送的遥测（h.latest），不够再逐寄存器问，避免把采样率拖到 10ms 以下。
跑完 0x163 退出角度模式并 0x125 失能。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost

R_ANG, R_MOTION = 0x116, 0x11F
R_PROF, R_OUT, R_PPS, R_ERR, R_BITS = 0x188, 0x189, 0x18A, 0x18B, 0x18C
STEP_UM = 0.05625
DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
REPS = int(sys.argv[2]) if len(sys.argv) > 2 else 2


def s32(v):
    return None if v is None else (v - 0x100000000 if v & 0x80000000 else v)


def unwrap(d):
    while d > 180000:
        d -= 360000
    while d < -180000:
        d += 360000
    return d


h = TTHost('COM9')
if h.read(0x16D) != 0:
    print('0x16D 外部 → 强制内部')
    h.write(0x16D, 0)
    time.sleep(0.2)
saved_closed = h.read(0x107)
h.write(0x124, 1)
time.sleep(0.8)
h.write(0x107, 1)
h.write(0x167, 400)
h.write(0x12F, 120)
h.write(0x130, 600)
h.read(R_BITS)
print(f'0x162={h.read(0x162)} 0x107=1 0x12F=120 0x130=600  阶跃 {DEG}° × {REPS}')
print('新寄存器在位？', {hex(r): h.read(r) for r in (R_PROF, R_OUT, R_PPS, R_ERR, R_BITS)})
print('固件推送的寄存器里有没有它们：',
      sorted(hex(k) for k in h.latest if k in (R_PROF, R_OUT, R_PPS, R_ERR, R_BITS)))
try:
    for rep in range(REPS):
        h.read(R_BITS)  # 让 latest 缓存新鲜一下（0x116 是只读遥测，别去写它）
        a0 = h.read(R_ANG)
        tgt = ((a0 + int(round(DEG * 1000))) % 360000)
        h.write(0x128, tgt)
        t0 = time.time()
        rec = []
        prev = a0
        moved = 0
        first_move = None
        while time.time() - t0 < 0.45:
            a = h.read(R_ANG)
            if a is None:
                continue
            moved += unwrap(a - prev)
            prev = a
            if first_move is None and abs(moved) >= 2000:   # ≥ 2 微步算真动了
                first_move = time.time() - t0
            # 固件不推送这几个诊断寄存器，必须显式读（4 次往返 ≈ 12ms/帧，
            # 死区有 84ms，够采 7 帧，足够判 A/B）
            rec.append((time.time() - t0, moved / 1000.0,
                        s32(h.read(R_OUT)), h.read(R_PPS), h.read(R_BITS)))
        print(f'\n=== 第 {rep + 1} 趟  目标 {tgt / 1000:.3f}°  首动 '
              f'{(first_move * 1000 if first_move else -1):.0f}ms  帧数 {len(rec)}')
        for t, mv, out, pps, bits in rec[:26]:
            o = '' if out is None else f'{out / 1000:.3f}'
            print(f'  {t * 1000:6.1f}ms 累计{mv:+8.3f}°  out={o:>7} rps  pps={pps:>6}  '
                  f'bits={format(bits, "07b") if bits is not None else "-"}')
        time.sleep(0.4)
        h.write(0x163, 1)
        time.sleep(0.5)
finally:
    h.write(0x163, 1, need_ack=False)
    time.sleep(0.1)
    if saved_closed is not None:
        h.write(0x107, saved_closed)
    h.write(0x125, 1, need_ack=False)
    time.sleep(0.1)
    print(f'\n已失能 0x162={h.read(0x162)}  0x107 还原={saved_closed}')
