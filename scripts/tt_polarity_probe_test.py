"""新固件极性探针端到端验证：触发 0x136 校准，观测探针 1/4 圈、学习段运动、
完成后读 0x176。判读：
  - 触发后 ~1s：角度应出现 ~+90° 快速行程（探针），随后回零
  - 之后几十秒内应持续出现周期性步进（800点x2趟学习）
  - 静止且 elapsed<20s => 探针失败退出；elapsed>60s 后静止 => 学习完成
  - 0x176：正常磁钢应=0；反装应=1（配合翻转磁铁第二轮测试）
"""
import sys, time
sys.path.insert(0, 'scripts')
from tt_host import TTHost, R_ENCODER_ANGLE_MDEG

def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v

h = TTHost('COM9')
time.sleep(0.5)
for _ in h._pump(0.3): pass
h.write(0x16D, 0)          # 接管运动通道
h.write(0x15F, 1)          # 清堵转
h.write(0x124, 1)          # 使能
time.sleep(0.2)

def unwrap_sample(window_s):
    tot = 0.0
    prev = None
    t0 = time.time()
    while time.time() - t0 < window_s:
        for _ in h._pump(0.05): pass
        a = h.latest.get(R_ENCODER_ANGLE_MDEG)
        if a is None:
            continue
        deg = s32(a) / 1000.0
        if prev is not None:
            d = deg - prev
            while d > 180: d -= 360
            while d < -180: d += 360
            tot += d
        prev = deg
    return tot

base = unwrap_sample(0.5)
print(f"触发前 0.5s 漂移基线: {base:+.2f}°")
print(">>> 写 0x136=1 触发校准（含极性探针）")
h.write(0x136, 1)

# 探针窗口：逐秒看行程
for i in range(10):
    tr = unwrap_sample(1.0)
    sp = h.latest.get(0x120)
    print(f"  t+{i+1:>2}s  行程 {tr:+8.2f}°  速度 {s32(sp) if sp is not None else '??'} RPM")
    if i == 2:
        print("  （探针特征：前 ~0.5s 出现 ~+90° 快速行程，随后归零回拉）")

t0 = time.time()
still_windows = 0
result = None
while time.time() - t0 < 330:
    tr = abs(unwrap_sample(10.0))
    el = time.time() - t0 + 10
    if tr < 5:
        still_windows += 1
    else:
        still_windows = 0
    print(f"  学习窗口 {el:5.0f}s 行程|{tr:7.2f}|° still={still_windows}")
    if el > 45 and still_windows >= 3:
        result = "DONE-or-ABORTED"
        break
    if el < 15 and still_windows >= 3:
        result = "EARLY-ABORT（探针失败嫌疑）"
        break

print(f"结束状态: {result}, 总用时 {time.time()-t0+10:.0f}s")
print("0x176 ENCODER_INVERT =", h.read(0x176))
print("0x15D STALL =", h.read(0x15D))
h.write(0x16D, 1)   # 归还外部运动源（板上默认形态）
h.write(0x125, 1)   # 失能（校准 finish 已恢复参数；保持台架原状）
h.close()
