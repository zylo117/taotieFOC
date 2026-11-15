"""验证「位置整段消失」是不是失能造成的：跑完 50 微步后，只写一次 0x125 失能，
什么运动都不再命令，只看角度掉不回掉。掉回去 = 断流弹回；不掉 = 另有原因。
再反向验证：失能后重新使能，位置会不会又回来（=只是保持转矩没了，不是步数丢了）。
"""
import sys
import time

sys.path.insert(0, 'scripts')
from tt_host import TTHost, R_MOTION, R_ENCODER_ANGLE_MDEG

PULSES = int(sys.argv[1]) if len(sys.argv) > 1 else 50
RPM = int(sys.argv[2]) if len(sys.argv) > 2 else 25


def unwrap(x):
    return x - 360000 if x > 180000 else (x + 360000 if x < -180000 else x)


def pump_for(h, sec):
    end = time.time() + sec
    while time.time() < end:
        for _ in h._pump(0.005):
            pass


def ang(h):
    pump_for(h, 0.25)
    return h.latest.get(R_ENCODER_ANGLE_MDEG)


h = TTHost('COM9')
print(f'口径 0x100={h.read(0x100)}  使能来源 0x169={h.read(0x169)}  0x16A={h.read(0x16A)}')
h.write(0x16D, 0)
h.write(0x15F, 1)
h.write(0x124, 1)
time.sleep(1.2)
h.write(0x107, 0)

try:
    for k in range(3):
        b0 = ang(h)
        h.write(0x11A, max(10, RPM)); h.write(0x11B, RPM)
        h.write(0x11C, 300); h.write(0x11D, PULSES)
        h.write(0x11E, 0); h.write(0x11F, 1)
        while h.latest.get(R_MOTION) != 0 and time.time() - b0 < 5:
            pump_for(h, 0.02)
        time.sleep(1.0)
        a_moved = ang(h)
        print(f'\n第{k + 1}次  运动后保持 = {unwrap(a_moved - b0) / 1000.0:+.3f}°  '
              f'(命令 {PULSES * 360.0 / 6400:.3f}°)')
        h.write(0x125, 1)                       # 只失能，不发任何运动
        time.sleep(1.0)
        a_off = ang(h)
        print(f'        失能后       = {unwrap(a_off - b0) / 1000.0:+.3f}°   '
              f'相对运动后变化 {unwrap(a_off - a_moved) / 1000.0:+.3f}°')
        h.write(0x124, 1)                       # 重新使能（会有一次拉入）
        time.sleep(1.2)
        a_on = ang(h)
        print(f'        重新使能后   = {unwrap(a_on - b0) / 1000.0:+.3f}°   '
              f'相对失能后变化 {unwrap(a_on - a_off) / 1000.0:+.3f}°')
        time.sleep(0.3)
finally:
    h.write(0x11F, 0)
    h.write(0x107, 1)
    h.write(0x16D, 1)
    h.close()

print('\n判读：失能后角度大幅往回掉 => 「卡回原位」= 断流弹回（保持转矩没了），'
      '要去查是谁在运动结束后把 EN 拉掉的（0x125 / 外部 EN 引脚 / 使能来源模式）。')
