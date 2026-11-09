"""观察梯形加减速：使能 -> 短行程正向位置运动 -> 采样命令/实测速度 -> 失能。
用于确认"起步是否直接跳到最高转速"。低速、短行程, 避开 300-800rpm 共振。
"""
import sys, time
sys.path.insert(0, r"D:\Documents\PlatformIO\Projects\taotieFOC\scripts")
from tt_proto import TT, R_ENABLE, R_DISABLE, R_MOTION_CMD, R_START_RPM, R_MAX_RPM, R_ACCEL, R_PULSE_COUNT, R_MOTION_MODE, R_SPEED_RPM, R_SPEED, R_MICROSTEPS, R_FAULT

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"
tt = TT(PORT)
try:
    ms = tt.read(R_MICROSTEPS)
    print("microsteps =", ms)
    steps_per_rev = 200 * (ms or 64)
    turns = 0.4
    count = int(turns * steps_per_rev)
    print("steps/rev", steps_per_rev, "pulse_count", count)

    tt.write(R_DISABLE, 0)      # 确认失能
    tt.write(R_ENABLE, 1)       # EN 低 = 使能
    time.sleep(0.3)
    print("fault", hex(tt.read(R_FAULT)))

    # 短行程正向: start=20 max=200 accel=300 (rpm/s)
    tt.write(R_START_RPM, 20)
    tt.write(R_MAX_RPM, 200)
    tt.write(R_ACCEL, 300)
    tt.write(R_PULSE_COUNT, count)
    tt.write(R_MOTION_MODE, 0)   # POSITION_FORWARD
    print("== 启动 motion ==, 0x11F=1")
    tt.write(R_MOTION_CMD, 1)

    t0 = time.time()
    last = None
    while time.time() - t0 < 3.0:
        spd = tt.read(R_SPEED_RPM)      # 实测 rpm
        cmd = tt.read(R_SPEED)          # 命令 mRPM (带方向 x1000)
        running = tt.read(R_MOTION_CMD)
        el = time.time() - t0
        line = "t=%5.2f  cmd=%7.0f rpm  meas=%6s rpm  running=%s" % (
            el, (cmd or 0) / 1000.0, (int(spd) if spd is not None else -1), running)
        if line != last:
            print(line, flush=True)
            last = line
        if running == 0:
            time.sleep(0.3)
            running2 = tt.read(R_MOTION_CMD)
            if running2 == 0:
                print("== motion 结束 ==")
                break
    time.sleep(0.5)
    # 测完再失能, 但确保运动已结束(上面已等 running==0)
    tt.write(R_DISABLE, 1)
    time.sleep(0.2)
    print("已失能。")
finally:
    try:
        tt.write(R_DISABLE, 1)
    except Exception:
        pass
    tt.close()
