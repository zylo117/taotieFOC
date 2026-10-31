#ifndef ENCODER_CALIBRATOR_H
#define ENCODER_CALIBRATOR_H

#include <stdint.h>

class ClosedLoopController;

// ===========================================================================
// 磁编码器双程校准（正转存储 → 反转平均 → 写表）
//
// 从 ClosedLoopController 拆出来的第一块（controller 原本 1900+ 行）。
// 和 MotionPlanner 一样用 owner 引用访问控制器状态，不复制任何共享数据。
//
// 流程：
//   HOMING   角度模式把转子拉到 0 度，等编码器读数稳定 100 拍
//   MOVING   每段走 steps_per_round/CALIBRATION_TABLE_SIZE 步
//   SAMPLING 停 50ms 等负载角稳定，再采 202 个样本去极值平均
//   正向趟只存不写表；反过来反向趟把两趟平均后写入
//
// 【为什么要正反转平均】负载角让转子正转时落后、反转时超前，
// 同一个机械位置两趟的编码器读数会一低一高，平均后正好把这块系统偏差
// 抵消（实测 1200RPM 时约 1.5 度）。代价是 2KB 临时数组（1024 点 × uint16）
// 与一倍时间；边界点 1023 没有反向采样值，直接用正向值写表。
// ===========================================================================
class EncoderCalibrator
{
public:
    explicit EncoderCalibrator(ClosedLoopController& owner);

    // 启动校准（对应原 calibrateEncoder）。
    // 会先关掉外部跟随、保证 angle_* 限幅非零、清空旧表、切到角度模式回零。
    void start();

    // 每拍推进（对应原 updateEncoderCalibration），在 TMR4 控制循环里调用。
    // raw 是最新的编码器原始值（未查表），0xFFFF 表示本次读数无效。
    void update(uint16_t raw);

    // 当前是否空闲（可接受新的校准请求）。真正生效（阶段 5 接线）后可替代
    // 各处直接读 controller->calibration_stage_ 的判断。
    bool isIdle() const;

    // 当前阶段，供遥测/调试查询。返回值与 ClosedLoopController::EncoderCalibrationStage 一致。
    uint8_t stage() const;

    // 由 MotionPlanner 在一小段校准位移走完时调用：切到 SAMPLING 并重置采样计数。
    // 采样计数已搬进本类，planner 不能再直接写，否则两个模块各持一份。
    void beginSampling();

private:
    void finish(bool save_table);
    bool startMove(uint16_t index);

    ClosedLoopController& owner_;

    // 【阶段状态不复制在这里】唯一真源是 owner_.calibration_stage_：
    //   motion_planner.cpp 的 updateTrapezoid 会在一小段步数走完时把它从 MOVING 改成 SAMPLING，
    //   readCalibrationTablePair / readCalibrationTableChecksum 也用"是否 IDLE"做门禁。
    // 本类再存一份 stage_ 就会有两份状态，接线后必然出现"一边改了另一边不知道"。
    // 因此本类只持有校准专用、外部不会碰的状态（下面这些）。
    volatile uint16_t index_ = 0U;

    // 正向趟的采样值（反向趟取平均时用）。2KB：1024 点 x uint16。
    static constexpr uint16_t kPoints = 1024U;
    uint16_t forward_raw_[kPoints] = {};
    bool backward_pass_ = false;

    volatile uint16_t sample_count_ = 0U;
    volatile uint16_t sample_attempts_ = 0U;
    volatile int32_t sample_sum_ = 0;
    volatile int32_t sample_min_ = 0;
    volatile int32_t sample_max_ = 0;
    volatile uint16_t sample_anchor_ = 0U;

    // HOMING 阶段：等转子稳定；SAMPLING 阶段复用 wait_ticks 做“采样前停 50ms”
    volatile uint32_t home_wait_ticks_ = 0U;
    volatile uint16_t home_stable_count_ = 0U;
    volatile float home_previous_angle_ = 0.0f;

    // 校准前的运动参数备份，结束时恢复
    float saved_start_rpm_ = 0.0f;
    float saved_max_rpm_ = 0.0f;
    float saved_accel_rpm_s_ = 0.0f;
    uint32_t saved_pulse_count_ = 0U;
    uint8_t saved_motion_mode_ = 0U;
    uint64_t saved_pulse_width_ns_ = 0U;
    bool saved_compensation_enabled_ = false;
};

#endif // ENCODER_CALIBRATOR_H
