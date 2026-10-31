#include "encoder_calibrator.h"

#include "closed_loop_controller.h"

// ===========================================================================
// 磁编码器双程校准（拆分第 1 阶段：骨架）
//
// 这个文件目前只有构造与状态查询，四个主流程（start / update / finish /
// startMove）还在 closed_loop_controller.cpp 里跑。
//
// 拆分节奏（避免一次搬 400 行引出回归）：
//   阶段 1（本文件）：类壳 + 状态成员 + 访问器
//   阶段 2：搬 finish()（最短，依赖 saved_* 与 encoder_）
//   阶段 3：搬 startMove()（依赖 motion_* 与 driver_）
//   阶段 4：搬 update()（最长，四个状态机）
//   阶段 5：搬 start()，并把 controller 的 calibrateEncoder /
//           updateEncoderCalibration / finishEncoderCalibration /
//           startEncoderCalibrationMove 改成四行转发
//   阶段 6：删掉 controller 里 15 个 calibration_* 成员与 EncoderCalibrationStage 枚举
//
// 每阶段都可独立编译、独立刷机验证，出问题只回退一个提交。
// ===========================================================================

EncoderCalibrator::EncoderCalibrator(ClosedLoopController& owner) : owner_(owner)
{
}

bool EncoderCalibrator::isIdle() const
{
    return stage_ == STAGE_IDLE;
}

// 结束校准并恢复运动参数（对应原 finishEncoderCalibration）。
// 阶段 2：已搬入。因为 calibrateEncoder（写备份值）还在 controller 里，
// 这里直接读 owner_.calibration_saved_*，两边共用一份备份，不会出现两份状态不一致。
void EncoderCalibrator::finish(bool save_table)
{
    ClosedLoopController& c = owner_;

    stage_ = STAGE_SAVING;
    c.closed_loop_angle_mode_enabled_ = false;
    c.stopMotion();

    bool saved = false;
    if (save_table)
    {
        saved = c.encoder_ != nullptr && c.encoder_->saveCalibration();
    }

    // 恢复校准前备份的运动参数
    c.motion_start_rpm_ = c.calibration_saved_start_rpm_;
    c.motion_max_rpm_ = c.calibration_saved_max_rpm_;
    c.motion_accel_rpm_s_ = c.calibration_saved_accel_rpm_s_;
    c.motion_pulse_count_ = c.calibration_saved_pulse_count_;
    c.motion_mode_ = c.calibration_saved_motion_mode_;
    c.step_pulse_width_ns_ = c.calibration_saved_pulse_width_ns_;

    // 【不恢复补偿开关】校准是开环操作；恢复成校准前的 true 会造成
    // “界面已取消勾选、固件里补偿却开着”的不一致。由上位机下次启动显式下发 0x107。
    c.closed_loop_compensation_enabled_ = false;

    c.calibration_stage_ = ClosedLoopController::CALIBRATION_IDLE;
    printf("[CALIBRATION] finished saved=%u\r\n", saved ? 1U : 0U);
}

// 把转子移动到第 index 个校准点（对应原 startEncoderCalibrationMove）。
// 阶段 3：已搬入。每段只走 steps_per_round/CALIBRATION_TABLE_SIZE 步（64 细分下 12~13 步）。
bool EncoderCalibrator::startMove(uint16_t index)
{
    ClosedLoopController& c = owner_;

    // 【每 128 点才打一次进度】原来每段都打，带 %f 约 100 字节/段，
    // 1024 段就是 100KB，115200 下约 10 秒纯串口输出。
    if ((index % 128U) == 0U)
    {
        printf("[CALIBRATION] moving to index: %u from angle: %f\r\n", index, c.encoder_->readFilteredAngle());
    }

    // 用 controller 的 public 方法，避免依赖 closed_loop_controller.cpp 里的内部函数
    // getMicroStepsPerRound（它在那个 TU 里，头文件不暴露）。
    const uint32_t steps_per_round = c.getStepsPerRound();
    const uint64_t table_size = CALIBRATION_TABLE_SIZE;
    const uint32_t first_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t next_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index + 1U) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t steps = next_step - first_step;
    if (steps == 0U)
    {
        printf("[CALIBRATION] unsupported microstep resolution\r\n");
        finish(false);
        return false;
    }

    c.motion_start_rpm_ = 60.0f;
    c.motion_max_rpm_ = 60.0f;
    // 【加速度必须足够大】每段只走 12~13 步，而 startMotion 按梯形算减速点
    // decel_total_steps_ = vmax^2/(2a)：
    //   60 RPM/s    -> 6400 步 >> 13 步：第一拍就进 DECEL，一步不发
    //   60000 RPM/s -> 6 步：吃掉半程（实测整圈只走 200 度）
    // 拉到 600000 RPM/s -> decel = 0.64 取整 0 -> steps_to_decel_ = 0 -> 全程 CRUISE。
    c.motion_accel_rpm_s_ = 600000.0f;
    c.motion_pulse_count_ = steps;
    // 每段移动前清零等待计数器，让 SAMPLING 阶段先停 50ms 再采样
    home_wait_ticks_ = 0U;
    // 反向趟用 POSITION_REVERSE（startMotion 会据此把 motion_direction_ 设成 -1）
    c.motion_mode_ = backward_pass_
        ? ClosedLoopController::MOTION_MODE_POSITION_REVERSE
        : ClosedLoopController::MOTION_MODE_POSITION_FORWARD;
    c.step_pulse_width_ns_ = 200ULL;
    c.calibration_stage_ = ClosedLoopController::CALIBRATION_MOVING;
    c.motion_direction_ = backward_pass_ ? -1 : 1;
    c.startMotion();
    if (!c.motion_running_)
    {
        printf("[CALIBRATION] could not start move at index=%u\r\n", static_cast<unsigned>(index));
        finish(false);
        return false;
    }
    return true;
}
