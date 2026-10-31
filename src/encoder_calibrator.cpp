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
