#include "telemetry_reporter.h"

#include "closed_loop_controller.h"
#include "external_pulse_follower.h"

namespace
{
    // 故障位与 closed_loop_controller.h 里的 enum FaultFlags 保持一致。
    // 不直接引用那个 enum 是因为它在控制器头文件里，而这个 TU 只 include
    // 了 closed_loop_controller.h 的前置部分；用局部常量更稳，值改动时两处一起改。
    constexpr uint32_t kFaultEncoderReadFailed = 1UL << 0;
    constexpr uint32_t kFaultMagneticFieldAlarm = 1UL << 1;
    constexpr uint32_t kFaultOutputStopped = 1UL << 2;
}

TelemetryReporter::TelemetryReporter(ClosedLoopController& owner) : owner_(owner)
{
}

void TelemetryReporter::report(TelemetryScheduler::Slot slot)
{
    ClosedLoopController& c = owner_;

    if (c.protocol_ == nullptr || slot == TelemetryScheduler::Slot::kNone)
    {
        return;
    }

    uint32_t fault_flags = 0U;
    if (c.fault_monitor_.encoderFaultActive())
    {
        fault_flags |= kFaultEncoderReadFailed;
    }
    if (c.fault_monitor_.magneticFaultActive())
    {
        fault_flags |= kFaultMagneticFieldAlarm;
    }
    if (c.fault_monitor_.outputStopped())
    {
        fault_flags |= kFaultOutputStopped;
    }

    if (slot == TelemetryScheduler::Slot::kSlow)
    {
        // 低速档（约 1Hz）：这里是唯一真正改编码器滤波窗口的地方。
        // 滤波器的环形缓冲/累加和正在采样中断里被改，只允许通过 requestFilterWindow
        // 登记请求，由中断自己重配；这里把控住频率，就不会让跟随误差被窗口切换抖出台阶。
        if (c.encoder_ != nullptr && c.encoder_filter_window_ != c.encoder_filter_window_active_)
        {
            c.encoder_->requestFilterWindow(c.encoder_filter_window_);
            c.encoder_filter_window_active_ = c.encoder_filter_window_;
        }
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW, c.encoder_filter_window_active_);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_FAULT_STATUS, fault_flags);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_FAULT,
                                        c.fault_monitor_.encoderFaultActive() ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_OUTPUT_STOP,
                                        c.fault_monitor_.outputStopped() ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_POS_LOOP_HZ, c.loop_stats_.positionLoopHz());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_VEL_LOOP_HZ, c.loop_stats_.velocityLoopHz());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_CUR_LOOP_HZ, c.loop_stats_.currentLoopHz());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_LAST_FAULT, fault_flags);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_START_RPM, static_cast<uint32_t>(c.motion_start_rpm_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAX_RPM, static_cast<uint32_t>(c.motion_max_rpm_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACCEL_RPM_S, static_cast<uint32_t>(c.motion_accel_rpm_s_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_MAX_RPM, static_cast<uint32_t>(c.angle_max_rpm_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S, static_cast<uint32_t>(c.angle_accel_rpm_s_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_PULSE_COUNT, c.motion_pulse_count_);
        // 【注意】SPEED_RPM 原来在这里（kSlow ≈ 1Hz），但波形图按 25Hz 采样：
        // 1 秒内 25 个采样点读到的都是同一个值，转速曲线看起来就是“纹丝不动”。
        // 已移到 kMedium（≈ 100Hz）档。
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS, c.step_pulse_width_ns_);
    }
    else if (slot == TelemetryScheduler::Slot::kMedium)
    {
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_FAULT,
                                        c.fault_monitor_.magneticFaultActive() ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_A,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.phase_a_current_a_ * 1000.0f)));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_B,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.phase_b_current_a_ * 1000.0f)));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_POSITION, static_cast<uint32_t>(c.target_step_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACTUAL_POSITION, static_cast<uint32_t>(c.actual_step_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_FOLLOW_ERROR, static_cast<uint32_t>(c.follow_error_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_RAW, c.encoder_filtered_angle_);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.getEncoderAngleMilliDegrees() * 1000.0f)));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_HIGH, c.magnetic_field_high_ ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_LOW, c.magnetic_field_low_ ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_MODE, static_cast<uint32_t>(c.motion_mode_));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_COMMAND, c.motion_running_ ? 1U : 0U);
        // 实时转速：波形图按 25Hz 采样，必须和误差同档（kMedium ≈ 100Hz）才有意义。
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.encoder_speed_rpm_)));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_ANGLE_DEG,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.target_angle_deg_ * 1000.0f)));
        // 外部脉冲跟随遥测：命令/已发/积压/丢步（100Hz 足够上位机画图）
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE,
                                        c.external_pulse_follower_.isEnabled() ? 1U : 0U);
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S,
                                        c.external_pulse_follower_.maxStepRate());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS,
                                        static_cast<uint32_t>(c.external_pulse_follower_.cmdSteps()));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS,
                                        static_cast<uint32_t>(c.external_pulse_follower_.emittedSteps()));
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG,
                                        c.external_pulse_follower_.backlogSteps());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED,
                                        c.external_pulse_follower_.droppedSteps());
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS,
                                        c.external_pulse_follower_.windowReversals());
        // 实时负载角（mdeg，有符号）：网页用于显示和「误差不计负载角」。
        //
        // 符号约定（很重要）：motion_load_angle_deg_ 内部始终是**正值**（滞后量大小），
        // 因为补偿段要用 follow_error + load_angle 归零，必须同向相加。
        // 但对外显示要和 follow_error 同号才直观：正向运动时电机滞后 \u2192 follow_error 是负值，
        // 负载角也应该是负的。所以按方向取反后再发：
        //   encoder_speed_rpm >= 0  ->  发 -L
        //   encoder_speed_rpm <  0  ->  发 +L
        // 网页那边扣除时用 followError - loadAngle（因为两者同号）。
        const float signed_load_angle_deg = (c.encoder_speed_rpm_ >= 0.0f)
            ? -c.motion_load_angle_deg_
            : c.motion_load_angle_deg_;
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_LOAD_ANGLE_MDEG,
                                        static_cast<uint32_t>(static_cast<int32_t>(signed_load_angle_deg * 1000.0f)));
    }
}
