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
    constexpr uint32_t kFaultStallDetected = 1UL << 3;
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
    // 堵转是“控制器自己判定”的故障，不在 FaultMonitor 的由来位里，单独补一位。
    if (c.stall_detected_)
    {
        fault_flags |= kFaultStallDetected;
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

        // 【VLOOP 诊断】写 0x155=1 才打一次（打完自动清零）。默认不打。
        // 【绝对不能在运动/角度模式运行中打印】这三行浮点 ≈ 270 字节 @115200 ≈ 23ms，
        // 而 syncProtocolTelemetry() 是在 20kHz 控制任务里调用的 —— 打印一次就会
        // 整整丢掉 ~460 个控制拍，脉冲出口直接停摆，电机顿一下。
        // 所以：请求先挂起，等运动停下来再打；运动中写 0x155 不会立刻输出。
        if (c.vloop_debug_once_ && !c.motion_running_ && !c.closed_loop_angle_mode_enabled_)
        {
            c.vloop_debug_once_ = false;
            {
                const double pos_kp_web = static_cast<double>(c.base_position_kp_);
                const double pos_ki_web = static_cast<double>(c.base_position_ki_);
                const double pos_kd_web = static_cast<double>(c.base_position_kd_);
                const double vel_kp_web = static_cast<double>(c.base_velocity_kp_);
                const double vel_ki_web = static_cast<double>(c.base_velocity_ki_);
                const double vel_kd_web = static_cast<double>(c.base_velocity_kd_);
                const double pos_scale = (c.base_position_kp_ > 0.001f)
                    ? static_cast<double>(c.position_pid_.kp) / c.base_position_kp_ : 0.0;
                const double vel_scale = (c.base_velocity_kp_ > 0.001f)
                    ? static_cast<double>(c.velocity_pid_.kp) / c.base_velocity_kp_ : 0.0;
                printf("[VLOOP] mode ang=%u comp=%u | err=%+.3fdeg ref=%+.4f meas=%+.4f verr=%+.4f\r\n",
                       c.closed_loop_angle_mode_enabled_ ? 1U : 0U,
                       c.closed_loop_compensation_enabled_ ? 1U : 0U,
                       static_cast<double>(c.vloop_.err_deg),
                       static_cast<double>(c.vloop_.ref_rps),
                       static_cast<double>(c.vloop_.meas_rps),
                       static_cast<double>(c.vloop_.vel_err_rps));
                printf("[VLOOP] P=%+.4f I=%+.4f D=%+.4f posOut=%+.4f out=%+.4frps ff=%+.4frps step=%+.0f/s\r\n",
                       static_cast<double>(c.vloop_.vel_p),
                       static_cast<double>(c.vloop_.vel_i),
                       static_cast<double>(c.vloop_.vel_d),
                       static_cast<double>(c.vloop_.pos_out_rps),
                       static_cast<double>(c.vloop_.out_rps),
                       static_cast<double>(c.vloop_.prof_vel_rps),
                       static_cast<double>(c.vloop_.step_speed_s));
                printf("[VLOOP] gains web pos %.3f/%.3f/%.3f vel %.3f/%.3f/%.3f live pos %.3f/%.3f/%.3f vel %.3f/%.3f/%.3f kscale %.3f/%.3f p_scale %.2f/%.2f\r\n",
                       pos_kp_web, pos_ki_web, pos_kd_web, vel_kp_web, vel_ki_web, vel_kd_web,
                       static_cast<double>(c.position_pid_.kp),
                       static_cast<double>(c.position_pid_.ki),
                       static_cast<double>(c.position_pid_.kd),
                       static_cast<double>(c.velocity_pid_.kp),
                       static_cast<double>(c.velocity_pid_.ki),
                       static_cast<double>(c.velocity_pid_.kd),
                       pos_scale, vel_scale,
                       1.0,   // 角度模式 p_scale：串级改造后已删除，永远是 1.00
                       1.0);  // 规划运动补偿路径 p_scale：本来就没有缩
            }
        }
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
        // 用的是 20ms 平滑的显示值：20kHz 原始估计在 25Hz 采样下全是毛刺，
        // 会让人把"测速噪声"当成"电机在抖"。控制环内部仍用 3ms 的那个。
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.encoder_speed_display_rpm_)));
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
        // 实时滞后角（mdeg，已带方向）：网页用于显示和「误差不计滞后角」。
        // motion_lag_angle_deg_ 在控制器里已经按方向取过符号了：
        //   正向运动 -> -L（和 follow_error 同号）
        //   反向运动 -> +L
        // 这里直接发，不能再取反（否则反向运动时符号会错）。
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_LAG_ANGLE_MDEG,
                                        static_cast<uint32_t>(static_cast<int32_t>(c.motion_lag_angle_deg_ * 1000.0f)));
    }
}
