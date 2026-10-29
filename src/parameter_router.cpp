#include "parameter_router.h"

#include "closed_loop_controller.h"

// 说明：下面这些分支里的成员访问都显式带上 owner_. 前缀。
// 这不是啰嗦 —— 这张表里既有“只改一个标量”的简单项，也有“改参数顺手清 PID 积分”
// 和“直接触发一次运动”的复合动作，去掉前缀以后很难一眼看出哪些是控制器状态、
// 哪些只是本地的临时量。

namespace
{
    // PID 增益上限：比例/微分 30、积分 3（上位机传的是放大 1000 倍的整数）。
    float clampPidGain(uint16_t reg, uint32_t raw_value)
    {
        const float max_gain = (reg == TMC2209_EXT_PARAM_POSITION_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KI)
            ? 3000.0f
            : 30000.0f;
        const float gain = static_cast<float>(raw_value) / 1000.0f;
        if (gain < 0.0f)
        {
            return 0.0f;
        }
        return gain > max_gain ? max_gain : gain;
    }
}

ParameterRouter::ParameterRouter(ClosedLoopController& owner) : owner_(owner)
{
}

bool ParameterRouter::write(uint16_t reg, uint32_t value)
{
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        owner_.closed_loop_compensation_enabled_ = value != 0U;
        owner_.position_pid_.resetIntegral();
        owner_.position_pid_.resetDeriv();
        owner_.velocity_pid_.resetIntegral();
        owner_.velocity_pid_.resetDeriv();
        return owner_.protocol_ != nullptr && owner_.protocol_->writeRegister(reg, value);
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        owner_.setEncoderZero(static_cast<uint16_t>(value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_UPLOAD_COUNT)
    {
        owner_.load_angle_fit_saved_ = false;
        return owner_.load_angle_.beginUpload(static_cast<uint16_t>(value));
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_INDEX)
    {
        owner_.load_angle_point_index_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_SPEED_RPM)
    {
        owner_.load_angle_point_speed_rpm_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_MDEG)
    {
        owner_.load_angle_point_mdeg_ = static_cast<int32_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COMMIT)
    {
        return value != 0U && owner_.load_angle_.setPoint(owner_.load_angle_point_index_,
                                                         owner_.load_angle_point_speed_rpm_,
                                                         owner_.load_angle_point_mdeg_);
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT)
    {
        if (value == 0U)
        {
            return true;
        }
        owner_.load_angle_fit_saved_ = owner_.load_angle_.fit() && owner_.load_angle_.saveToFlash();
        return owner_.load_angle_fit_saved_;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_SPEED_RPM)
    {
        owner_.load_angle_query_speed_rpm_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        owner_.motion_start_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        owner_.motion_max_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        owner_.motion_accel_rpm_s_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        owner_.angle_max_rpm_ = static_cast<float>(value) < 1.0f ? 1.0f
            : (static_cast<float>(value) > 12000.0f ? 12000.0f : static_cast<float>(value));
        return owner_.protocol_ != nullptr &&
               owner_.protocol_->setCustomParameter(reg, static_cast<uint32_t>(owner_.angle_max_rpm_));
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        owner_.angle_accel_rpm_s_ = static_cast<float>(value) < 1.0f ? 1.0f
            : (static_cast<float>(value) > 12000.0f ? 12000.0f : static_cast<float>(value));
        return owner_.protocol_ != nullptr &&
               owner_.protocol_->setCustomParameter(reg, static_cast<uint32_t>(owner_.angle_accel_rpm_s_));
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_CONTROL)
    {
        if (value != 0U)
        {
            owner_.pid_auto_tuner_.beginTrial();
        }
        else
        {
            owner_.pid_auto_tuner_.finishTrial();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        owner_.motion_pulse_count_ = value;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        owner_.motion_mode_ = static_cast<ClosedLoopController::MotionMode>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        if (value != 0U)
        {
            owner_.startMotion();
        }
        else
        {
            owner_.closed_loop_angle_mode_enabled_ = false;
            // 跟随时 stopMotion() 停不掉“跟随”本身：external_follow_enabled_ 还开着，
            // 下一拍就会接着发。所以停运动必须把跟随一起关掉。
            owner_.enableExternalPulseFollow(false);
            owner_.stopMotion();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATE_ENCODER)
    {
        owner_.calibrateEncoder();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        // 只登记期望值，真正的重配交给遥测慢档（syncProtocolTelemetry），
        // 否则会在运动中途改滤波窗口，把跟随误差抖出台阶。
        owner_.encoder_filter_window_ = value > EncoderFilterConfig::kMaxWindow
            ? EncoderFilterConfig::kMaxWindow
            : static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        owner_.enableExternalPulseFollow(value != 0U);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        // 0 = 自动（按 angle_max_rpm_ 换算）；上限按 2.56Mstep/s（128 步/窗）夹住
        owner_.setExternalPulseMaxStepRate(value > 2560000UL ? 2560000UL : value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_RESET)
    {
        if (value != 0U)
        {
            owner_.resetExternalPulseFollowCounters();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        const uint64_t clamped_ns = value < ClosedLoopController::kMinStepPulseNs
            ? ClosedLoopController::kMinStepPulseNs
            : (value > ClosedLoopController::kMaxStepPulseNs
                   ? ClosedLoopController::kMaxStepPulseNs
                   : static_cast<uint64_t>(value));
        owner_.step_pulse_width_ns_ = clamped_ns;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        owner_.calibration_lookup_raw_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        owner_.calibration_lookup_corrected_value_ = owner_.encoder_ == nullptr
            ? 0U
            : owner_.encoder_->getCorrectedRaw(owner_.calibration_lookup_raw_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        owner_.calibration_lookup_corrected_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        owner_.calibration_lookup_raw_value_ = owner_.encoder_ == nullptr
            ? 0U
            : owner_.lookupOriginalRaw(owner_.calibration_lookup_corrected_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        owner_.setTargetAngleDeg(static_cast<float>(value) / 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
        reg == TMC2209_EXT_PARAM_POSITION_KD || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
        reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        // 上位机传的是放大 1000 倍的整数，保留三位小数；上限见 clampPidGain。
        const float gain = clampPidGain(reg, value);
        if (reg == TMC2209_EXT_PARAM_POSITION_KP) owner_.base_position_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KI) owner_.base_position_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KD) owner_.base_position_kd_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KP) owner_.base_velocity_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KI) owner_.base_velocity_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KD) owner_.base_velocity_kd_ = gain;
        owner_.position_pid_.setGains(owner_.base_position_kp_, owner_.base_position_ki_, owner_.base_position_kd_);
        owner_.velocity_pid_.setGains(owner_.base_velocity_kp_, owner_.base_velocity_ki_, owner_.base_velocity_kd_);
        owner_.position_pid_.resetIntegral();
        owner_.position_pid_.resetDeriv();
        owner_.velocity_pid_.resetIntegral();
        owner_.velocity_pid_.resetDeriv();
        return owner_.protocol_ != nullptr && owner_.protocol_->setCustomParameter(reg, value);
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_ENABLE)
    {
        owner_.stopMotion();
        if (owner_.driver_ != nullptr)
        {
            owner_.driver_->setEnable(true);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_DISABLE)
    {
        owner_.closed_loop_angle_mode_enabled_ = false;
        owner_.stopMotion();
        if (owner_.driver_ != nullptr)
        {
            owner_.driver_->setEnable(false);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS)
    {
        const uint32_t micro_steps_per_round = owner_.getStepsPerRound();
        const uint32_t half_round_steps = micro_steps_per_round / 2U;

        // 设置运动参数，交给梯形规划引擎输出脉冲
        owner_.setMotionConfig(owner_.motion_start_rpm_, owner_.motion_max_rpm_, owner_.motion_accel_rpm_s_,
                               half_round_steps, ClosedLoopController::MOTION_MODE_POSITION_FORWARD);
        owner_.startMotion();

        owner_.last_en_state_ = 2U;
        return true;
    }
    if (owner_.protocol_ == nullptr)
    {
        return false;
    }
    return owner_.protocol_->writeRegister(reg, value);
}

bool ParameterRouter::read(uint16_t reg, uint32_t* value)
{
    if (value == nullptr)
    {
        return false;
    }
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        *value = owner_.closed_loop_compensation_enabled_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        *value = owner_.encoder_zero_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        *value = static_cast<uint32_t>(owner_.motion_start_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        *value = static_cast<uint32_t>(owner_.motion_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(owner_.motion_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        *value = static_cast<uint32_t>(owner_.angle_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(owner_.angle_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        *value = owner_.motion_pulse_count_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        *value = static_cast<uint32_t>(owner_.motion_mode_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        *value = owner_.motion_running_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SPEED_RPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.encoder_speed_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_DEG)  // 实际是跟随误差
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.motion_follow_error_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        *value = owner_.encoder_filter_window_active_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        *value = owner_.external_follow_enabled_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        *value = owner_.external_max_step_s_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_cmd_steps_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_emitted_steps_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG)
    {
        *value = owner_.external_backlog_steps_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED)
    {
        *value = owner_.external_dropped_steps_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS)
    {
        *value = owner_.external_window_reversals_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        *value = owner_.step_pulse_width_ns_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        *value = owner_.calibration_lookup_corrected_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        *value = owner_.calibration_lookup_raw_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.target_angle_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_INDEX)
    {
        *value = owner_.load_angle_point_index_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_SPEED_RPM)
    {
        uint16_t speed_rpm = owner_.load_angle_point_speed_rpm_;
        int32_t load_angle_mdeg = owner_.load_angle_point_mdeg_;
        owner_.load_angle_.getPoint(owner_.load_angle_point_index_, &speed_rpm, &load_angle_mdeg);
        *value = speed_rpm;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_MDEG)
    {
        uint16_t speed_rpm = owner_.load_angle_point_speed_rpm_;
        int32_t load_angle_mdeg = owner_.load_angle_point_mdeg_;
        owner_.load_angle_.getPoint(owner_.load_angle_point_index_, &speed_rpm, &load_angle_mdeg);
        *value = static_cast<uint32_t>(load_angle_mdeg);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_SPEED_RPM)
    {
        *value = owner_.load_angle_query_speed_rpm_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_MDEG)
    {
        *value = static_cast<uint32_t>(owner_.load_angle_.getLoadAngleMilliDegrees(owner_.load_angle_query_speed_rpm_));
        return owner_.load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COUNT)
    {
        *value = owner_.load_angle_.pointCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT)
    {
        *value = owner_.load_angle_fit_saved_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_SLOPE)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.load_angle_.powerCoefficient() * 1000000.0f));
        return owner_.load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_INTERCEPT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.load_angle_.interceptDeg() * 1000.0f));
        return owner_.load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_EXPONENT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.load_angle_.exponent() * 1000000.0f));
        return owner_.load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP)
    {
        *value = static_cast<uint32_t>(owner_.base_position_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KI)
    {
        *value = static_cast<uint32_t>(owner_.base_position_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KD)
    {
        *value = static_cast<uint32_t>(owner_.base_position_kd_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG)
    {
        *value = owner_.pid_auto_tuner_.scoreMilliDegrees();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT)
    {
        *value = owner_.pid_auto_tuner_.sampleCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS)
    {
        *value = owner_.motion_steps_emitted_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.motion_speed_rpm_ *
                                                             static_cast<float>(owner_.motion_direction_) * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT)
    {
        *value = owner_.cpu_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT)
    {
        *value = owner_.control_task_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KP)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KI)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_kd_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_REG_TSTEP)
    {
        *value = owner_.step_period_us_;
        return true;
    }
    if (reg == TMC2209_REG_MSCNT)
    {
        *value = owner_.encoder_filtered_angle_;
        return true;
    }
    if (reg == TMC2209_REG_DRV_STATUS)
    {
        *value = owner_.magnetic_field_high_ || owner_.magnetic_field_low_ ? (1U << 24U) : 0U;
        return true;
    }
    // 扩展表里没认识的地址，交给协议层的官方 TMC 寄存器分支。
    if (owner_.protocol_ == nullptr)
    {
        return false;
    }
    return owner_.protocol_->readRegister(reg, value);
}
