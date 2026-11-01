#include "parameter_router.h"

#include "closed_loop_controller.h"
#include "param_store.h"

#include <cstdio>

// 说明：下面这些分支里的成员访问都显式带上 owner_. 前缀。
// 这不是啰嗦 —— 这张表里既有“只改一个标量”的简单项，也有“改参数顺手清 PID 积分”
// 和“直接触发一次运动”的复合动作，去掉前缀以后很难一眼看出哪些是控制器状态、
// 哪些只是本地的临时量。

namespace
{
    // PID 增益上限（网页口径，即“实际增益 × 1000”的口径下直接填的数）。
    //
    // 【上限曾一度设成 30/3，是个错误】用户原来用的 60/20/60 会被静默钳成 30/3/30，
    // 于是加 D 想压震荡却永远加不上去（被钳在 30），表现为“加了 D 反而更震荡”。
    // 现在恢复 30000/3000（即实际增益 30 / 3？不：这里除 1000 后就是实际增益，
    // 所以上限实际是 30000 / 3000），配合 param_store 的 uint32_t 字段不会溢出。
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
    if (reg == TMC2209_EXT_PARAM_SAVE_PARAMS)
    {
        // 【写入会 stall Flash 总线几毫秒】运动中绝对不能做，否则会丢脉冲。
        if (owner_.motion_running_ || owner_.closed_loop_angle_mode_enabled_)
        {
            printf("[PARAM] save rejected: motion is running\r\n");
            owner_.param_save_ok_ = false;
            return true;
        }
        if (value == 0U)
        {
            return true;   // 写 0 不做任何事
        }

        PersistedMotorParams params = {};
        // PID 用上位机口径（×1000 的整数），与 param_store.h 注释一致
        params.position_kp = static_cast<uint32_t>(owner_.position_pid_.kp * 1000.0f + 0.5f);
        params.position_ki = static_cast<uint32_t>(owner_.position_pid_.ki * 1000.0f + 0.5f);
        params.position_kd = static_cast<uint32_t>(owner_.position_pid_.kd * 1000.0f + 0.5f);
        params.velocity_kp = static_cast<uint32_t>(owner_.velocity_pid_.kp * 1000.0f + 0.5f);
        params.velocity_ki = static_cast<uint32_t>(owner_.velocity_pid_.ki * 1000.0f + 0.5f);
        params.velocity_kd = static_cast<uint32_t>(owner_.velocity_pid_.kd * 1000.0f + 0.5f);
        // 角度限速/加速度不存 Flash：它们属于“本次运行设定”，每次按现场需要在界面上调，
        // 存进 Flash 反而会让一个旧值在重启后静默生效。结构体字段保留（置 0）用于兼容。
        params.angle_max_rpm = 0U;
        params.angle_accel_rpm_s = 0U;

        owner_.param_save_ok_ = param_store_save(&params);
        // 日志按“网页上的值”打印：存储里放大 1000 倍只为保留小数，
        // 直接打 60000 会让人误以为有个倍率在捣鬼。
        printf("[PARAM] save %s (pid %.3f/%.3f/%.3f %.3f/%.3f/%.3f)\r\n",
               owner_.param_save_ok_ ? "ok" : "FAILED",
               params.position_kp / 1000.0f,
               params.position_ki / 1000.0f,
               params.position_kd / 1000.0f,
               params.velocity_kp / 1000.0f,
               params.velocity_ki / 1000.0f,
               params.velocity_kd / 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FF_RATIO)
    {
        // 负载角前馈信任度：网页传百分数 0~100，内部存 0.0~1.0。
        // 写 0 = 完全关闭前馈（退回无前馈行为）；写 100 = 完全相信表。
        const uint32_t percent = value > 100U ? 100U : value;
        owner_.load_angle_feedforward_ratio_ = static_cast<float>(percent) / 100.0f;
        return owner_.protocol_ != nullptr && owner_.protocol_->writeRegister(reg, percent);
    }
    // ===== 校准期间：所有“闭环 / 运动命令”入口由校准器接管 =====
    // 磁编码器校准必须纯开环跑：每一段的“真角”完全由脉冲数定义。如果这时候还允许
    // 闭环补偿（0x107）、角度模式（0x128）、外部脉冲跟随（0x149）或主机的运动命令
    //（0x11F=1）进来，PID 就会往校准段里掺额外脉冲 —— emitSteps 实际发的是
    // 补偿累加器的步数，而“这段走完了”看的是原始累加器的记账，两者一旦不等，
    // 整圈进给就会被抬高（实测 +3.0%，约 1.5 微步/段）。
    // 校准器自己的 startMotion()/setTargetAngleDeg() 是直接 C++ 调用、不走寄存器入口，
    // 所以“归零用角度模式 + 扫段用开环规划运动”的结构不受影响。
    // 0x11F=0（停下）仍然放行：那是急停，不该被挡；校准器会在下一段发现起不来时
    // 自己 finish(false) 干净中止。
    if (!owner_.calibrator_.isIdle() &&
        (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE ||
         reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG ||
         reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE ||
         (reg == TMC2209_EXT_PARAM_MOTION_COMMAND && value != 0U)))
    {
        printf("[CALIBRATION] write rejected while calibrating (open-loop only): reg=0x%03lX value=%lu\r\n",
               static_cast<unsigned long>(reg), static_cast<unsigned long>(value));
        return true;   // 照常回 ACK，但不让控制结构在校准中途被改掉
    }
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
        owner_.external_pulse_follower_.setMaxStepRate(
            value > ExternalPulseFollower::kMaxStepRateLimit
                ? ExternalPulseFollower::kMaxStepRateLimit
                : value);
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
        // 显式使能：清除安全锁，允许启动
        owner_.user_disabled_ = false;
        owner_.stopMotion();
        if (owner_.driver_ != nullptr)
        {
            owner_.driver_->setEnable(true);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_DISABLE)
    {
        // 【安全锁】记住“用户主动失能”：之后任何 0x11F=1 / setTargetAngleDeg 都会拒绝，
        // 必须重新收到 MOTOR_ENABLE 才能再动。
        // 否则自动测试循环里的下一条 0x11F=1 会通过 startMotion 的 setEnable(true)
        // 把电流重新打开——表现为“已失能但电机自己转起来”。
        owner_.user_disabled_ = true;
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
    if (reg == TMC2209_EXT_PARAM_SAVE_PARAMS)
    {
        *value = owner_.param_save_ok_ ? 1U : 0U;
        return true;
    }
    // 【PID 回读必须实时取值】原来 PID 六项走的是 custom_parameters_ 缓存，
    // 而 init() 从 Flash 恢复时只改了 position_pid_ / velocity_pid_，没同步那份缓存，
    // 于是重启后网页读到的仍是旧值（0 或编译期默认），看起来“写入成功但重连没读出来”。
    if (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
        reg == TMC2209_EXT_PARAM_POSITION_KD || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
        reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        const PidController& pid = (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
                                    reg == TMC2209_EXT_PARAM_POSITION_KD)
            ? owner_.position_pid_ : owner_.velocity_pid_;
        const float gain = (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_VELOCITY_KP) ? pid.kp
                         : (reg == TMC2209_EXT_PARAM_POSITION_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KI) ? pid.ki
                         : pid.kd;
        *value = static_cast<uint32_t>(gain * 1000.0f + 0.5f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FF_RATIO)
    {
        // 回读百分数（0~100），与写入口径一致
        *value = static_cast<uint32_t>(owner_.load_angle_feedforward_ratio_ * 100.0f + 0.5f);
        return true;
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
        *value = owner_.external_pulse_follower_.isEnabled() ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        *value = owner_.external_pulse_follower_.maxStepRate();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_pulse_follower_.cmdSteps());
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_pulse_follower_.emittedSteps());
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG)
    {
        *value = owner_.external_pulse_follower_.backlogSteps();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED)
    {
        *value = owner_.external_pulse_follower_.droppedSteps();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS)
    {
        *value = owner_.external_pulse_follower_.windowReversals();
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
