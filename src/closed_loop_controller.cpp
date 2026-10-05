#include "closed_loop_controller.h"
#include "at32f403a_407_board.h"

#define STEP_EDGE_TIMEOUT_US    200U
#define STEP_PERIOD_US_DEFAULT  5000U
#define FULL_STEPS_PER_ROUND    200U  // 1.8度步进
#define DEFAULT_MICROSTEPS      64U
#define MIN_STEP_PULSE_NS       100ULL
#define DEFAULT_STEP_PULSE_NS   2000ULL
#define MAX_STEP_PULSE_NS       20000ULL
#define MAX_PID_OUTPUT          2000.0f
#define MAX_I_TERM              100.0f

namespace
{
    uint16_t clampMicrosteps(uint16_t value)
    {
        if (value < 2U)
        {
            return 2U;
        }
        if (value > 256U)
        {
            return 256U;
        }
        return value;
    }

    float clampCurrentA(float value)
    {
        if (value < 0.0f)
        {
            return 0.0f;
        }
        if (value > 3.0f)
        {
            return 3.0f;
        }
        return value;
    }

    uint64_t clampStepPulseWidthNs(uint64_t value)
    {
        if (value < MIN_STEP_PULSE_NS)
        {
            return MIN_STEP_PULSE_NS;
        }
        if (value > MAX_STEP_PULSE_NS)
        {
            return MAX_STEP_PULSE_NS;
        }
        return value;
    }

    // 每圈多少微步
    uint32_t getMicroStepsPerRound(const StepperDriver* driver)
    {
        uint16_t microsteps = DEFAULT_MICROSTEPS;
        if (driver != nullptr)
        {
            microsteps = driver->config().microsteps;
        }
        if (microsteps == 0U)
        {
            microsteps = DEFAULT_MICROSTEPS;
        }
        return FULL_STEPS_PER_ROUND * static_cast<uint32_t>(microsteps);
    }

    uint64_t pulseWidthNsToUs(uint64_t pulse_width_ns)
    {
        return (pulse_width_ns + 999ULL) / 1000ULL;
    }
}

#if USE_HARD_FLOAT_ACCELERATION
static inline float fast_abs(float value)
{
    return __builtin_fabsf(value);
}

static inline float fast_clamp(float value, float min_value, float max_value)
{
    return __builtin_fminf(__builtin_fmaxf(value, min_value), max_value);
}
#else
static inline float fast_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline float fast_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
#endif

namespace
{
    float normalize_signed_angle_error_deg(float error_deg)
    {
        while (error_deg > 180.0f)
        {
            error_deg -= 360.0f;
        }
        while (error_deg < -180.0f)
        {
            error_deg += 360.0f;
        }
        return error_deg;
    }

    float normalize_signed_delta_deg(float delta_deg)
    {
        while (delta_deg > 180.0f)
        {
            delta_deg -= 360.0f;
        }
        while (delta_deg < -180.0f)
        {
            delta_deg += 360.0f;
        }
        return delta_deg;
    }

    int16_t normalize_signed_delta_deg(int32_t delta_deg)
    {
        while (delta_deg > 32767)
        {
            delta_deg -= 65536;
        }
        while (delta_deg < -32768)
        {
            delta_deg += 65536;
        }
        return static_cast<int16_t>(delta_deg);
    }
}

PidController::PidController()
    : kp(1.0f), ki(0.04f), kd(0.02f), integral(0.0f), last_error(0.0f),
      max_integral(MAX_I_TERM), max_output(MAX_PID_OUTPUT)
{
}

void PidController::setGains(float kp_value, float ki_value, float kd_value)
{
    kp = kp_value;
    ki = ki_value;
    kd = kd_value;
}

// PID 的核心计算公式：
// output = Kp * error + Ki * integral(error) + Kd * d(error)/dt
// 其中：
// - P 项让系统对误差有直接修正能力
// - I 项用于消除稳态误差
// - D 项用于抑制超调和提高阻尼
// 但在电机闭环中，积分项和微分项不能无限大，否则会导致振荡，因此要做限幅。
float PidController::update(float error, float dt)
{
    float derivative = 0.0f;
    float output = 0.0f;

    if (dt > 0.0f)
    {
        // 误差变化率用于反映系统“趋势”，可帮助提前修正冲击和速度变化。
        derivative = (error - last_error) / dt;
    }

    // 科学意义：积分项是误差的累积，能消除长期偏差。
    // 但如果不限制，它很容易带来持续过冲，所以必须做区间裁剪。
    integral += error * dt;
    if (integral > max_integral)
    {
        integral = max_integral;
    }
    else if (integral < -max_integral)
    {
        integral = -max_integral;
    }

    // 总输出等于三个项加权和：位置和速度环都使用这一套结构。
    output = kp * error + ki * integral + kd * derivative;
    if (output > max_output)
    {
        output = max_output;
    }
    else if (output < -max_output)
    {
        output = -max_output;
    }

    // 更新微分历史值，保证下一周期以最新误差为基准。
    last_error = error;
    return output;
}

void PidController::resetIntegral()
{
    integral = 0.0f;
}

void PidController::resetDeriv()
{
    last_error = 0.0f;
}

ClosedLoopController::ClosedLoopController()
    : driver_(nullptr), encoder_(nullptr), protocol_(nullptr),
      base_position_kp_(60.0f), base_position_ki_(20.0f), base_position_kd_(60.0f),
      base_velocity_kp_(30.0f), base_velocity_ki_(8.0f), base_velocity_kd_(20.0f),
      adaptive_pid_enabled_(false), adaptive_config_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      target_step_(0), actual_step_(0), last_actual_step_(0), command_step_(0),
      follow_error_(0.0f), measured_velocity_rps_(0.0f), target_velocity_rps_(0.0f),
    motion_start_rpm_(60.0f), motion_max_rpm_(1200.0f), motion_accel_rpm_s_(6000.0f),
    angle_max_rpm_(1200.0f), angle_accel_rpm_s_(6000.0f),
      motion_pulse_count_(0U), motion_mode_(MOTION_MODE_POSITION_FORWARD),
    closed_loop_compensation_enabled_(false), motion_running_(false), motion_paused_(false),
    motion_speed_rpm_(0.0f), encoder_speed_rpm_(0.0f),
      motion_position_deg_(0.0f),
    motion_follow_error_deg_(0.0f), motion_commanded_travel_deg_(0.0f), motion_encoder_travel_raw_(0),
    motion_encoder_previous_raw_(0), motion_encoder_reference_valid_(false),
    motion_last_step_time_us_(0U), motion_last_ramp_time_us_(0U), motion_steps_emitted_(0U),
    motion_leg_pulse_count_(0U), motion_step_accumulator_(0.0f), motion_direction_(1),
    motion_leg_reversed_(false), motion_direction_change_pending_(false),
      motion_step_high_(false), step_pulse_width_ns_(DEFAULT_STEP_PULSE_NS),
      step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), encoder_filtered_angle_(0.0F), encoder_filtered_raw_(0),
      magnetic_field_high_(false), magnetic_field_low_(false), last_process_time_us_(0U),
      last_step_state_(0U), last_dir_state_(0U), last_en_state_(0U),
      stop_on_encoder_fault_(true), stop_on_magnetic_fault_(true), encoder_fault_active_(false),
      magnetic_fault_active_(false), output_stopped_(false), phase_a_current_a_(0.0f),
    phase_b_current_a_(0.0f), cpu_usage_centi_percent_(0U), control_task_usage_centi_percent_(0U),
    calibration_lookup_raw_value_(0U), calibration_lookup_corrected_value_(0U),
    loop_stats_enabled_(false), last_position_tick_ns_(0ULL),
      last_velocity_tick_ns_(0ULL), last_current_tick_ns_(0ULL), position_loop_hz_(0U),
      velocity_loop_hz_(0U), current_loop_hz_(0U), position_samples_(0U), velocity_samples_(0U),
    current_samples_(0U), calibration_stage_(CALIBRATION_IDLE), calibration_index_(0U),
    calibration_sample_count_(0U), calibration_sample_attempts_(0U),
    calibration_sample_min_(0), calibration_sample_max_(0), calibration_sample_anchor_(0U),
    calibration_sample_sum_(0), calibration_home_stable_count_(0U), calibration_home_wait_ticks_(0U),
    calibration_home_previous_angle_(0.0f), calibration_saved_start_rpm_(0.0f),
    calibration_saved_max_rpm_(0.0f), calibration_saved_accel_rpm_s_(0.0f),
    calibration_saved_pulse_count_(0U), calibration_saved_pulse_width_ns_(0U),
    calibration_saved_motion_mode_(MOTION_MODE_POSITION_FORWARD),
    calibration_saved_compensation_enabled_(false)
{
    position_pid_.setGains(base_position_kp_, base_position_ki_, base_position_kd_);
    velocity_pid_.setGains(base_velocity_kp_, base_velocity_ki_, base_velocity_kd_);
}

void ClosedLoopController::init(StepperDriver* driver, AngleEncoder* encoder)
{
    driver_ = driver;
    encoder_ = encoder;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_init_type input_gpio;
    gpio_default_para_init(&input_gpio);
    input_gpio.gpio_mode = GPIO_MODE_INPUT;
    input_gpio.gpio_pull = GPIO_PULL_NONE;
    input_gpio.gpio_pins = GPIO_PINS_15;
    gpio_init(GPIOA, &input_gpio);
    input_gpio.gpio_pins = GPIO_PINS_3;
    gpio_init(GPIOB, &input_gpio);
    input_gpio.gpio_pins = EN_OUT_PIN;
    gpio_init(EN_OUTPUT_PORT, &input_gpio);

    position_pid_.setGains(base_position_kp_, base_position_ki_, base_position_kd_);
    velocity_pid_.setGains(base_velocity_kp_, base_velocity_ki_, base_velocity_kd_);

    if (protocol_ != nullptr && driver_ != nullptr)
    {
        protocol_->attachDriver(driver_);
        protocol_->init();
    }

    if (encoder_ != nullptr)
    {
        encoder_->init();
        encoder_zero_ = encoder_->readFilteredAngle();
    }

    if (driver_ != nullptr)
    {
        driver_->init();
        driver_->setEnable(false);
        driver_->setDirection(false);
        driver_->setStepState(false);
    }

    last_process_time_us_ = 0U;
}

void ClosedLoopController::setProtocol(ClosedLoopDriverProtocol* protocol)
{
    protocol_ = protocol;
    if (protocol_ != nullptr && driver_ != nullptr)
    {
        protocol_->attachDriver(driver_);
        protocol_->init();
    }
}

void ClosedLoopController::setFaultPolicy(bool stop_on_encoder_fault, bool stop_on_magnetic_fault)
{
    stop_on_encoder_fault_ = stop_on_encoder_fault;
    stop_on_magnetic_fault_ = stop_on_magnetic_fault;
}

void ClosedLoopController::reportEncoderFault(bool active)
{
    encoder_fault_active_ = active;
    if (active && stop_on_encoder_fault_ && driver_ != nullptr)
    {
        driver_->setEnable(false);
        output_stopped_ = true;
    }
    else if (!active && !magnetic_fault_active_)
    {
        output_stopped_ = false;
    }
}

void ClosedLoopController::reportMagneticFieldAlarm(bool active)
{
    magnetic_fault_active_ = active;
    if (active && stop_on_magnetic_fault_ && driver_ != nullptr)
    {
        driver_->setEnable(false);
        output_stopped_ = true;
    }
    else if (!active && !encoder_fault_active_)
    {
        output_stopped_ = false;
    }
}

void ClosedLoopController::setPhaseCurrentTelemetry(float phase_a_a, float phase_b_a)
{
    phase_a_current_a_ = phase_a_a;
    phase_b_current_a_ = phase_b_a;
}

void ClosedLoopController::setCpuUsageTelemetry(uint16_t cpu_usage_centi_percent,
                                                 uint16_t control_task_usage_centi_percent)
{
    cpu_usage_centi_percent_ = cpu_usage_centi_percent;
    control_task_usage_centi_percent_ = control_task_usage_centi_percent;
}

void ClosedLoopController::syncProtocolTelemetry()
{
    if (protocol_ == nullptr)
    {
        return;
    }

    uint32_t fault_flags = 0U;
    if (encoder_fault_active_)
    {
        fault_flags |= FAULT_ENCODER_READ_FAILED;
    }
    if (magnetic_fault_active_)
    {
        fault_flags |= FAULT_MAGNETIC_FIELD_ALARM;
    }
    if (output_stopped_)
    {
        fault_flags |= FAULT_OUTPUT_STOPPED;
    }

    protocol_->setCustomParameter(TMC2209_EXT_PARAM_FAULT_STATUS, fault_flags);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_FAULT, encoder_fault_active_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_FAULT, magnetic_fault_active_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_OUTPUT_STOP, output_stopped_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_A,
                                  static_cast<uint32_t>(static_cast<int32_t>(phase_a_current_a_ * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_B,
                                  static_cast<uint32_t>(static_cast<int32_t>(phase_b_current_a_ * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_POS_LOOP_HZ, position_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_VEL_LOOP_HZ, velocity_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_CUR_LOOP_HZ, current_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_LAST_FAULT, fault_flags);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_POSITION, static_cast<uint32_t>(target_step_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACTUAL_POSITION, static_cast<uint32_t>(actual_step_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_FOLLOW_ERROR, static_cast<uint32_t>(follow_error_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_RAW, encoder_filtered_angle_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG, static_cast<uint32_t>(static_cast<int32_t>(getEncoderAngleMilliDegrees() * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_HIGH, magnetic_field_high_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_LOW, magnetic_field_low_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_START_RPM, static_cast<uint32_t>(motion_start_rpm_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAX_RPM, static_cast<uint32_t>(motion_max_rpm_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACCEL_RPM_S, static_cast<uint32_t>(motion_accel_rpm_s_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_MAX_RPM, static_cast<uint32_t>(angle_max_rpm_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S, static_cast<uint32_t>(angle_accel_rpm_s_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_PULSE_COUNT, motion_pulse_count_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_MODE, static_cast<uint32_t>(motion_mode_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_COMMAND, motion_running_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                  static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_POSITION_DEG,
                                  static_cast<uint32_t>(static_cast<int32_t>(closed_loop_angle_error_deg_ * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_ANGLE_DEG,
                                  static_cast<uint32_t>(static_cast<int32_t>(target_angle_deg_ * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS, step_pulse_width_ns_);
}

bool ClosedLoopController::writeParameter(uint16_t reg, uint32_t value)
{
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        closed_loop_compensation_enabled_ = value != 0U;
        position_pid_.resetIntegral();
        position_pid_.resetDeriv();
        velocity_pid_.resetIntegral();
        velocity_pid_.resetDeriv();
        return protocol_ != nullptr && protocol_->writeRegister(reg, value);
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        setEncoderZero(static_cast<uint16_t>(value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        motion_start_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        motion_max_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        motion_accel_rpm_s_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        angle_max_rpm_ = fast_clamp(static_cast<float>(value), 1.0f, 12000.0f);
        return protocol_ != nullptr && protocol_->setCustomParameter(reg, static_cast<uint32_t>(angle_max_rpm_));
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        angle_accel_rpm_s_ = fast_clamp(static_cast<float>(value), 1.0f, 12000.0f);
        return protocol_ != nullptr && protocol_->setCustomParameter(reg, static_cast<uint32_t>(angle_accel_rpm_s_));
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_CONTROL)
    {
        if (value != 0U)
        {
            pid_auto_tuner_.beginTrial();
        }
        else
        {
            pid_auto_tuner_.finishTrial();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        motion_pulse_count_ = value;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        motion_mode_ = static_cast<MotionMode>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        if (value != 0U)
        {
            startMotion();
        }
        else
        {
            closed_loop_angle_mode_enabled_ = false;
            stopMotion();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATE_ENCODER)
    {
        calibrateEncoder();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        step_pulse_width_ns_ = clampStepPulseWidthNs(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        calibration_lookup_raw_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        calibration_lookup_corrected_value_ = encoder_ == nullptr ? 0U : encoder_->getCorrectedRaw(calibration_lookup_raw_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        calibration_lookup_corrected_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        calibration_lookup_raw_value_ = encoder_ == nullptr ? 0U : lookupOriginalRaw(calibration_lookup_corrected_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        setTargetAngleDeg(static_cast<float>(value) / 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
        reg == TMC2209_EXT_PARAM_POSITION_KD || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
        reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        const float max_gain = reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KD
            ? 30000.0f
            : (reg == TMC2209_EXT_PARAM_POSITION_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
               reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
                ? 3000.0f
                : 30000.0f;
        const float gain = fast_clamp(static_cast<float>(value) / 1000.0f, 0.0f, max_gain);
        if (reg == TMC2209_EXT_PARAM_POSITION_KP) base_position_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KI) base_position_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KD) base_position_kd_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KP) base_velocity_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KI) base_velocity_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KD) base_velocity_kd_ = gain;
        position_pid_.setGains(base_position_kp_, base_position_ki_, base_position_kd_);
        velocity_pid_.setGains(base_velocity_kp_, base_velocity_ki_, base_velocity_kd_);
        position_pid_.resetIntegral();
        position_pid_.resetDeriv();
        velocity_pid_.resetIntegral();
        velocity_pid_.resetDeriv();
        return protocol_ != nullptr && protocol_->setCustomParameter(reg, value);
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_ENABLE)
    {
        stopMotion();
        if (driver_ != nullptr)
        {
            driver_->setEnable(true);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_DISABLE)
    {
        closed_loop_angle_mode_enabled_ = false;
        stopMotion();
        if (driver_ != nullptr)
        {
            driver_->setEnable(false);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS)
    {
        const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        const uint32_t half_round_steps = micro_steps_per_round / 2U;

        // 设置运动参数，交给梯形规划引擎输出脉冲
        setMotionConfig(motion_start_rpm_, motion_max_rpm_, motion_accel_rpm_s_,
                        half_round_steps, MOTION_MODE_POSITION_FORWARD);
        startMotion();

        last_en_state_ = 2U;
        return true;
    }
    if (protocol_ == nullptr)
    {
        return false;
    }
    return protocol_->writeRegister(reg, value);
}

bool ClosedLoopController::readParameter(uint16_t reg, uint32_t* value)
{
    if (value == nullptr)
    {
        return false;
    }
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        *value = closed_loop_compensation_enabled_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        *value = encoder_zero_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        *value = static_cast<uint32_t>(motion_start_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        *value = static_cast<uint32_t>(motion_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(motion_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        *value = static_cast<uint32_t>(angle_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(angle_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        *value = motion_pulse_count_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        *value = static_cast<uint32_t>(motion_mode_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        *value = motion_running_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SPEED_RPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_DEG)  // 实际是跟随误差
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(motion_follow_error_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        *value = step_pulse_width_ns_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        *value = calibration_lookup_corrected_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        *value = calibration_lookup_raw_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(target_angle_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP)
    {
        *value = static_cast<uint32_t>(base_position_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KI)
    {
        *value = static_cast<uint32_t>(base_position_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KD)
    {
        *value = static_cast<uint32_t>(base_position_kd_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG)
    {
        *value = pid_auto_tuner_.scoreMilliDegrees();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT)
    {
        *value = pid_auto_tuner_.sampleCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS)
    {
        *value = motion_steps_emitted_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(motion_speed_rpm_ *
                                                              static_cast<float>(motion_direction_) * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT)
    {
        *value = cpu_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT)
    {
        *value = control_task_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KP)
    {
        *value = static_cast<uint32_t>(base_velocity_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KI)
    {
        *value = static_cast<uint32_t>(base_velocity_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        *value = static_cast<uint32_t>(base_velocity_kd_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_REG_TSTEP)
    {
        *value = step_period_us_;
        return true;
    }
    if (reg == TMC2209_REG_MSCNT)
    {
        *value = encoder_filtered_angle_;
        return true;
    }
    if (reg == TMC2209_REG_DRV_STATUS)
    {
        *value = magnetic_field_high_ || magnetic_field_low_ ? (1U << 24U) : 0U;
        return true;
    }
    if (protocol_ == nullptr)
    {
        return false;
    }
    return protocol_->readRegister(reg, value);
}

void ClosedLoopController::updateLoopFrequencyStats(uint64_t time_ns)
{
    if (!loop_stats_enabled_)
    {
        return;
    }

    // printf("time_ns: %lu\n", time_ns);
    // printf("last_position_tick_ns_: %lu\n", last_position_tick_ns_);
    // printf("last_velocity_tick_ns_: %lu\n", last_velocity_tick_ns_);
    // printf("last_current_tick_ns_: %lu\n", last_current_tick_ns_);

    if (time_ns > last_position_tick_ns_)
    {
        const uint64_t delta_pos_ns = time_ns - last_position_tick_ns_;
        if (delta_pos_ns > 0ULL)
        {
            position_loop_hz_ = (1000000000ULL / delta_pos_ns);
            position_samples_++;
        }
        last_position_tick_ns_ = time_ns;
    }

    if (time_ns > last_velocity_tick_ns_)
    {
        const uint64_t delta_vel_ns = time_ns - last_velocity_tick_ns_;
        if (delta_vel_ns > 0ULL)
        {
            velocity_loop_hz_ = (1000000000ULL / delta_vel_ns);
            velocity_samples_++;
        }
        last_velocity_tick_ns_ = time_ns;
    }

    if (time_ns > last_current_tick_ns_)
    {
        const uint64_t delta_cur_ns = time_ns - last_current_tick_ns_;
        if (delta_cur_ns > 0ULL)
        {
            current_loop_hz_ = (1000000000ULL / delta_cur_ns);
            current_samples_++;
        }
        last_current_tick_ns_ = time_ns;
    }
}

void ClosedLoopController::setTargetStep(int32_t target_step)
{
    target_step_ = target_step;
}

void ClosedLoopController::setTargetAngleDeg(float angle_deg)
{
    if (angle_deg < 0.0f)
    {
        angle_deg = 0.0f;
    }
    else if (angle_deg > 360.0f)
    {
        angle_deg = 360.0f;
    }

    if (motion_max_rpm_ <= 0.0f)
    {
        motion_max_rpm_ = angle_max_rpm_ > 0.0f ? angle_max_rpm_ : 1200.0f;
    }
    if (motion_accel_rpm_s_ <= 0.0f)
    {
        motion_accel_rpm_s_ = angle_accel_rpm_s_ > 0.0f ? angle_accel_rpm_s_ : 6000.0f;
    }
    if (motion_start_rpm_ <= 0.0f)
    {
        motion_start_rpm_ = fminf(60.0f, motion_max_rpm_ * 0.1f);
        if (motion_start_rpm_ <= 0.0f)
        {
            motion_start_rpm_ = 10.0f;
        }
    }

    target_angle_deg_ = angle_deg;
    closed_loop_compensation_enabled_ = true;
    closed_loop_angle_mode_enabled_ = true;
    // angle_position_tolerance_deg_ = 0.1f;
    step_pulse_width_ns_ = 200U;
    k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) / static_cast<float>(stepper_common::target_tick_time));
    closed_loop_angle_error_deg_ = 0.0f;
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    motion_running_ = true;
    motion_paused_ = false;
    motion_first_run_ = true;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0U;
    motion_step_accumulator_ = 0.0f;
    motion_encoder_previous_raw_ = encoder_ != nullptr ? encoder_->readFilteredRaw() : encoder_filtered_raw_;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
    if (driver_ != nullptr)
    {
        driver_->setEnable(true);
    }
}

float ClosedLoopController::getTargetAngleDeg() const
{
    return target_angle_deg_;
}

float ClosedLoopController::getTargetAngleErrorDeg() const
{
    return closed_loop_angle_error_deg_;
}

void ClosedLoopController::setTargetVelocity(float rps)
{
    target_velocity_rps_ = rps;
}

void ClosedLoopController::setMotionConfig(float start_rpm, float max_rpm, float accel_rpm_s, uint32_t pulse_count,
                                           MotionMode mode)
{
    motion_start_rpm_ = start_rpm;
    motion_max_rpm_ = max_rpm;
    motion_accel_rpm_s_ = accel_rpm_s;
    motion_pulse_count_ = pulse_count;
    motion_mode_ = mode;
}

void ClosedLoopController::startMotion()
{
    if (motion_start_rpm_ <= 0.0f)
    {
        motion_start_rpm_ = 60.0f;
    }
    if (motion_max_rpm_ <= 0.0f)
    {
        motion_max_rpm_ = angle_max_rpm_ > 0.0f ? angle_max_rpm_ : 1200.0f;
    }
    if (motion_accel_rpm_s_ <= 0.0f)
    {
        motion_accel_rpm_s_ = angle_accel_rpm_s_ > 0.0f ? angle_accel_rpm_s_ : 6000.0f;
    }
    if (motion_max_rpm_ < motion_start_rpm_)
    {
        motion_max_rpm_ = motion_start_rpm_;
    }

    closed_loop_angle_mode_enabled_ = false;
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    motion_paused_ = false;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0U;
    motion_step_accumulator_ = 0.0f;
    motion_step_high_ = false;
    motion_zero_speed_recovery_logged_ = false;
    motion_leg_reversed_ = false;
    motion_direction_change_pending_ = true;
    motion_commanded_travel_deg_ = 0.0f;
    motion_encoder_travel_raw_ = 0;
    motion_follow_error_deg_ = 0.0f;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
    if (motion_encoder_reference_valid_)
    {
        motion_encoder_previous_raw_ = encoder_->readFilteredRaw();
    }
    if (motion_mode_ == MOTION_MODE_POSITION_REVERSE ||
        motion_mode_ == MOTION_MODE_VELOCITY_REVERSE ||
        motion_mode_ == MOTION_MODE_HOME_REVERSE ||
        motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE)
    {
        motion_direction_ = -1;
    }
    else
    {
        motion_direction_ = 1;
    }
    motion_speed_rpm_ = motion_start_rpm_;
    const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
    const float rpm2step = static_cast<float>(micro_steps_per_round) / 60.0f;

    if (driver_ != nullptr && !encoder_fault_active_ && !magnetic_fault_active_)
    {
        driver_->setEnable(true);
        driver_->setDirection(motion_direction_ > 0);
        const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        // 需要跑motion_pulse_count_个脉冲，速度从motion_start_rpm_用加速度motion_accel_rpm_s_（RPM/s）加速到motion_max_rpm_
        // 最后再用加速度motion_accel_rpm_s_（RPM/s）减速到0，脉冲用driver_->sendStepPulse(step_pulse_width_ns_)发送，
        // step_pulse_width_ns_是脉宽，固定且不可随便改，虽然短，但是也要考虑它可能存在的的影响
        // 每一圈有micro_steps_per_round个细分微步

        k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) / static_cast<float>(stepper_common::target_tick_time));

        // 单位换算 RPM → step/s；RPM/s加速度 → step/s²
        // f_step(step/s) = RPM * micro_steps_per_round / 60，每秒多少微步
        const float rpm2step = static_cast<float>(micro_steps_per_round) / 60.0f;
        motion_start_step_s_ = motion_start_rpm_ * rpm2step;
        motion_max_step_s_ = motion_max_rpm_ * rpm2step;
        motion_accel_step_s2_ = motion_accel_rpm_s_ * rpm2step;

        // 加速段：从start速度加速到max速度，需要多少步
        // 运动学公式 v² - v0² = 2*a*s → s = (v² - v0²)/(2a)
        const float v0 = motion_start_step_s_;
        const float vmax = motion_max_step_s_;
        const float a = motion_accel_step_s2_;
        printf("v0: %.6f, vmax: %.6f, a: %.6f\n", v0, vmax, a);
        const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                        motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;
        motion_leg_pulse_count_ = alternating_motion ? (motion_pulse_count_ + 1U) / 2U : motion_pulse_count_;
        accel_total_steps_ = static_cast<uint32_t>((vmax * vmax - v0 * v0) / (2.0f * a));

        // 减速段：从vmax减速到0，加速度大小同样a
        decel_total_steps_ = static_cast<uint32_t>((vmax * vmax) / (2.0f * a));
        printf("original accel_total_steps_: %lu\n", accel_total_steps_);
        printf("original decel_total_steps_: %lu\n", decel_total_steps_);

        uint32_t total_req_steps = motion_leg_pulse_count_;

        // 判断：行程够不够跑完整梯形（加速+匀速+减速），不够就退化成三角曲线（无匀速段）
        if (accel_total_steps_ + decel_total_steps_ <= total_req_steps)
        {
            // 完整梯形：存在匀速区间
            cruise_total_steps_ = total_req_steps - accel_total_steps_ - decel_total_steps_;
            motion_ramp_stage_ = RAMP_STAGE_ACCEL; // 开局加速
        }
        else
        {
            // 三角曲线：没有匀速段，重新计算能达到的峰值速度v_peak
            // v_peak² = a * total_req_steps + v0²
            // 重新分配加速/减速步数，加速到v_peak立刻减速
            cruise_total_steps_ = 0;
            float v_peak_sq = a * total_req_steps + v0 * v0;
            float v_peak = sqrtf(v_peak_sq);
            accel_total_steps_ = static_cast<uint32_t>((v_peak * v_peak - v0 * v0) / (2 * a));
            if (accel_total_steps_ < 1U) accel_total_steps_ = 1U; // 至少1步加速
            decel_total_steps_ = total_req_steps - accel_total_steps_;
            if (decel_total_steps_ < 1U) decel_total_steps_ = 1U; // 至少1步减速
            motion_ramp_stage_ = RAMP_STAGE_ACCEL; // 开局加速

            printf("v_peak_sq: %.6f\n", v_peak_sq);
            printf("v_peak: %.6f\n", v_peak);
        }
        printf("total_req_steps: %lu\n", total_req_steps);
        printf("accel_total_steps_: %lu\n", accel_total_steps_);
        printf("decel_total_steps_: %lu\n", decel_total_steps_);
        printf("cruise_total_steps_: %lu\n", cruise_total_steps_);

        // 关键标记：剩余步数 <= steps_to_decel_ 就进入减速阶段
        steps_to_decel_ = decel_total_steps_;
        current_step_speed_ = motion_start_step_s_; // 初始速度

        // 复位步累积器，用于固定频率定时器里的脉冲生成（经典DDA微分器思路）
        motion_step_accumulator_ = 0.0f;
        // ===================================================================

        // 全部参数准备好再开始
        motion_first_run_ = true;
        motion_running_ = true;
    }
}

void generate_constant_speed_step_sequence(uint32_t *seq, uint32_t seq_count, uint32_t arr) {
    for (uint32_t i = 1; i < seq_count - 1; i++){  //头尾一个是用来换向的，不是脉冲用的
        // 这里就贪方便匀速，实际测试要改成各种匀加速，S加速
        seq[i] = arr;
    }
}

void add_dir_to_step_sequence(uint32_t *seq, uint32_t seq_count, uint32_t guard_tick, bool switch_direction)
{
    // 如果开局和上一次方向相同则不必加额外换向等待，否则等一个guard_tick
    // 但是末端一定要加，避免这一局最后一步脉冲结束不到guard_tick就进入下一局开局换向
    // 注意换向后要外面自己维护方向这个状态
    if (not switch_direction) {
        seq[0] = 1;  // 这1tick至关紧要，初始化的ARR如果是0就永久卡住了
    } else {
        seq[0] = guard_tick;
    }
    seq[seq_count - 1U] = guard_tick;
}

void stop_timer_dma_for_reload()
{
    tmr_counter_enable(TMR2, FALSE);
    tmr_output_enable(TMR2, FALSE);
    tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_flag_clear(DMA1_FDT2_FLAG);
    tmr_counter_value_set(TMR2, 0U);
}

void dma_reload_arr_sequence(uint32_t *seq, uint32_t seq_count)
{
    dma_init_type dma_conf;
    dma_default_para_init(&dma_conf);
    dma_conf.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_conf.buffer_size           = static_cast<uint16_t>(seq_count);
    dma_conf.peripheral_inc_enable  = FALSE;
    dma_conf.memory_inc_enable      = TRUE;
    dma_conf.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_conf.memory_data_width      = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_conf.loop_mode_enable      = FALSE;
    dma_conf.priority              = DMA_PRIORITY_HIGH;

    dma_conf.peripheral_base_addr  = reinterpret_cast<uint32_t>(&TMR2->pr);
    dma_conf.memory_base_addr      = reinterpret_cast<uint32_t>(seq);
    dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
    dma_init(DMA1_CHANNEL2, &dma_conf);

    dma_flag_clear(DMA1_FDT2_FLAG);
    tmr_counter_value_set(TMR2, 0U);
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
}

void start_step_sequence(uint32_t *seq, uint32_t seq_count, uint32_t k_psc, uint32_t k_step_pulse_ticks, bool direction)
{
    stop_timer_dma_for_reload();
    dma_reload_arr_sequence(seq, seq_count);
    stepper_common::stepper_init_motion_timer(k_psc, k_step_pulse_ticks, direction);
}

/**
 * @brief 梯形加减速速度规划更新函数，纳秒时间基准，在FreeRTOS控制任务中异步调用
 * 依据运动规划、编码器反馈和 PID 补偿更新 STEP 脉冲速度
 * @param now_ns 系统高精度硬件时间戳(纳秒)，使用DWT CYCCNT获取真实硬件时间，禁止软件虚拟累加时间
 * @note 调用源：TMR4定时器中断通知唤醒control_task任务上下文；**禁止在中断内直接调用**
 * @note DDA微分累加器实现，任务调度延迟时依靠时间差批量补齐脉冲，保证不会丢失脉冲
 * @note sendStepPulse输出STEP脉冲，脉宽由参数step_pulse_width_ns_固定，底层驱动完成ns级脉冲生成
 */
void ClosedLoopController::rampUpdate(uint64_t now_ns)
{
    // 更新FOC频率信息
    updateLoopFrequencyStats(now_ns);

    if (driver_ == nullptr)
    {
        return;
    }

    float encoder_delta_deg = 0.0f;
    // 更新编码器信息
    if (encoder_ != nullptr)
    {
        const float previous_encoder_angle = encoder_filtered_angle_;
        const uint16_t previous_encoder_raw = encoder_filtered_raw_;
        encoder_filtered_raw_ = encoder_->readFilteredRaw();
        encoder_filtered_angle_ = encoder_->readFilteredAngle();
        magnetic_field_high_ = encoder_->magneticFieldHigh();
        magnetic_field_low_ = encoder_->magneticFieldLow();
        reportMagneticFieldAlarm(magnetic_field_high_ || magnetic_field_low_);

        motion_position_deg_ = encoder_filtered_angle_;
        encoder_delta_deg = normalize_signed_delta_deg(encoder_filtered_angle_ - previous_encoder_angle);

        if (motion_running_ && motion_encoder_reference_valid_)
        {
            const int16_t encoder_delta_raw = normalize_signed_delta_deg(static_cast<int32_t>(encoder_filtered_raw_) - static_cast<int32_t>(motion_encoder_previous_raw_));
            motion_encoder_travel_raw_ += encoder_delta_raw;

            motion_follow_error_deg_ = normalize_signed_angle_error_deg(
                static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F - motion_commanded_travel_deg_);  // 比如命令要走3度，结果才走了2度，那跟随误差就是-1度
        }
        motion_encoder_previous_raw_ = encoder_filtered_raw_;
    }

    updateEncoderCalibration();

    if (motion_running_ && !closed_loop_angle_mode_enabled_ && closed_loop_compensation_enabled_)
    {
        const bool latter_half = motion_steps_emitted_ >= motion_pulse_count_ / 2U;
        pid_auto_tuner_.addSample(motion_follow_error_deg_, latter_half);
    }

    // 如果当前没有运动在运行，直接退出
    if (!motion_running_ && !closed_loop_angle_mode_enabled_)
    {
        syncProtocolTelemetry();
        return;
    }

    if (motion_first_run_)
    {
        motion_last_step_time_ns_ = now_ns;
        motion_last_ramp_time_ns_ = now_ns;
        // printf("fuck0 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
        motion_first_run_ = false;
    }

    uint64_t delta_ramp_ns = now_ns - motion_last_ramp_time_ns_;
    if (delta_ramp_ns > 0U && encoder_ != nullptr)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        const float measured_rpm = (encoder_delta_deg / 360.0f) * (60.0f / dt);
        // 平滑计算电机的实际转速
        encoder_speed_rpm_ = encoder_speed_rpm_ * 0.98f + measured_rpm * 0.02f;
    }

    if (closed_loop_angle_mode_enabled_)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        // 以真实机械角度误差做闭环控制，单位是度/圈；
        // 微分步数只影响输出脉冲密度，不改变 PID 的误差定义，
        // 因此不同驱动或不同细分设置不会让同一组 PID 失效。
        const float desired_error_deg = normalize_signed_angle_error_deg(target_angle_deg_ - motion_position_deg_);
        closed_loop_angle_error_deg_ = desired_error_deg;

        if (dt > 0.0f)
        {
            const float position_error_rev = desired_error_deg / 360.0f;
            const float position_reference_rps = position_pid_.update(position_error_rev, dt);
            const float velocity_error_rps = position_reference_rps - (encoder_speed_rpm_ / 60.0f);
            const float velocity_correction_rps = velocity_pid_.update(velocity_error_rps, dt);
            const float max_rps = angle_max_rpm_ / 60.0f;
            const float unconstrained_rps = fast_clamp(position_reference_rps + velocity_correction_rps,
                                                       -max_rps,
                                                       max_rps);
            const float target_direction_rps = desired_error_deg >= 0.0f ? 1.0f : -1.0f;
            const float directed_rps = unconstrained_rps * target_direction_rps < 0.0f
                ? 0.0f
                : unconstrained_rps;
            const float accel_limit_rps2 = angle_accel_rpm_s_ / 60.0f;
            const float previous_rps = motion_speed_rpm_ / 60.0f;
            const float max_delta_rps = accel_limit_rps2 * dt;
            const float desired_rps = fast_clamp(directed_rps,
                                                 previous_rps - max_delta_rps,
                                                 previous_rps + max_delta_rps);
            const float calibration_rps = calibration_stage_ == CALIBRATION_HOMING &&
                                                  fast_abs(desired_error_deg) <= 0.25f
                ? 0.0f
                : desired_rps;
            if (calibration_stage_ == CALIBRATION_HOMING && calibration_rps == 0.0f)
            {
                motion_step_accumulator_ = 0.0f;
                position_pid_.resetIntegral();
                position_pid_.resetDeriv();
                velocity_pid_.resetIntegral();
                velocity_pid_.resetDeriv();
            }
            const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
            current_step_speed_ = calibration_rps * static_cast<float>(micro_steps_per_round);
            motion_speed_rpm_ = calibration_rps * 60.0f;
            if (calibration_rps != 0.0f)
            {
                motion_direction_ = calibration_rps >= 0.0f ? 1 : -1;
            }
            if (driver_ != nullptr)
            {
                driver_->setDirection(motion_direction_ > 0);
            }
        }

        // 注释掉这段，这段代码不能启用，因为我需要它摇摆才能测好pid，但是以后需要强制定在某个角度可能有用
        // if (fast_abs(closed_loop_angle_error_deg_) <= angle_position_tolerance_deg_)
        // {
        //     current_step_speed_ = 0.0f;
        //     motion_speed_rpm_ = 0.0f;
        //     motion_step_accumulator_ = 0.0f;
        //     position_pid_.resetIntegral();
        //     position_pid_.resetDeriv();
        //     velocity_pid_.resetIntegral();
        //     velocity_pid_.resetDeriv();
        // }
    }
    else
    {
        const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                                        motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;
        if (alternating_motion && !motion_leg_reversed_ && motion_leg_pulse_count_ > 0U &&
            motion_steps_emitted_ >= motion_leg_pulse_count_ &&
            motion_steps_emitted_ < motion_pulse_count_)
        {
            motion_direction_ = static_cast<int8_t>(-motion_direction_);
            motion_leg_reversed_ = true;
            motion_direction_change_pending_ = true;
            motion_ramp_stage_ = RAMP_STAGE_ACCEL;
            current_step_speed_ = 0.0f;
            motion_speed_rpm_ = 0.0f;
            motion_step_accumulator_ = 0.0f;
            printf("[MOTION] turnaround: leg_steps=%lu total=%lu dir=%d\r\n",
                   static_cast<unsigned long>(motion_leg_pulse_count_),
                   static_cast<unsigned long>(motion_pulse_count_),
                   motion_direction_);
        }
        // 条件：已经输出全部需要的脉冲，运动正常结束
        if (motion_steps_emitted_ >= motion_pulse_count_)
        {
            if (calibration_stage_ == CALIBRATION_MOVING)
            {
                // static uint64_t dma_done_since_ns = 0ULL;
                // if (dma_flag_get(DMA1_FDT2_FLAG) == RESET)
                // {
                //     dma_done_since_ns = 0ULL;
                //     syncProtocolTelemetry();
                //     return;
                // }
                // if (dma_done_since_ns == 0ULL)
                // {
                //     stepper_common::stepper_stop_motion_timer();
                //     dma_done_since_ns = now_ns;
                //     syncProtocolTelemetry();
                //     return;
                // }
                // if (now_ns - dma_done_since_ns < 1000000ULL)
                // {
                //     syncProtocolTelemetry();
                //     return;
                // }

                // dma_done_since_ns = 0ULL;
                motion_running_ = false;
                motion_first_run_ = false;
                current_step_speed_ = 0.0f;
                motion_speed_rpm_ = 0.0f;
                motion_step_accumulator_ = 0.0f;
                calibration_stage_ = CALIBRATION_SAMPLING;
                calibration_sample_count_ = 0U;
                calibration_sample_attempts_ = 0U;
                calibration_sample_sum_ = 0;
                calibration_sample_min_ = 0x7FFFFFFF;
                calibration_sample_max_ = (-0x7FFFFFFF - 1);
                syncProtocolTelemetry();
                return;
            }
            printf("motion_steps_emitted_: %lu, motion_pulse_count_: %lu\n", motion_steps_emitted_, motion_pulse_count_);
            motion_running_ = false; // 标记运动停止
            motion_ramp_stage_ = RAMP_STAGE_DONE; // 设置状态为运动完成
            current_step_speed_ = 0.0f; // 运动结束强制把当前速度清零，防止下次运动残留速度
            stopMotion();
            syncProtocolTelemetry();
            return;
        }
    }

    if (!closed_loop_angle_mode_enabled_)
    {
        // 计算还剩余多少微步脉冲有待输出
        const uint32_t remaining_total_steps = motion_pulse_count_ - motion_steps_emitted_;
        const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                                        motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;
        const uint32_t leg_steps_emitted = alternating_motion && motion_leg_reversed_
            ? motion_steps_emitted_ - motion_leg_pulse_count_
            : motion_steps_emitted_;
        const uint32_t remaining_steps = alternating_motion
            ? (motion_leg_reversed_ ? remaining_total_steps : motion_leg_pulse_count_ - leg_steps_emitted)
            : remaining_total_steps;
        const float dt_ramp_s = static_cast<float>(delta_ramp_ns) / 1.0e9f;

        if (motion_ramp_stage_ == RAMP_STAGE_ACCEL)
        {
            current_step_speed_ += motion_accel_step_s2_ * dt_ramp_s;
            if (current_step_speed_ >= motion_max_step_s_)
            {
                current_step_speed_ = motion_max_step_s_;
                motion_ramp_stage_ = RAMP_STAGE_CRUISE;
            }
            if (remaining_steps <= steps_to_decel_)
            {
                motion_ramp_stage_ = RAMP_STAGE_DECEL;
            }
        }
        else if (motion_ramp_stage_ == RAMP_STAGE_CRUISE)
        {
            if ((steps_to_decel_ != 0xFFFFFFFFU) && (remaining_steps <= steps_to_decel_))
            {
                motion_ramp_stage_ = RAMP_STAGE_DECEL;
            }
        }
        else if (motion_ramp_stage_ == RAMP_STAGE_DECEL)
        {
            current_step_speed_ -= motion_accel_step_s2_ * dt_ramp_s;
            if (current_step_speed_ < 0.0f)
            {
                current_step_speed_ = 0.0f;
            }
        }
    }

    if (closed_loop_compensation_enabled_ && motion_running_ && !closed_loop_angle_mode_enabled_ &&
        encoder_ != nullptr && delta_ramp_ns > 0U && motion_max_step_s_ > 0.0f)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        // 位置跟随误差定义为“命令 - 实际”。如果命令已经领先真实位置，说明该减速；
        // 这里必须取反后再送入位置 PID，否则正误差会被误当成要求继续加速。
        const float position_reference_rps = position_pid_.update(motion_follow_error_deg_ / 360.0f, dt);
        const float measured_motion_rps = encoder_speed_rpm_ / 60.0f;
        const float velocity_error_rps = position_reference_rps - measured_motion_rps;
        const float velocity_correction_rps = velocity_pid_.update(velocity_error_rps, dt);
        const float max_correction_rps = motion_max_rpm_ / 60.0f;
        const float correction_rps = fast_clamp(position_reference_rps + velocity_correction_rps,
                                                -max_correction_rps,
                                                max_correction_rps);
        const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        const float correction_step_s = correction_rps * static_cast<float>(motion_direction_) *
                                        static_cast<float>(micro_steps_per_round);
        const float max_correction_step_s = motion_accel_step_s2_ * dt;
        const float limited_correction_step_s = fast_clamp(correction_step_s,
                                                           -max_correction_step_s,
                                                           max_correction_step_s);
        current_step_speed_ = fast_clamp(current_step_speed_ + limited_correction_step_s,
                                         0.0f,
                                         motion_max_step_s_);
        motion_speed_rpm_ = current_step_speed_ * 60.0f / static_cast<float>(micro_steps_per_round);
    }

    const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                                    motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;

    if (!closed_loop_angle_mode_enabled_ && driver_ != nullptr)
    {
        const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        motion_speed_rpm_ = current_step_speed_ * 60.0f / static_cast<float>(micro_steps_per_round);
    }

    // 更新本次的时间戳，作为下一次调用的“上一次时间点”
    // printf("fuck1 motion_last_ramp_time_ns_: %f, now_ns: %f\n", motion_last_ramp_time_ns_/1e9, now_ns/1e9);
    motion_last_ramp_time_ns_ = now_ns;
    // printf("fuck2 motion_last_ramp_time_ns_: %f, now_ns: %f\n", motion_last_ramp_time_ns_/1e9, now_ns/1e9);

    // ========== 2.DDA微分累加器：生成步进脉冲，核心部分 ==========
    // DDA原理：步进步数增量 = 瞬时速度(step/s) × 流逝时间(s)
    // 即使任务被抢占延迟很久，delta_step_ns会记录真实流逝时间，累加器累积需要输出的步数
    // while循环一次性输出多个脉冲，做到调度抖动下不丢脉冲

    // 获取两次脉冲生成之间真实流逝的纳秒
    const uint64_t delta_step_ns = now_ns - motion_last_step_time_ns_;
    // 时间单位换算：纳秒 → 秒
    const double dt_step_s = static_cast<double>(delta_step_ns) / 1.0e9f;

    // 累加本次时间内应该产生的步数（浮点数，允许小数累积）
    motion_step_accumulator_ += fast_abs(current_step_speed_) * dt_step_s;
    if (closed_loop_angle_mode_enabled_ && motion_step_accumulator_ > 1.0)
    {
        motion_step_accumulator_ = 1.0;
    }

    // TMR2 是 32 位计数器，因此 ARR 序列必须是 32 位。
    // 这里复用同一份周期表，正转和反转都只需要切换 DIR 输出 + 重新装载同一块 RAM，
    // 这样既能完成“正转一圈 -> 反转一圈”，又不会把 RAM 翻倍消耗掉。

    if ((motion_step_accumulator_ >= 1.0f) &&
        (closed_loop_angle_mode_enabled_ || motion_steps_emitted_ < motion_pulse_count_))
    {
        // 人为规定第一个和最后arr周期是用来提前和延后换向的，
        // 如果是同向，这两个的arr周期为0，否则arr为guard_tick
        static uint32_t arr_seq_cycle[MAX_PULSE_ARR_LEN + 2];
        uint32_t num_steps = static_cast<uint32_t>(floor(motion_step_accumulator_));
        if (!closed_loop_angle_mode_enabled_)
        {
            const uint32_t remaining_total_steps = motion_pulse_count_ - motion_steps_emitted_;
            const uint32_t leg_steps_emitted = alternating_motion && motion_leg_reversed_
                ? motion_steps_emitted_ - motion_leg_pulse_count_
                : motion_steps_emitted_;
            const uint32_t remaining_steps = alternating_motion
                ? (motion_leg_reversed_ ? remaining_total_steps : motion_leg_pulse_count_ - leg_steps_emitted)
                : remaining_total_steps;
            if (num_steps > remaining_steps)
            {
                num_steps = remaining_steps;
            }
        }
        if (num_steps > MAX_PULSE_ARR_LEN)
        {
            num_steps = MAX_PULSE_ARR_LEN;
        }
        const uint32_t seq_count = num_steps + 2U;
        const bool current_direction = motion_direction_ > 0;
        // uint32_t step_ticks = stepper_common::F_APB / current_step_speed_;

        const uint64_t direction_switch_guard_time_ns = 2 * stepper_common::guard_tick * stepper_common::target_tick_time;
        const uint64_t reserved_time_ns = 5;
        const uint64_t overhead_ns = reserved_time_ns + direction_switch_guard_time_ns;
        if (delta_step_ns <= overhead_ns)
        {
            motion_last_step_time_ns_ = now_ns;
            syncProtocolTelemetry();
            return;
        }
        const uint64_t remaining_time = delta_step_ns - overhead_ns;
        const uint64_t remaining_ticks = remaining_time / stepper_common::target_tick_time;

        // 要尽快执行，不可以用下面那种平均的平滑模式，会丢步
        uint32_t step_ticks = k_step_pulse_ticks * 2;  // 最少脉宽两倍，留足高电平脉宽之余的低电平脉宽

        // 丢步直接丢了，不打印b报错，因为打印反而会加剧丢步
        // uint64_t max_iter_time_ns = (2 * stepper_common::guard_tick + step_ticks * num_steps) * stepper_common::target_tick_time;
        // // 因为硬件TMR定时器+DMA工作是异步的，耗时必须短于软件定时器迭代时间，否则就会输出延迟
        // if (delta_step_ns < max_iter_time_ns)
        // {
        //     printf("step_ticks: %lu, num_steps: %lu, now_ns: %.6fus, motion_last_step_time_ns_: %.6fus, delta_step_ns: %.6fus\n", step_ticks, num_steps, now_ns / 1000.f, motion_last_step_time_ns_ / 1000.f, delta_step_ns / 1000.f);
        //     printf("shitfuck, delta_step_ns: %.6f us < max_iter_time_ns: %.6f us, lower your iter rate.\n", delta_step_ns / 1000.f, max_iter_time_ns / 1000.f);
        // }

        // uint64_t step_ticks_wide = remaining_ticks / num_steps;
        // const uint32_t min_step_ticks = stepper_common::k_step_pulse_ticks * 2U;
        // if (step_ticks_wide < min_step_ticks)
        // {
        //     step_ticks_wide = min_step_ticks;
        // }
        // if (step_ticks_wide > UINT32_MAX)
        // {
        //     step_ticks_wide = UINT32_MAX;
        // }
        // const uint32_t step_ticks = static_cast<uint32_t>(step_ticks_wide);
        // if (step_ticks < stepper_common::k_step_pulse_ticks * 2)
        // {
        //     printf("step_ticks: %lu, num_steps: %lu, now_ns: %.6fus, motion_last_step_time_ns_: %.6fus, delta_step_ns: %.6fus\n", step_ticks, num_steps, now_ns / 1000.f, motion_last_step_time_ns_ / 1000.f, delta_step_ns / 1000.f);
        //     printf("shitfuck, delta_step_ns: %.6f us, step_ticks < 2*min_k_step_pulse_ticks %lu us, lower your iter rate.\n", delta_step_ns / 1000.f, stepper_common::k_step_pulse_ticks * 2);
        // }
#ifdef USE_SOFT_PULSE
        // 只要累加器≥1，代表需要输出1个step脉冲；循环批量输出，直到没有脉冲待输出或者全部脉冲发完
        // cpu软脉冲实现，效率低
        for (int i = 0; i < num_steps; i++)
        {
            // 调用驱动输出STEP脉冲；脉冲高电平宽度固定为step_pulse_width_ns_，底层实现ns延时
            driver_->sendStepPulse(step_pulse_width_ns_);
        }
#else
        // todo 注意这里仅恒速脉冲，也就是每个rampupdate中仅开头一段时间就尽可能快地把脉冲恒速跑完，虽然间隔很短，但总会带来一些卡顿的问题，后期应该改成平滑插入脉冲
        // todo 注意这里目前仅支持每次rampupdate仅支持最多一次换向。后续需要根据换向次数和位置提前规划好换向点并安排好guard ticks
        generate_constant_speed_step_sequence(arr_seq_cycle, seq_count, step_ticks);
        add_dir_to_step_sequence(arr_seq_cycle, seq_count, stepper_common::guard_tick,
                     motion_direction_change_pending_);
        start_step_sequence(arr_seq_cycle, seq_count, stepper_common::k_psc, stepper_common::k_step_pulse_ticks, current_direction);
#endif
        motion_direction_change_pending_ = false;

        if (!closed_loop_angle_mode_enabled_)
        {
            motion_steps_emitted_ += num_steps;
            motion_steps_emitted_signed += motion_direction_ * num_steps;  // 正向就加步数，反向就减
            const float step_angle_deg = 360.0f / static_cast<float>(getMicroStepsPerRound(driver_));
            motion_commanded_travel_deg_ = static_cast<float>(motion_steps_emitted_signed) * step_angle_deg;
        }
        motion_step_accumulator_ -= num_steps;
    }

    // 更新脉冲模块的时间戳
    motion_last_step_time_ns_ = now_ns;
    syncProtocolTelemetry();
}

void ClosedLoopController::stopMotion()
{
    printf("motion stopping!!! motion_commanded_travel_deg_: %f, motion_encoder_travel_deg_: %f\n", motion_commanded_travel_deg_, static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F);
    motion_running_ = false;
    motion_first_run_ = false;
    motion_paused_ = false;
    motion_speed_rpm_ = 0.0f;
    encoder_speed_rpm_ = 0.0f;
    target_velocity_rps_ = 0.0f;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0U;
    motion_leg_pulse_count_ = 0U;
    motion_leg_reversed_ = false;
    motion_direction_change_pending_ = false;
    motion_step_accumulator_ = 0.0f;
    motion_last_step_time_ns_ = 0ULL;
    motion_last_ramp_time_ns_ = 0ULL;
    // printf("fuck3 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
    motion_step_high_ = false;
    if (driver_ != nullptr)
    {
        stepper_common::stepper_stop_motion_timer();
        driver_->setStepState(false);
    }
}

bool ClosedLoopController::isMotionRunning() const
{
    return motion_running_;
}

float ClosedLoopController::getMotionSpeedRpm() const
{
    return motion_speed_rpm_;
}

float ClosedLoopController::getMotionPositionDeg() const
{
    return motion_position_deg_;
}

void ClosedLoopController::setPid(float kp, float ki, float kd)
{
    base_position_kp_ = kp;
    base_position_ki_ = ki;
    base_position_kd_ = kd;
    base_velocity_kp_ = kp * 0.5f;
    base_velocity_ki_ = ki * 0.5f;
    base_velocity_kd_ = kd * 0.5f;
    position_pid_.setGains(base_position_kp_, base_position_ki_, base_position_kd_);
    velocity_pid_.setGains(base_velocity_kp_, base_velocity_ki_, base_velocity_kd_);
}

void ClosedLoopController::enableAdaptivePid(bool enable)
{
    adaptive_pid_enabled_ = enable;
}

void ClosedLoopController::setAdaptivePidConfig(const PidAutoTuneConfig& config)
{
    adaptive_config_ = config;
}

void ClosedLoopController::updateAdaptivePid(float speed_rps, float acceleration_rps2, float follow_error)
{
    if (!adaptive_pid_enabled_)
    {
        return;
    }

    const float speed_scale = fast_clamp(fast_abs(speed_rps) / (adaptive_config_.max_speed_rps + 1.0e-6f), 0.0f, 1.0f);
    const float accel_scale = fast_clamp(fast_abs(acceleration_rps2) / (adaptive_config_.acceleration_rps2 + 1.0e-6f),
                                         0.0f, 1.0f);
    const float error_scale = fast_clamp(fast_abs(follow_error) / (adaptive_config_.target_follow_error + 1.0e-6f),
                                         0.0f, 1.0f);
    const float gain_scale = 1.0f + speed_scale * 0.45f + accel_scale * 0.35f + error_scale * 0.20f;

    position_pid_.kp = base_position_kp_ * gain_scale;
    position_pid_.ki = base_position_ki_ * (0.8f + speed_scale * 0.6f);
    position_pid_.kd = base_position_kd_ * (0.9f + accel_scale * 0.5f);

    velocity_pid_.kp = base_velocity_kp_ * gain_scale;
    velocity_pid_.ki = base_velocity_ki_ * (0.8f + speed_scale * 0.6f);
    velocity_pid_.kd = base_velocity_kd_ * (0.9f + accel_scale * 0.5f);
}

void ClosedLoopController::enableLoopStats(bool enable)
{
    loop_stats_enabled_ = enable;
    if (enable)
    {
        last_position_tick_ns_ = 0ULL;
        last_velocity_tick_ns_ = 0ULL;
        last_current_tick_ns_ = 0ULL;
    }
}

void ClosedLoopController::getLoopFrequencyStats(LoopFrequencyStats* stats) const
{
    if (stats == nullptr)
    {
        return;
    }

    stats->position_loop_hz = position_loop_hz_;
    stats->velocity_loop_hz = velocity_loop_hz_;
    stats->current_loop_hz = current_loop_hz_;
    stats->position_samples = position_samples_;
    stats->velocity_samples = velocity_samples_;
    stats->current_samples = current_samples_;
}

void ClosedLoopController::resetLoopFrequencyStats()
{
    position_loop_hz_ = 0U;
    velocity_loop_hz_ = 0U;
    current_loop_hz_ = 0U;
    position_samples_ = 0U;
    velocity_samples_ = 0U;
    current_samples_ = 0U;
    last_position_tick_ns_ = 0ULL;
    last_velocity_tick_ns_ = 0ULL;
    last_current_tick_ns_ = 0ULL;
}

int32_t ClosedLoopController::getPositionSteps() const
{
    return actual_step_;
}

int32_t ClosedLoopController::getTargetSteps() const
{
    return target_step_;
}

float ClosedLoopController::getEncoderZero() const
{
    return encoder_zero_;
}

void ClosedLoopController::setEncoderZero(float zero_angle)
{
    encoder_zero_ = zero_angle;
    if (encoder_ != nullptr)
    {
        encoder_->setZero(zero_angle);
    }
}

float ClosedLoopController::getFollowError() const
{
    return follow_error_;
}

float ClosedLoopController::getEncoderFilteredAngle() const
{
    return encoder_filtered_angle_;
}

float ClosedLoopController::getEncoderAngleMilliDegrees() const
{
    return encoder_filtered_angle_;
}

bool ClosedLoopController::isMagneticFieldHigh() const
{
    return magnetic_field_high_;
}

bool ClosedLoopController::isMagneticFieldLow() const
{
    return magnetic_field_low_;
}

float ClosedLoopController::getPhaseCurrentTelemetryA() const
{
    return phase_a_current_a_;
}

float ClosedLoopController::getPhaseCurrentTelemetryB() const
{
    return phase_b_current_a_;
}

uint32_t ClosedLoopController::getPositionLoopHz() const
{
    return position_loop_hz_;
}

uint32_t ClosedLoopController::getVelocityLoopHz() const
{
    return velocity_loop_hz_;
}

uint32_t ClosedLoopController::getCurrentLoopHz() const
{
    return current_loop_hz_;
}

bool ClosedLoopController::readCalibrationTablePair(uint8_t table, uint16_t pair_index,
                                                    uint32_t* packed_values) const
{
    if (encoder_ == nullptr || packed_values == nullptr || calibration_stage_ != CALIBRATION_IDLE)
    {
        return false;
    }

    const uint32_t first_index = static_cast<uint32_t>(pair_index) * 2U;
    uint16_t first = 0xFFFFU;
    uint16_t second = 0xFFFFU;
    if (table == 0U && first_index < CALIBRATION_TABLE_SIZE)
    {
        first = encoder_->calibrationMainValue(static_cast<uint16_t>(first_index));
        if (first_index + 1U < CALIBRATION_TABLE_SIZE)
        {
            second = encoder_->calibrationMainValue(static_cast<uint16_t>(first_index + 1U));
        }
    }
    else if (table == 1U && first_index < 65536U)
    {
        first = encoder_->calibrationFastValue(first_index);
        second = encoder_->calibrationFastValue(first_index + 1U);
    }
    else
    {
        return false;
    }

    *packed_values = (static_cast<uint32_t>(first) << 16U) | second;
    return true;
}

uint16_t ClosedLoopController::lookupCorrectedRaw(uint16_t raw_value) const
{
    if (encoder_ == nullptr)
    {
        return 0U;
    }
    return encoder_->getCorrectedRaw(raw_value);
}

uint16_t ClosedLoopController::lookupOriginalRaw(uint16_t corrected_value) const
{
    if (encoder_ == nullptr)
    {
        return 0U;
    }

    uint16_t best_raw = 0U;
    uint32_t best_delta = 0xFFFFFFFFU;
    for (uint32_t raw_value = 0U; raw_value < 65536U; ++raw_value)
    {
        const uint16_t corrected = encoder_->getCorrectedRaw(static_cast<uint16_t>(raw_value));
        const uint32_t delta = corrected >= corrected_value ? static_cast<uint32_t>(corrected - corrected_value)
                                                            : static_cast<uint32_t>(corrected_value - corrected);
        if (delta < best_delta)
        {
            best_delta = delta;
            best_raw = static_cast<uint16_t>(raw_value);
            if (delta == 0U)
            {
                break;
            }
        }
    }
    return best_raw;
}

bool ClosedLoopController::readCalibrationTableChecksum(uint32_t* checksum) const
{
    if (encoder_ == nullptr || checksum == nullptr || calibration_stage_ != CALIBRATION_IDLE)
    {
        return false;
    }
    *checksum = encoder_->calibrationChecksum();
    return true;
}

void ClosedLoopController::calibrateEncoder()
{
    if (calibration_stage_ != CALIBRATION_IDLE)
    {
        return;
    }
    if (driver_ == nullptr || encoder_ == nullptr || encoder_fault_active_ || magnetic_fault_active_)
    {
        printf("[CALIBRATION] start rejected: driver/encoder unavailable or fault active\r\n");
        return;
    }

    step_pulse_width_ns_ = 200U;
    k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) / static_cast<float>(stepper_common::target_tick_time));
    calibration_saved_start_rpm_ = motion_start_rpm_;
    calibration_saved_max_rpm_ = motion_max_rpm_;
    calibration_saved_accel_rpm_s_ = motion_accel_rpm_s_;
    calibration_saved_pulse_count_ = motion_pulse_count_;
    calibration_saved_pulse_width_ns_ = step_pulse_width_ns_;
    calibration_saved_motion_mode_ = motion_mode_;
    calibration_saved_compensation_enabled_ = closed_loop_compensation_enabled_;
    calibration_index_ = 0U;
    calibration_home_stable_count_ = 0U;
    calibration_home_previous_angle_ = encoder_->readFilteredAngle();
    calibration_stage_ = CALIBRATION_HOMING;
    setTargetAngleDeg(0.0f);
    printf("[CALIBRATION] homing to zero\r\n");
}

bool ClosedLoopController::startEncoderCalibrationMove(uint16_t index)
{
    const uint32_t steps_per_round = getMicroStepsPerRound(driver_);
    const uint64_t table_size = CALIBRATION_TABLE_SIZE;
    const uint32_t first_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t next_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index + 1U) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t steps = next_step - first_step;
    if (steps == 0U)
    {
        printf("[CALIBRATION] unsupported microstep resolution\r\n");
        finishEncoderCalibration(false);
        return false;
    }

    motion_start_rpm_ = 60.0f;
    motion_max_rpm_ = 60.0f;
    motion_accel_rpm_s_ = 60.0f;
    motion_pulse_count_ = steps;
    motion_mode_ = MOTION_MODE_POSITION_FORWARD;
    step_pulse_width_ns_ = 200ULL;
    calibration_stage_ = CALIBRATION_MOVING;
    motion_direction_ = 1;
    startMotion();
    if (!motion_running_)
    {
        printf("[CALIBRATION] could not start move at index=%u\r\n", static_cast<unsigned>(index));
        finishEncoderCalibration(false);
        return false;
    }
    return true;
}

void ClosedLoopController::updateEncoderCalibration()
{
    if (calibration_stage_ == CALIBRATION_HOMING)
    {
        calibration_home_wait_ticks_++;
        if (calibration_home_wait_ticks_ >= 1000000U)
        {
            printf("[CALIBRATION] homing timeout\r\n");
            finishEncoderCalibration(false);
            return;
        }
        const float position = encoder_->readFilteredAngle();
        const float position_error = normalize_signed_angle_error_deg(-position);
        const float position_delta = normalize_signed_delta_deg(position - calibration_home_previous_angle_);
        calibration_home_previous_angle_ = position;
        if (fast_abs(position_error) <= 0.25f && fast_abs(position_delta) <= 0.02f)
        {
            if (calibration_home_stable_count_ < 100U)
            {
                calibration_home_stable_count_++;
            }
        }
        else
        {
            calibration_home_stable_count_ = 0U;
            calibration_home_wait_ticks_ = 0U;
        }
        if (calibration_home_stable_count_ >= 100U)
        {
            closed_loop_angle_mode_enabled_ = false;
            stopMotion();
            calibration_stage_ = CALIBRATION_SAMPLING;
            calibration_sample_count_ = 0U;
            calibration_sample_attempts_ = 0U;
            calibration_sample_sum_ = 0;
            calibration_sample_min_ = 0x7FFFFFFF;
            calibration_sample_max_ = (-0x7FFFFFFF - 1);
            printf("[CALIBRATION] zero reached; sampling index=0\r\n");
        }
        return;
    }

    if (calibration_stage_ != CALIBRATION_SAMPLING)
    {
        return;
    }

    const uint16_t raw = encoder_->readRaw();
    // printf("[CALIBRATION] sample %3u, try: %u, raw=%5u\n", static_cast<unsigned>(calibration_sample_count_),
    //        static_cast<unsigned>(calibration_sample_attempts_), static_cast<unsigned>(raw));
    calibration_sample_attempts_++;
    if (raw != 0xFFFFU)
    {
        int32_t unwrapped = static_cast<int32_t>(raw);
        if (calibration_sample_count_ > 0U)
        {
            int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(calibration_sample_anchor_);
            if (delta > 32767)
            {
                delta -= 65536;
            }
            else if (delta < -32768)
            {
                delta += 65536;
            }
            unwrapped = static_cast<int32_t>(calibration_sample_anchor_) + delta;
        }
        else
        {
            calibration_sample_anchor_ = raw;
        }
        calibration_sample_sum_ += unwrapped;
        if (unwrapped < calibration_sample_min_)
        {
            calibration_sample_min_ = unwrapped;
        }
        if (unwrapped > calibration_sample_max_)
        {
            calibration_sample_max_ = unwrapped;
        }
        calibration_sample_count_++;
    }

    if (calibration_sample_count_ < 202U)
    {
        if (calibration_sample_attempts_ >= 1000U)
        {
            printf("[CALIBRATION] insufficient encoder samples at index=%u\r\n",
                   static_cast<unsigned>(calibration_index_));
            finishEncoderCalibration(false);
        }
        return;
    }

    const int32_t trimmed_mean = (calibration_sample_sum_ - calibration_sample_min_ - calibration_sample_max_) / 200;
    const uint16_t mean_raw = static_cast<uint16_t>(static_cast<uint32_t>(trimmed_mean) & 0xFFFFU);
    encoder_->feedCalibrationSample(calibration_index_, mean_raw);
    if (calibration_index_ + 1U >= CALIBRATION_TABLE_SIZE)
    {
        finishEncoderCalibration(true);
        return;
    }
    calibration_index_++;
    startEncoderCalibrationMove(calibration_index_);
}

void ClosedLoopController::finishEncoderCalibration(bool save_table)
{
    calibration_stage_ = CALIBRATION_SAVING;
    closed_loop_angle_mode_enabled_ = false;
    stopMotion();

    bool saved = false;
    if (save_table)
    {
        saved = encoder_ != nullptr && encoder_->saveCalibration();
    }
    motion_start_rpm_ = calibration_saved_start_rpm_;
    motion_max_rpm_ = calibration_saved_max_rpm_;
    motion_accel_rpm_s_ = calibration_saved_accel_rpm_s_;
    motion_pulse_count_ = calibration_saved_pulse_count_;
    motion_mode_ = calibration_saved_motion_mode_;
    step_pulse_width_ns_ = calibration_saved_pulse_width_ns_;
    closed_loop_compensation_enabled_ = calibration_saved_compensation_enabled_;
    calibration_stage_ = CALIBRATION_IDLE;
    printf("[CALIBRATION] finished saved=%u\r\n", saved ? 1U : 0U);
}
