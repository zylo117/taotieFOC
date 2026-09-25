#include "closed_loop_controller.h"
#include <cmath>
#include "at32f403a_407_board.h"

#define STEP_EDGE_TIMEOUT_US    200U
#define STEP_PERIOD_US_DEFAULT  5000U
#define FULL_STEPS_PER_ROUND    200U  // 1.8度步进
#define MIN_STEP_PULSE_NS       100ULL
#define DEFAULT_STEP_PULSE_NS   2000ULL
#define MAX_STEP_PULSE_NS       20000ULL
#define MAX_PID_OUTPUT          2000.0f
#define MAX_I_TERM              100.0f

namespace
{
    constexpr uint32_t kHardwareStepPeriodTick = 9765UL;
    constexpr uint32_t kHardwarePulseWidthTick = 25UL;
    constexpr uint32_t kHardwareGuardTicks = 2UL;
    constexpr uint32_t kHardwareStepWindowLimit = 12800UL;

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
        uint16_t microsteps = 32U;
        if (driver != nullptr)
        {
            microsteps = driver->config().microsteps;
        }
        if (microsteps == 0U)
        {
            microsteps = 32U;
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
      base_position_kp_(1.0f), base_position_ki_(0.04f), base_position_kd_(0.02f),
      base_velocity_kp_(0.7f), base_velocity_ki_(0.06f), base_velocity_kd_(0.01f),
      adaptive_pid_enabled_(false), adaptive_config_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      target_step_(0), actual_step_(0), last_actual_step_(0), command_step_(0),
      follow_error_(0.0f), measured_velocity_rps_(0.0f), target_velocity_rps_(0.0f),
      motion_start_rpm_(0.0f), motion_max_rpm_(0.0f), motion_accel_rpm_s_(0.0f),
    motion_pulse_count_(0U), motion_window_ms_(2000U), motion_mode_(MOTION_MODE_POSITION_FORWARD),
    simulation_mode_(false),
      motion_running_(false), motion_first_run_(false), motion_paused_(false), motion_speed_rpm_(0.0f), encoder_speed_rpm_(0.0f),
      motion_position_deg_(0.0f),
      motion_last_step_time_us_(0U), motion_last_ramp_time_us_(0U), motion_steps_emitted_(0U),
      motion_step_accumulator_(0.0f), motion_direction_(1),
      motion_step_high_(false), step_pulse_width_ns_(DEFAULT_STEP_PULSE_NS),
      step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), encoder_raw_angle_(0U),
      magnetic_field_high_(false), magnetic_field_low_(false), last_process_time_us_(0U),
      last_step_state_(0U), last_dir_state_(0U), last_en_state_(0U),
      stop_on_encoder_fault_(true), stop_on_magnetic_fault_(true), encoder_fault_active_(false),
      magnetic_fault_active_(false), output_stopped_(false), phase_a_current_a_(0.0f),
      phase_b_current_a_(0.0f), loop_stats_enabled_(false), last_position_tick_ns_(0ULL),
      last_velocity_tick_ns_(0ULL), last_current_tick_ns_(0ULL), position_loop_hz_(0U),
      velocity_loop_hz_(0U), current_loop_hz_(0U), position_samples_(0U), velocity_samples_(0U),
      current_samples_(0U)
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
        encoder_zero_ = encoder_->readRawAngle();
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
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_A, static_cast<uint32_t>(phase_a_current_a_ * 1000.0f));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_B, static_cast<uint32_t>(phase_b_current_a_ * 1000.0f));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_POS_LOOP_HZ, position_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_VEL_LOOP_HZ, velocity_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_CUR_LOOP_HZ, current_loop_hz_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_LAST_FAULT, fault_flags);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_POSITION, static_cast<uint32_t>(target_step_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACTUAL_POSITION, static_cast<uint32_t>(actual_step_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_FOLLOW_ERROR, static_cast<uint32_t>(follow_error_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_RAW, encoder_raw_angle_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG, getEncoderAngleMilliDegrees());
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_HIGH, magnetic_field_high_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_LOW, magnetic_field_low_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_START_RPM, static_cast<uint32_t>(motion_start_rpm_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAX_RPM, static_cast<uint32_t>(motion_max_rpm_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACCEL_RPM_S, static_cast<uint32_t>(motion_accel_rpm_s_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_PULSE_COUNT, motion_pulse_count_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_MODE, static_cast<uint32_t>(motion_mode_));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_COMMAND, motion_running_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_HOST_SIMULATE, simulation_mode_ ? 1U : 0U);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                  static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_POSITION_DEG,
                                  static_cast<uint32_t>(static_cast<int32_t>(motion_position_deg_ * 1000.0f)));
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_WAVEFORM_WINDOW_MS, motion_window_ms_);
    protocol_->setCustomParameter(TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS, step_pulse_width_ns_);
}

bool ClosedLoopController::writeParameter(uint16_t reg, uint32_t value)
{
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
            stopMotion();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_HOST_SIMULATE)
    {
        setSimulationMode(value != 0U);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_WAVEFORM_WINDOW_MS)
    {
        setWaveformWindowMs(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        step_pulse_width_ns_ = clampStepPulseWidthNs(value);
        return true;
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
    if (reg == TMC2209_EXT_PARAM_HOST_SIMULATE)
    {
        *value = simulation_mode_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SPEED_RPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_DEG)
    {
        *value = static_cast<uint32_t>(motion_position_deg_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_WAVEFORM_WINDOW_MS)
    {
        *value = motion_window_ms_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        *value = step_pulse_width_ns_;
        return true;
    }
    if (reg == TMC2209_REG_TSTEP)
    {
        *value = step_period_us_;
        return true;
    }
    if (reg == TMC2209_REG_MSCNT)
    {
        *value = encoder_raw_angle_;
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

void ClosedLoopController::syncStepDirection()
{
    if (driver_ == nullptr)
    {
        return;
    }

    gpio_type* step_port = GPIOB;
    gpio_type* dir_port = GPIOA;
    gpio_type* en_port = GPIOA;

    uint8_t step_state = gpio_input_data_bit_read(step_port, GPIO_PINS_3);
    uint8_t dir_state = gpio_input_data_bit_read(dir_port, GPIO_PINS_15);
    uint8_t en_state = gpio_input_data_bit_read(en_port, EN_OUT_PIN);

    if (motion_running_)
    {
        return;
    }

    if (step_state != last_step_state_)
    {
        if (step_state != 0U)
        {
            command_step_ += (dir_state != 0U) ? 1 : -1;
            target_step_ = command_step_;
            driver_->sendStepPulse(step_pulse_width_ns_);
        }
        last_step_state_ = step_state;
    }

    if (dir_state != last_dir_state_)
    {
        driver_->setDirection(dir_state != 0U);
        last_dir_state_ = dir_state;
    }

    if (en_state != last_en_state_)
    {
        driver_->setEnable(en_state != 0U);
        last_en_state_ = en_state;
    }
}

void ClosedLoopController::consumeQueuedStepDirEvents()
{
    stepper_common::StepDirCaptureEvent event;
    while (stepper_common::stepper_pop_capture_event(&event))
    {
        if (simulation_mode_)
        {
            command_step_ += (event.dir) ? 1 : -1;
            target_step_ = command_step_;
            actual_step_ = command_step_;
            if (event.step)
            {
                motion_steps_emitted_ += 1U;
                if (driver_ != nullptr)
                {
                    driver_->setDirection(event.dir);
                    driver_->sendStepPulse(step_pulse_width_ns_);
                }
            }
            else if (driver_ != nullptr)
            {
                driver_->setDirection(event.dir);
            }
            continue;
        }

        if (event.step)
        {
            command_step_ += (event.dir) ? 1 : -1;
            target_step_ = command_step_;
            actual_step_ = command_step_;
            if (driver_ != nullptr)
            {
                driver_->setDirection(event.dir);
                driver_->sendStepPulse(step_pulse_width_ns_);
            }
        }
        else if (driver_ != nullptr)
        {
            driver_->setDirection(event.dir);
        }
    }
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

void ClosedLoopController::setSimulationMode(bool enable)
{
    simulation_mode_ = enable;
    if (enable)
    {
        stepper_common::stepper_reset_capture_ring_buffer();
    }
}

bool ClosedLoopController::isSimulationMode() const
{
    return simulation_mode_;
}

void ClosedLoopController::startMotion()
{
    motion_paused_ = false;
    motion_steps_emitted_ = 0U;
    motion_step_accumulator_ = 0.0f;
    motion_step_high_ = false;
    if (motion_mode_ == MOTION_MODE_POSITION_REVERSE ||
        motion_mode_ == MOTION_MODE_VELOCITY_REVERSE ||
        motion_mode_ == MOTION_MODE_HOME_REVERSE)
    {
        motion_direction_ = -1;
    }
    else
    {
        motion_direction_ = 1;
    }
    motion_speed_rpm_ = motion_direction_ > 0 ? motion_start_rpm_ : -motion_start_rpm_;
    const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
    const float rpm2step = static_cast<float>(micro_steps_per_round) / 60.0f;

    if (driver_ != nullptr && !encoder_fault_active_ && !magnetic_fault_active_)
    {
        driver_->setEnable(true);
        driver_->setDirection(motion_direction_ > 0);
        // 需要跑motion_pulse_count_个脉冲，速度从motion_start_rpm_用加速度motion_accel_rpm_s_（RPM/s）加速到motion_max_rpm_
        // 最后再用加速度motion_accel_rpm_s_（RPM/s）减速到0，脉冲用driver_->sendStepPulse(step_pulse_width_ns_)发送，
        // step_pulse_width_ns_是脉宽，固定且不可随便改，虽然短，但是也要考虑它可能存在的的影响
        // 每一圈有micro_steps_per_round个细分微步

        // 单位换算 RPM → step/s；RPM/s加速度 → step/s²
        // f_step(step/s) = RPM * micro_steps_per_round / 60，每秒多少微步
        motion_start_step_s_ = motion_start_rpm_ * rpm2step;
        motion_max_step_s_ = motion_max_rpm_ * rpm2step;
        motion_accel_step_s2_ = motion_accel_rpm_s_ * rpm2step;

        // 加速段：从start速度加速到max速度，需要多少步
        // 运动学公式 v² - v0² = 2*a*s → s = (v² - v0²)/(2a)
        const float v0 = motion_start_step_s_;
        const float vmax = motion_max_step_s_;
        const float a = motion_accel_step_s2_;
        accel_total_steps_ = static_cast<uint32_t>((vmax * vmax - v0 * v0) / (2.0f * a));

        // 减速段：从vmax减速到0，加速度大小同样a
        decel_total_steps_ = static_cast<uint32_t>((vmax * vmax) / (2.0f * a));

        uint32_t total_req_steps = motion_pulse_count_;

        printf("total_req_steps: %lu\n", total_req_steps);
        printf("accel_total_steps_: %lu\n", accel_total_steps_);
        printf("decel_total_steps_: %lu\n", decel_total_steps_);

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
            printf("accel_total_steps_: %lu\n", accel_total_steps_);
            printf("decel_total_steps_: %lu\n", decel_total_steps_);
        }
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

/**
 * @brief 梯形加减速速度规划更新函数，纳秒时间基准，在FreeRTOS控制任务中异步调用
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

    // 更新编码器信息
    if (encoder_ != nullptr)
    {
        uint16_t encoder_raw = encoder_->readRawAngle();
        encoder_raw_angle_ = encoder_raw;
        magnetic_field_high_ = encoder_->magneticFieldHigh();
        magnetic_field_low_ = encoder_->magneticFieldLow();
        reportMagneticFieldAlarm(magnetic_field_high_ || magnetic_field_low_);

        motion_position_deg_ = static_cast<float>(encoder_zero_ - encoder_raw) * 360.0f / 65536.0f;
        if (motion_position_deg_ > 180.0f)
        {
            motion_position_deg_ -= 360.0f;
        }
        else if (motion_position_deg_ < -180.0f)
        {
            motion_position_deg_ += 360.0f;
        }
    }

    // 同步上传信息到串口
    syncProtocolTelemetry();

    // 如果当前没有运动在运行，直接退出
    if (!motion_running_)
    {
        return;
    }

    if (motion_first_run_)
    {
        motion_last_step_time_ns_ = now_ns;
        motion_last_ramp_time_ns_ = now_ns;
        // printf("fuck0 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
        motion_first_run_ = false;
    }

    // 条件：已经输出全部需要的脉冲，运动正常结束
    if (motion_steps_emitted_ >= motion_pulse_count_)
    {
        printf("motion_steps_emitted_: %lu, motion_pulse_count_: %lu\n", motion_steps_emitted_, motion_pulse_count_);
        motion_running_ = false; // 标记运动停止
        motion_ramp_stage_ = RAMP_STAGE_DONE; // 设置状态为运动完成
        current_step_speed_ = 0.0f; // 运动结束强制把当前速度清零，防止下次运动残留速度
        stopMotion();
        return;
    }

    // 计算还剩余多少微步脉冲有待输出
    uint32_t remaining_steps = motion_pulse_count_ - motion_steps_emitted_;

    // ========== 1.加减速阶段的速度积分更新 ==========
    // 计算距离上一次rampUpdate调用的时间差(纳秒)
    uint64_t delta_ramp_ns = now_ns - motion_last_ramp_time_ns_;
    // 纳秒转换为秒，用于加速度公式计算
    float dt_ramp_s = static_cast<float>(delta_ramp_ns) / 1.0e9f;

    if (motion_ramp_stage_ == RAMP_STAGE_ACCEL)
    {
        // 【加速阶段】速度 = 当前速度 + 加速度 * 时间
        current_step_speed_ += motion_accel_step_s2_ * dt_ramp_s;

        // 速度限幅：到达设定最大速度，切换到匀速阶段
        if (current_step_speed_ >= motion_max_step_s_)
        {
            current_step_speed_ = motion_max_step_s_;
            motion_ramp_stage_ = RAMP_STAGE_CRUISE;
        }

        // 关键判断：剩余步数 <= 减速需要的总步数 → 必须立刻切入减速，防止冲过目标位置
        // 短行程三角曲线模式下会直接从加速转入减速，不会经过匀速
        if (remaining_steps <= steps_to_decel_)
        {
            motion_ramp_stage_ = RAMP_STAGE_DECEL;
        }
    }
    else if (motion_ramp_stage_ == RAMP_STAGE_CRUISE)
    {
        // 【匀速阶段】速度保持不变，只监控剩余步数，判断何时开启减速
        // steps_to_decel_为极大值时不会触发切换到减速
        if ((steps_to_decel_ != 0xFFFFFFFFU) && (remaining_steps <= steps_to_decel_))
        {
            motion_ramp_stage_ = RAMP_STAGE_DECEL;
        }
    }
    else if (motion_ramp_stage_ == RAMP_STAGE_DECEL)
    {
        // 【减速阶段】速度 = 当前速度 - 加速度 * 时间（减速加速度大小与加速一致）
        current_step_speed_ -= motion_accel_step_s2_ * dt_ramp_s;

        // 速度下限保护，不能出现负速度
        if (current_step_speed_ < 0.0f)
        {
            current_step_speed_ = 0.0f;
        }
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
    uint64_t delta_step_ns = now_ns - motion_last_step_time_ns_;
    // 时间单位换算：纳秒 → 秒
    double dt_step_s = static_cast<float>(delta_step_ns) / 1.0e9f;

    // 累加本次时间内应该产生的步数（浮点数，允许小数累积）
    motion_step_accumulator_ += current_step_speed_ * dt_step_s;

    // 只要累加器≥1，代表需要输出1个step脉冲；循环批量输出，直到没有脉冲待输出或者全部脉冲发完
    // TODO: 尽可能前移这部分，让now_ns更加即时
    while ((motion_step_accumulator_ >= 1.0f) && (motion_steps_emitted_ < motion_pulse_count_))
    {
        const uint32_t remaining = motion_pulse_count_ - motion_steps_emitted_;
        const uint32_t due_steps = static_cast<uint32_t>(motion_step_accumulator_);
        const uint32_t burst_steps = (due_steps < remaining) ? due_steps : remaining;
        const uint32_t safe_burst = (burst_steps > kHardwareStepWindowLimit) ? kHardwareStepWindowLimit : burst_steps;

        if (simulation_mode_)
        {
            for (uint32_t i = 0U; i < safe_burst; ++i)
            {
                stepper_common::stepper_push_capture_event(static_cast<uint32_t>(now_ns / 1000ULL), true,
                                                           motion_direction_ > 0);
            }
            motion_steps_emitted_ += safe_burst;
            motion_step_accumulator_ -= static_cast<float>(safe_burst);
            if (motion_steps_emitted_ >= motion_pulse_count_)
            {
                break;
            }
            continue;
        }

        if (safe_burst > 0U)
        {
            const bool direction = (motion_direction_ > 0);
            if (stepper_common::stepper_plan_dma_window(safe_burst, direction,
                                                       kHardwareStepPeriodTick,
                                                       kHardwarePulseWidthTick,
                                                       kHardwareGuardTicks))
            {
                motion_steps_emitted_ += safe_burst;
                motion_step_accumulator_ -= static_cast<float>(safe_burst);
                continue;
            }
        }

        driver_->sendStepPulse(step_pulse_width_ns_);
        motion_steps_emitted_++;
        motion_step_accumulator_ -= 1.0f;
    }

    // 更新脉冲模块的时间戳
    motion_last_step_time_ns_ = now_ns;
}

void ClosedLoopController::stopMotion()
{
    printf("motion stopping!!!");
    motion_running_ = false;
    motion_first_run_ = false;
    motion_paused_ = false;
    motion_speed_rpm_ = 0.0f;
    encoder_speed_rpm_ = 0.0f;
    target_velocity_rps_ = 0.0f;
    motion_steps_emitted_ = 0U;
    motion_step_accumulator_ = 0.0f;
    motion_last_step_time_ns_ = 0ULL;
    motion_last_ramp_time_ns_ = 0ULL;
    // printf("fuck3 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
    motion_step_high_ = false;
    if (driver_ != nullptr)
    {
        // stepper_common::stepper_stop_motion_timer();
        driver_->setStepState(false);
        driver_->setEnable(false);
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

uint32_t ClosedLoopController::getWaveformWindowMs() const
{
    return motion_window_ms_;
}

void ClosedLoopController::setWaveformWindowMs(uint32_t window_ms)
{
    if (window_ms < 10U)
    {
        motion_window_ms_ = 10U;
    }
    else if (window_ms > 2000U)
    {
        motion_window_ms_ = 2000U;
    }
    else
    {
        motion_window_ms_ = window_ms;
    }
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

void ClosedLoopController::calibrateEncoder(const EncoderCalibrationConfig& config)
{
    if (encoder_ == nullptr)
    {
        return;
    }

    EncoderCalibrationResult result = {false, false, false, false, 0, 0.0f, 0.0f};
    encoder_->calibrate(config, &result);

    if (result.offset_ok)
    {
        encoder_zero_ = static_cast<uint16_t>(encoder_zero_ + static_cast<uint16_t>(result.offset_correction));
        encoder_->setZero(encoder_zero_);
    }
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

uint16_t ClosedLoopController::getEncoderZero() const
{
    return encoder_zero_;
}

void ClosedLoopController::setEncoderZero(uint16_t zero_angle)
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

uint16_t ClosedLoopController::getEncoderRawAngle() const
{
    return encoder_raw_angle_;
}

uint32_t ClosedLoopController::getEncoderAngleMilliDegrees() const
{
    return static_cast<uint32_t>((static_cast<uint64_t>(encoder_raw_angle_) * 360000ULL) / 65536ULL);
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
