#include "closed_loop_controller.h"
#include <math.h>
#include "at32f403a_407_board.h"

#define STEP_EDGE_TIMEOUT_US  200U
#define STEP_PERIOD_US_DEFAULT 5000U
#define OPEN_LOOP_STEPS_PER_REV 51200U
#define MAX_PID_OUTPUT        2000.0f
#define MAX_I_TERM            100.0f

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
    motion_running_(false), motion_paused_(false), motion_speed_rpm_(0.0f), encoder_speed_rpm_(0.0f), motion_position_deg_(0.0f),
    motion_last_step_time_us_(0U), motion_last_ramp_time_us_(0U), motion_steps_emitted_(0U), motion_step_accumulator_(0.0f), motion_direction_(1),
    motion_step_high_(false), step_pulse_width_us_(2U),
    step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), encoder_raw_angle_(0U),
    magnetic_field_high_(false), magnetic_field_low_(false), last_process_time_us_(0U),
    last_step_state_(0U), last_dir_state_(0U), last_en_state_(0U),
    stop_on_encoder_fault_(true), stop_on_magnetic_fault_(true), encoder_fault_active_(false),
    magnetic_fault_active_(false), output_stopped_(false), phase_a_current_a_(0.0f),
    phase_b_current_a_(0.0f), loop_stats_enabled_(false), last_position_tick_us_(0U),
    last_velocity_tick_us_(0U), last_current_tick_us_(0U), position_loop_hz_(0U),
    velocity_loop_hz_(0U), current_loop_hz_(0U), position_samples_(0U), velocity_samples_(0U),
    current_samples_(0U)
{
  position_pid_.setGains(base_position_kp_, base_position_ki_, base_position_kd_);
  velocity_pid_.setGains(base_velocity_kp_, base_velocity_ki_, base_velocity_kd_);
}

void ClosedLoopController::init(StepperDriver *driver, AngleEncoder *encoder)
{
  driver_ = driver;
  encoder_ = encoder;

  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  gpio_init_type input_gpio;
  gpio_default_para_init(&input_gpio);
  input_gpio.gpio_mode = GPIO_MODE_INPUT;
  input_gpio.gpio_pull = GPIO_PULL_DOWN;
  input_gpio.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init(GPIOC, &input_gpio);

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

void ClosedLoopController::setProtocol(ClosedLoopDriverProtocol *protocol)
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
  protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_)));
  protocol_->setCustomParameter(TMC2209_EXT_PARAM_POSITION_DEG,
                                static_cast<uint32_t>(static_cast<int32_t>(motion_position_deg_ * 1000.0f)));
  protocol_->setCustomParameter(TMC2209_EXT_PARAM_WAVEFORM_WINDOW_MS, motion_window_ms_);
  protocol_->setCustomParameter(TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_US, step_pulse_width_us_);
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
  if (reg == TMC2209_EXT_PARAM_WAVEFORM_WINDOW_MS)
  {
    setWaveformWindowMs(value);
    return true;
  }
  if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_US)
  {
    step_pulse_width_us_ = value < 1U ? 1U : (value > 20U ? 20U : value);
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
  if (reg == TMC2209_EXT_PARAM_SINGLE_FORWARD_STEP)
  {
    stopMotion();
    if (driver_ != nullptr)
    {
      driver_->setEnable(true);
      driver_->setDirection(true);
      driver_->setStepState(true);
      stepper_common::stepper_delay_us(step_pulse_width_us_);
      driver_->setStepState(false);
    }
    return true;
  }
  if (protocol_ == nullptr)
  {
    return false;
  }
  return protocol_->writeRegister(reg, value);
}

bool ClosedLoopController::readParameter(uint16_t reg, uint32_t *value)
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
  if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_US)
  {
    *value = step_pulse_width_us_;
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

  gpio_type *step_port = GPIOC;
  gpio_type *dir_port = GPIOC;
  gpio_type *en_port = GPIOC;

  uint8_t step_state = gpio_input_data_bit_read(step_port, GPIO_PINS_15);
  uint8_t dir_state = gpio_input_data_bit_read(dir_port, GPIO_PINS_14);
  uint8_t en_state = gpio_input_data_bit_read(en_port, GPIO_PINS_13);

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
      driver_->setStepState(true);
      driver_->setStepState(false);
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

void ClosedLoopController::updateLoopFrequencyStats(uint32_t time_us)
{
  if (!loop_stats_enabled_)
  {
    return;
  }

  if (time_us > last_position_tick_us_)
  {
    const uint32_t delta_pos_us = time_us - last_position_tick_us_;
    if (delta_pos_us > 0U)
    {
      position_loop_hz_ = (1000000U / delta_pos_us);
      position_samples_++;
    }
    last_position_tick_us_ = time_us;
  }

  if (time_us > last_velocity_tick_us_)
  {
    const uint32_t delta_vel_us = time_us - last_velocity_tick_us_;
    if (delta_vel_us > 0U)
    {
      velocity_loop_hz_ = (1000000U / delta_vel_us);
      velocity_samples_++;
    }
    last_velocity_tick_us_ = time_us;
  }

  if (time_us > last_current_tick_us_)
  {
    const uint32_t delta_cur_us = time_us - last_current_tick_us_;
    if (delta_cur_us > 0U)
    {
      current_loop_hz_ = (1000000U / delta_cur_us);
      current_samples_++;
    }
    last_current_tick_us_ = time_us;
  }
}

// process() 是闭环控制器最核心的函数，负责完成一整个控制周期：
// 1. 读取编码器实际位置
// 2. 计算位置误差
// 3. 用位置 PID 生成速度参考
// 4. 通过速度 PID 调整输出修正
// 5. 最终通过步进输出和驱动器状态更新实现闭环控制
//
// 设计上采用双环结构：
// - 外层位置环：控制“目标位置 vs 当前位置”的误差
// - 内层速度环：控制“速度参考 vs 当前速度”的误差
// 这样可以把大范围位置控制和短周期速度控制分开，提高稳定性和响应性。
void ClosedLoopController::process(uint32_t time_us)
{
  updateLoopFrequencyStats(time_us);

  if (driver_ == nullptr)
  {
    return;
  }

  if (encoder_ != nullptr)
  {
    uint16_t encoder_raw = encoder_->readRawAngle();
    encoder_raw_angle_ = encoder_raw;
    magnetic_field_high_ = encoder_->magneticFieldHigh();
    magnetic_field_low_ = encoder_->magneticFieldLow();
    reportMagneticFieldAlarm(magnetic_field_high_ || magnetic_field_low_);

    motion_position_deg_ = static_cast<float>(static_cast<int32_t>(encoder_raw) - static_cast<int32_t>(encoder_zero_)) * 360.0f / 65536.0f;
    if (motion_position_deg_ > 180.0f)
    {
      motion_position_deg_ -= 360.0f;
    }
    else if (motion_position_deg_ < -180.0f)
    {
      motion_position_deg_ += 360.0f;
    }
  }

  if (motion_running_)
  {
    const float max_rpm = motion_max_rpm_ > 0.0f ? motion_max_rpm_ : motion_start_rpm_;
    const float accel_rpm = motion_accel_rpm_s_ > 0.0f ? motion_accel_rpm_s_ : 300.0f;
    const float target_speed = max_rpm > 0.0f ? max_rpm : motion_start_rpm_;
    if (motion_last_ramp_time_us_ == 0U)
    {
      motion_last_ramp_time_us_ = time_us;
    }

    const float dt = static_cast<float>(time_us - motion_last_ramp_time_us_) * 1.0e-6f;
    if (dt > 0.0f)
    {
      if (motion_speed_rpm_ < target_speed)
      {
        motion_speed_rpm_ = motion_speed_rpm_ + accel_rpm * dt;
        if (motion_speed_rpm_ > target_speed)
        {
          motion_speed_rpm_ = target_speed;
        }
      }
      else if (motion_speed_rpm_ > target_speed)
      {
        motion_speed_rpm_ = motion_speed_rpm_ - accel_rpm * dt;
        if (motion_speed_rpm_ < target_speed)
        {
          motion_speed_rpm_ = target_speed;
        }
      }
      motion_speed_rpm_ = (motion_direction_ > 0) ? motion_speed_rpm_ : -motion_speed_rpm_;
      motion_last_ramp_time_us_ = time_us;
    }

    const float commanded_rpm = fast_abs(motion_speed_rpm_);
    const float step_hz = (commanded_rpm * static_cast<float>(OPEN_LOOP_STEPS_PER_REV)) / 60.0f;
    if (!output_stopped_ && step_hz > 0.0f)
    {
      driver_->setDirection(motion_direction_ > 0);
      stepper_common::stepper_update_motion_timer(static_cast<uint32_t>(step_hz), step_pulse_width_us_);
      if (motion_last_step_time_us_ != 0U && time_us > motion_last_step_time_us_)
      {
        const float elapsed_s = static_cast<float>(time_us - motion_last_step_time_us_) * 1.0e-6f;
        motion_step_accumulator_ += step_hz * elapsed_s;
        const uint32_t completed_steps = static_cast<uint32_t>(motion_step_accumulator_);
        motion_step_accumulator_ -= static_cast<float>(completed_steps);
        motion_steps_emitted_ += completed_steps;
      }
      motion_last_step_time_us_ = time_us;
      const uint32_t max_steps = motion_pulse_count_ == 0U ? UINT32_MAX : motion_pulse_count_;
      if (motion_steps_emitted_ >= max_steps)
      {
        stopMotion();
        return;
      }
    }
    else if (output_stopped_)
    {
      stepper_common::stepper_stop_motion_timer();
    }
  }

  // 读取编码器原始角度，并换算成相对零点的步数。
  // 这里的逻辑是把编码器量化到一个可比较的相对位置值，
  // 后续 position_error 能直接反映目标和当前位置的偏差。
  uint16_t encoder_raw = encoder_raw_angle_;
  int32_t actual_step = 0;

  actual_step = static_cast<int32_t>(encoder_raw) - static_cast<int32_t>(encoder_zero_);
  if (actual_step > 32768)
  {
    actual_step -= 65536;
  }
  else if (actual_step < -32768)
  {
    actual_step += 65536;
  }

  actual_step_ = actual_step;
  follow_error_ = static_cast<float>(target_step_ - actual_step_);
  if (last_process_time_us_ == 0U)
  {
    last_process_time_us_ = time_us;
  }

  // 根据两个采样时刻之间的位置变化，计算实际速度。
  // 这里的 /200.0f 是粗略折算到电机机械转速单位，体现在闭环控制中
  // 是“步数变化量 -> 速度参考”的桥接。
  if (last_process_time_us_ != 0U && time_us > last_process_time_us_)
  {
    uint32_t dt_us = time_us - last_process_time_us_;
    float dt = static_cast<float>(dt_us) * 1.0e-6f;
    if (dt > 0.0f)
    {
      measured_velocity_rps_ = (static_cast<float>(actual_step_ - last_actual_step_)) / 65536.0f / dt;
      encoder_speed_rpm_ = measured_velocity_rps_ * 60.0f;
    }
  }

  syncProtocolTelemetry();

  // 位置误差 = 目标位置 - 当前位置。
  float position_error = static_cast<float>(target_step_ - actual_step_);

  // 位置环输出的 correction 是“速度参考”而不是直接驱动量。
  // 这个设计将位置控制和速度控制拆开，避免大范围位置误差直接作用到驱动器。
  float position_correction = position_pid_.update(position_error, 0.001f);

  // 速度参考 -> 速度误差 -> 速度 PID -> 更细粒度的控制输出。
  float velocity_reference = position_correction;
  float velocity_error = velocity_reference - measured_velocity_rps_;
  float velocity_correction = velocity_pid_.update(velocity_error, 0.001f);

  // 速度修正量会参与步进周期的补偿：
  // 如果速度误差较大，就缩短步进周期，增加动作频率；
  // 如果误差较小，则维持或放宽步进节奏。 
  float correction_gain = velocity_correction / MAX_PID_OUTPUT;
  correction_gain = fast_clamp(correction_gain, -1.0f, 1.0f);

  if (step_period_us_ > 10U)
  {
    uint32_t compensated_step_period = static_cast<uint32_t>(static_cast<float>(step_period_us_) * (1.0f - 0.25f * correction_gain));
    if (compensated_step_period < 10U)
    {
      compensated_step_period = 10U;
    }
    step_period_us_ = compensated_step_period;
  }

  // 当接近目标时，清零积分项，避免大误差导致积分发散。
  if (fast_abs(position_error) < 0.25f)
  {
    position_pid_.resetIntegral();
    velocity_pid_.resetIntegral();
  }

  // 更接近零时，清理微分历史，减少抖动和噪声放大。
  if (fast_abs(position_error) < 0.1f)
  {
    position_pid_.resetDeriv();
    velocity_pid_.resetDeriv();
  }

  last_actual_step_ = actual_step_;
  last_process_time_us_ = time_us;

  syncStepDirection();
}

void ClosedLoopController::setTargetStep(int32_t target_step)
{
  target_step_ = target_step;
}

void ClosedLoopController::setTargetVelocity(float rps)
{
  target_velocity_rps_ = rps;
}

void ClosedLoopController::setMotionConfig(float start_rpm, float max_rpm, float accel_rpm_s, uint32_t pulse_count, MotionMode mode)
{
  motion_start_rpm_ = start_rpm;
  motion_max_rpm_ = max_rpm;
  motion_accel_rpm_s_ = accel_rpm_s;
  motion_pulse_count_ = pulse_count;
  motion_mode_ = mode;
}

void ClosedLoopController::startMotion()
{
  motion_running_ = true;
  motion_paused_ = false;
  motion_steps_emitted_ = 0U;
  motion_step_accumulator_ = 0.0f;
  motion_last_step_time_us_ = 0U;
  motion_last_ramp_time_us_ = 0U;
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
  if (driver_ != nullptr && !encoder_fault_active_ && !magnetic_fault_active_)
  {
    driver_->setEnable(true);
    driver_->setDirection(motion_direction_ > 0);
    stepper_common::stepper_start_motion_timer(
      static_cast<uint32_t>(fast_abs(motion_speed_rpm_) * static_cast<float>(OPEN_LOOP_STEPS_PER_REV) / 60.0f),
      step_pulse_width_us_);
  }
}

void ClosedLoopController::stopMotion()
{
  motion_running_ = false;
  motion_paused_ = false;
  motion_speed_rpm_ = 0.0f;
  encoder_speed_rpm_ = 0.0f;
  target_velocity_rps_ = 0.0f;
  motion_steps_emitted_ = 0U;
  motion_step_accumulator_ = 0.0f;
  motion_last_step_time_us_ = 0U;
  motion_last_ramp_time_us_ = 0U;
  motion_step_high_ = false;
  if (driver_ != nullptr)
  {
    stepper_common::stepper_stop_motion_timer();
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

void ClosedLoopController::setAdaptivePidConfig(const PidAutoTuneConfig &config)
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
  const float accel_scale = fast_clamp(fast_abs(acceleration_rps2) / (adaptive_config_.acceleration_rps2 + 1.0e-6f), 0.0f, 1.0f);
  const float error_scale = fast_clamp(fast_abs(follow_error) / (adaptive_config_.target_follow_error + 1.0e-6f), 0.0f, 1.0f);
  const float gain_scale = 1.0f + speed_scale * 0.45f + accel_scale * 0.35f + error_scale * 0.20f;

  position_pid_.kp = base_position_kp_ * gain_scale;
  position_pid_.ki = base_position_ki_ * (0.8f + speed_scale * 0.6f);
  position_pid_.kd = base_position_kd_ * (0.9f + accel_scale * 0.5f);

  velocity_pid_.kp = base_velocity_kp_ * gain_scale;
  velocity_pid_.ki = base_velocity_ki_ * (0.8f + speed_scale * 0.6f);
  velocity_pid_.kd = base_velocity_kd_ * (0.9f + accel_scale * 0.5f);
}

void ClosedLoopController::calibrateEncoder(const EncoderCalibrationConfig &config)
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
    last_position_tick_us_ = 0U;
    last_velocity_tick_us_ = 0U;
    last_current_tick_us_ = 0U;
  }
}

void ClosedLoopController::getLoopFrequencyStats(LoopFrequencyStats *stats) const
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
  last_position_tick_us_ = 0U;
  last_velocity_tick_us_ = 0U;
  last_current_tick_us_ = 0U;
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
