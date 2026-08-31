#include "closed_loop_controller.h"
#include <math.h>
#include "at32f403a_407_board.h"

#define STEP_EDGE_TIMEOUT_US  200U
#define STEP_PERIOD_US_DEFAULT 5000U
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
    step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), last_process_time_us_(0U),
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
    driver_->setEnable(true);
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
  else if (!active && !magnetic_fault_active_ && driver_ != nullptr)
  {
    driver_->setEnable(true);
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
  else if (!active && !encoder_fault_active_ && driver_ != nullptr)
  {
    driver_->setEnable(true);
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
}

bool ClosedLoopController::writeParameter(uint8_t reg, uint32_t value)
{
  if (protocol_ == nullptr)
  {
    return false;
  }
  return protocol_->writeRegister(reg, value);
}

bool ClosedLoopController::readParameter(uint8_t reg, uint32_t *value)
{
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

  if (step_state != last_step_state_)
  {
    if (step_state != 0U)
    {
      command_step_ += (dir_state != 0U) ? 1 : -1;
      target_step_ = command_step_;
      driver_->setStepState(true);
      delay_us(STEP_EDGE_TIMEOUT_US);
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
  syncProtocolTelemetry();

  if (driver_ == nullptr || encoder_ == nullptr)
  {
    return;
  }

  // 读取编码器原始角度，并换算成相对零点的步数。
  // 这里的逻辑是把编码器量化到一个可比较的相对位置值，
  // 后续 position_error 能直接反映目标和当前位置的偏差。
  uint16_t encoder_raw = encoder_->readRawAngle();
  int32_t actual_step = 0;

  if (encoder_raw > encoder_zero_)
  {
    actual_step = static_cast<int32_t>(encoder_raw - encoder_zero_);
  }
  else
  {
    actual_step = -static_cast<int32_t>(encoder_zero_ - encoder_raw);
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
      measured_velocity_rps_ = (static_cast<float>(actual_step_ - last_actual_step_)) / dt / 200.0f;
    }
  }

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

float ClosedLoopController::getFollowError() const
{
  return follow_error_;
}
