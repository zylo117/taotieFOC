#include "closed_loop_controller.h"
#include <math.h>
#include "at32f403a_407_board.h"

#define STEP_EDGE_TIMEOUT_US  200U
#define STEP_PERIOD_US_DEFAULT 5000U
#define MAX_PID_OUTPUT        2000.0f
#define MAX_I_TERM            100.0f

static float clampf(float value, float min_value, float max_value)
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

float PidController::update(float error, float dt)
{
  float derivative = 0.0f;
  float output = 0.0f;

  if (dt > 0.0f)
  {
    derivative = (error - last_error) / dt;
  }

  integral += error * dt;
  if (integral > max_integral)
  {
    integral = max_integral;
  }
  else if (integral < -max_integral)
  {
    integral = -max_integral;
  }

  output = kp * error + ki * integral + kd * derivative;
  if (output > max_output)
  {
    output = max_output;
  }
  else if (output < -max_output)
  {
    output = -max_output;
  }

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
  : driver_(nullptr), encoder_(nullptr),
    base_position_kp_(1.0f), base_position_ki_(0.04f), base_position_kd_(0.02f),
    base_velocity_kp_(0.7f), base_velocity_ki_(0.06f), base_velocity_kd_(0.01f),
    adaptive_pid_enabled_(false), adaptive_config_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    target_step_(0), actual_step_(0), last_actual_step_(0), command_step_(0),
    follow_error_(0.0f), measured_velocity_rps_(0.0f), target_velocity_rps_(0.0f),
    step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), last_process_time_us_(0U),
    last_step_state_(0U), last_dir_state_(0U), last_en_state_(0U)
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

void ClosedLoopController::process(uint32_t time_us)
{
  if (driver_ == nullptr || encoder_ == nullptr)
  {
    return;
  }

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

  if (last_process_time_us_ != 0U && time_us > last_process_time_us_)
  {
    uint32_t dt_us = time_us - last_process_time_us_;
    float dt = static_cast<float>(dt_us) * 1.0e-6f;
    if (dt > 0.0f)
    {
      measured_velocity_rps_ = (static_cast<float>(actual_step_ - last_actual_step_)) / dt / 200.0f;
    }
  }

  float position_error = static_cast<float>(target_step_ - actual_step_);
  float position_correction = position_pid_.update(position_error, 0.001f);

  float velocity_reference = position_correction;
  float velocity_error = velocity_reference - measured_velocity_rps_;
  float velocity_correction = velocity_pid_.update(velocity_error, 0.001f);

  float correction_gain = velocity_correction / MAX_PID_OUTPUT;
  correction_gain = fminf(1.0f, fmaxf(-1.0f, correction_gain));

  if (step_period_us_ > 10U)
  {
    uint32_t compensated_step_period = static_cast<uint32_t>(static_cast<float>(step_period_us_) * (1.0f - 0.25f * correction_gain));
    if (compensated_step_period < 10U)
    {
      compensated_step_period = 10U;
    }
    step_period_us_ = compensated_step_period;
  }

  if (fabsf(position_error) < 0.25f)
  {
    position_pid_.resetIntegral();
    velocity_pid_.resetIntegral();
  }

  if (fabsf(position_error) < 0.1f)
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

  const float speed_scale = clampf(fabsf(speed_rps) / (adaptive_config_.max_speed_rps + 1.0e-6f), 0.0f, 1.0f);
  const float accel_scale = clampf(fabsf(acceleration_rps2) / (adaptive_config_.acceleration_rps2 + 1.0e-6f), 0.0f, 1.0f);
  const float error_scale = clampf(fabsf(follow_error) / (adaptive_config_.target_follow_error + 1.0e-6f), 0.0f, 1.0f);
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

int32_t ClosedLoopController::getPositionSteps() const
{
  return actual_step_;
}

float ClosedLoopController::getFollowError() const
{
  return follow_error_;
}
