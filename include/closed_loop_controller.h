#ifndef CLOSED_LOOP_CONTROLLER_H
#define CLOSED_LOOP_CONTROLLER_H

#include <stdint.h>

#include "angle_encoder.h"
#include "closed_loop_protocol.h"
#include "stepper_driver.h"

#ifndef USE_HARD_FLOAT_ACCELERATION
#define USE_HARD_FLOAT_ACCELERATION 1
#endif

struct PidAutoTuneConfig
{
  float min_speed_rps;
  float max_speed_rps;
  float acceleration_rps2;
  float target_follow_error;
  float gain_step;
};

class PidController
{
public:
  PidController();
  void setGains(float kp, float ki, float kd);
  float update(float error, float dt);
  void resetIntegral();
  void resetDeriv();

  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float max_integral;
  float max_output;
};

struct LoopFrequencyStats
{
  uint32_t position_loop_hz;
  uint32_t velocity_loop_hz;
  uint32_t current_loop_hz;
  uint32_t position_samples;
  uint32_t velocity_samples;
  uint32_t current_samples;
};

class ClosedLoopController
{
public:
  ClosedLoopController();

  void init(StepperDriver *driver, AngleEncoder *encoder);
  void setProtocol(ClosedLoopDriverProtocol *protocol);
  bool writeParameter(uint8_t reg, uint32_t value);
  bool readParameter(uint8_t reg, uint32_t *value);
  void syncStepDirection();
  void process(uint32_t time_us);
  void setTargetStep(int32_t target_step);
  void setTargetVelocity(float rps);
  void setPid(float kp, float ki, float kd);
  void enableAdaptivePid(bool enable);
  void setAdaptivePidConfig(const PidAutoTuneConfig &config);
  void updateAdaptivePid(float speed_rps, float acceleration_rps2, float follow_error);
  void calibrateEncoder(const EncoderCalibrationConfig &config);
  void enableLoopStats(bool enable);
  void getLoopFrequencyStats(LoopFrequencyStats *stats) const;
  void resetLoopFrequencyStats();
  int32_t getPositionSteps() const;
  float getFollowError() const;

private:
  void updateLoopFrequencyStats(uint32_t time_us);
  StepperDriver *driver_;
  AngleEncoder *encoder_;
  ClosedLoopDriverProtocol *protocol_;
  PidController position_pid_;
  PidController velocity_pid_;

  float base_position_kp_;
  float base_position_ki_;
  float base_position_kd_;
  float base_velocity_kp_;
  float base_velocity_ki_;
  float base_velocity_kd_;
  bool adaptive_pid_enabled_;
  PidAutoTuneConfig adaptive_config_;

  volatile int32_t target_step_;
  volatile int32_t actual_step_;
  volatile int32_t last_actual_step_;
  volatile int32_t command_step_;
  volatile float follow_error_;
  volatile float measured_velocity_rps_;
  volatile float target_velocity_rps_;
  volatile uint32_t step_period_us_;
  volatile uint16_t encoder_zero_;
  volatile uint32_t last_process_time_us_;
  volatile uint8_t last_step_state_;
  volatile uint8_t last_dir_state_;
  volatile uint8_t last_en_state_;

  bool loop_stats_enabled_;
  uint32_t last_position_tick_us_;
  uint32_t last_velocity_tick_us_;
  uint32_t last_current_tick_us_;
  uint32_t position_loop_hz_;
  uint32_t velocity_loop_hz_;
  uint32_t current_loop_hz_;
  uint32_t position_samples_;
  uint32_t velocity_samples_;
  uint32_t current_samples_;
};

#endif
