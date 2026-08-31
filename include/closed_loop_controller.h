#ifndef CLOSED_LOOP_CONTROLLER_H
#define CLOSED_LOOP_CONTROLLER_H

#include <stdint.h>

#include "angle_encoder.h"
#include "closed_loop_protocol.h"
#include "stepper_driver.h"

#ifndef USE_HARD_FLOAT_ACCELERATION
#define USE_HARD_FLOAT_ACCELERATION 1
#endif

// 自适应 PID 的基础参数配置。
// 它描述了在不同速度和加速度区间下，系统对跟随误差的容忍度与增益调整幅度。
struct PidAutoTuneConfig
{
  float min_speed_rps;
  float max_speed_rps;
  float acceleration_rps2;
  float target_follow_error;
  float gain_step;
};

// PidController 是一个通用 PID 控制器。
// 在闭环步进控制中，它同时用于两层控制：
// 1）位置环：控制目标位置和当前编码器位置之间的误差
// 2）速度环：控制速度参考值和真实速度之间的误差
class PidController
{
public:
  PidController();

  // 设定比例、积分、微分三个增益参数。
  void setGains(float kp, float ki, float kd);

  // 计算当前误差下的控制输出。
  // error：当前误差，dt：控制周期时间
  float update(float error, float dt);

  // 清零积分项，防止误差长期累积导致超调。
  void resetIntegral();

  // 清零微分项历史值，避免误差突变导致抖动。
  void resetDeriv();

  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float max_integral;
  float max_output;
};

// 采样周期统计结构，用于评估控制器循环运行频率。
struct LoopFrequencyStats
{
  uint32_t position_loop_hz;
  uint32_t velocity_loop_hz;
  uint32_t current_loop_hz;
  uint32_t position_samples;
  uint32_t velocity_samples;
  uint32_t current_samples;
};

// ClosedLoopController 是整个闭环步进控制器的核心。
// 它负责连接：
// - 驱动器底层：实际输出步进信号和电流配置
// - 编码器：读取位置和速度反馈
// - 协议适配器：对外呈现 TMC 兼容调参接口
// 在控制上，它采用“位置环 -> 速度参考 -> 速度环 -> 步进修正”的双环结构。
class ClosedLoopController
{
public:
  ClosedLoopController();

  // 绑定驱动器与编码器，并初始化 PID 与零点偏移。
  void init(StepperDriver *driver, AngleEncoder *encoder);

  // 绑定协议适配器，允许外部通过 TMC 风格寄存器访问闭环参数。
  void setProtocol(ClosedLoopDriverProtocol *protocol);

  // 对外写入指定寄存器参数。
  bool writeParameter(uint8_t reg, uint32_t value);

  // 对外读取指定寄存器参数。
  bool readParameter(uint8_t reg, uint32_t *value);

  // 读取外部步进方向/使能/步进状态，并同步到闭环控制器状态机。
  void syncStepDirection();

  // 核心控制循环，负责读取编码器、计算误差、更新两层 PID，并输出修正。
  void process(uint32_t time_us);

  // 设置目标位置步数。
  void setTargetStep(int32_t target_step);

  // 设置目标速度（转/秒）。
  void setTargetVelocity(float rps);

  // 统一设置基本 PID 参数。
  void setPid(float kp, float ki, float kd);

  // 启用/关闭自适应增益调节。
  void enableAdaptivePid(bool enable);

  // 设置自适应 PID 的参数阈值。
  void setAdaptivePidConfig(const PidAutoTuneConfig &config);

  // 根据当前速度、加速度和跟随误差动态修正 PID 参数。
  void updateAdaptivePid(float speed_rps, float acceleration_rps2, float follow_error);

  // 对编码器执行校准，更新零点偏差。
  void calibrateEncoder(const EncoderCalibrationConfig &config);

  // 开启/关闭循环频率统计功能。
  void enableLoopStats(bool enable);

  // 获取控制环运行频率统计。
  void getLoopFrequencyStats(LoopFrequencyStats *stats) const;

  // 清空 frequency stats。
  void resetLoopFrequencyStats();

  // 读取当前闭环位置步数。
  int32_t getPositionSteps() const;

  // 返回当前位置与目标之间的跟随误差。
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
