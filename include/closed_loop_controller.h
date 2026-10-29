#ifndef CLOSED_LOOP_CONTROLLER_H
#define CLOSED_LOOP_CONTROLLER_H

#include <cstdint>
#include <cmath>

#include "angle_encoder.h"
#include "closed_loop_protocol.h"
#include "load_angle.h"
#include "pid_auto_tuner.h"
#include "stepper_driver.h"

#ifndef USE_HARD_FLOAT_ACCELERATION
#define USE_HARD_FLOAT_ACCELERATION 1
#endif

#define MAX_PULSE_ARR_LEN 128U  // 20khz也就是50us一个rampupdate，所以每个ramp最多可以跑250个脉冲（算100ns各起伏一次为一个完整脉冲）
// #define CLAMP_FOC_SPEED

#include <cmath>
#include <vector>

#include "core_cm4.h"
#include "pid_controller.h"
// AT32F403A system_core_clock 是内核时钟，例如 240000000UL
// 使用 DWT (Data Watchpoint and Trace) 实现高精度纳秒计时
// CYCCNT 是 32 位寄存器，@250MHz 时约 17.17 秒溢出一次
// 通过跟踪溢出次数，可实现长期高精度时间戳（理论上可运行数千年）

static uint32_t g_dwt_overflow_count = 0;
static uint32_t g_last_dwt_value = 0;
static bool g_dwt_initialized = false;

static inline uint64_t get_hw_time_ns(void)
{
    // 首次初始化 DWT
    if (!g_dwt_initialized)
    {
        // 解锁 DEMCR 寄存器（TRCENA 位）
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        // 清零 CYCCNT 计数器
        DWT->CYCCNT = 0;
        // 使能 CYCCNT 计数器
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        g_dwt_initialized = true;
        g_last_dwt_value = 0;
        g_dwt_overflow_count = 0;
    }

    // 读取当前 DWT cycle 计数（32 位）
    uint32_t current_cc = DWT->CYCCNT;

    // 检测 DWT 溢出：从大值跳变到小值表示发生了溢出
    // 例如：0xFFFFFFFF → 0x00000000
    if (current_cc < g_last_dwt_value)
    {
        g_dwt_overflow_count++;
    }
    g_last_dwt_value = current_cc;

    // 系统核心频率参数
    // @250MHz: 1 cycle = 4ns (10^9 / 250,000,000)
    const uint32_t cpu_freq_hz = system_core_clock;
    const uint64_t ns_per_cycle = 1000000000ULL / (uint64_t)cpu_freq_hz;

    // 计算总周期数：溢出部分 + 当前周期内部分
    // 使用 64 位整数避免溢出：total_cycles = overflow_count * 2^32 + current_cc
    uint64_t total_cycles = ((uint64_t)g_dwt_overflow_count << 32) | (uint64_t)current_cc;

    // 转换为纳秒
    uint64_t time_ns = total_cycles * ns_per_cycle;

    return time_ns;
}

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

    enum FaultFlags : uint32_t
    {
        FAULT_ENCODER_READ_FAILED = 1UL << 0,
        FAULT_MAGNETIC_FIELD_ALARM = 1UL << 1,
        FAULT_OUTPUT_STOPPED = 1UL << 2
    };

    ClosedLoopController();

    // 绑定驱动器与编码器，并初始化 PID 与零点偏移。
    void init(StepperDriver* driver, AngleEncoder* encoder);

    // 配置故障响应策略：读编码器失败或磁场报警时是否立即停止输出。
    void setFaultPolicy(bool stop_on_encoder_fault, bool stop_on_magnetic_fault);

    // 报告编码器读取失败状态，并在设置为失效时执行停输出动作。
    void reportEncoderFault(bool active);

    // 报告磁场过高/过低状态，并在设置为失效时执行停输出动作。
    void reportMagneticFieldAlarm(bool active);

    // 设置 A/B 相电流监控值，供上位机实时显示。
    void setPhaseCurrentTelemetry(float phase_a_a, float phase_b_a);
    void setCpuUsageTelemetry(uint16_t cpu_usage_centi_percent, uint16_t control_task_usage_centi_percent);

    // 将控制器当前故障状态和采样频率同步到协议扩展寄存器表。
    void syncProtocolTelemetry();

    // 绑定协议适配器，允许外部通过 TMC 风格寄存器访问闭环参数。
    void setProtocol(ClosedLoopDriverProtocol* protocol);

    // 对外写入指定寄存器参数。
    bool writeParameter(uint16_t reg, uint32_t value);

    // 对外读取指定寄存器参数。
    bool readParameter(uint16_t reg, uint32_t* value);

    // 设置目标位置步数。
    void setTargetStep(int32_t target_step);

    // 设定目标角度（0~360°），开启基于编码器反馈的闭环位置控制。
    void setTargetAngleDeg(float angle_deg);
    float getTargetAngleErrorDeg() const;

    // 设置目标速度（转/秒）。
    void setTargetVelocity(float rps);

    enum MotionMode : uint32_t
    {
        MOTION_MODE_POSITION_FORWARD = 0U,
        MOTION_MODE_POSITION_REVERSE = 1U,
        MOTION_MODE_VELOCITY_FORWARD = 2U,
        MOTION_MODE_VELOCITY_REVERSE = 3U,
        MOTION_MODE_HOME_FORWARD = 4U,
        MOTION_MODE_HOME_REVERSE = 5U,
        MOTION_MODE_ALTERNATING_FORWARD = 6U,
        MOTION_MODE_ALTERNATING_REVERSE = 7U
    };

    void setMotionConfig(float start_rpm, float max_rpm, float accel_rpm_s, uint32_t pulse_count, MotionMode mode);
    void startMotion();
    void stopMotion();
    void rampUpdate(uint64_t now_ns);
    bool isMotionRunning() const;
    float getMotionSpeedRpm() const;
    float getMotionPositionDeg() const;
    uint32_t getWaveformWindowMs() const;
    void setWaveformWindowMs(uint32_t window_ms);

    // 统一设置基本 PID 参数。
    void setPid(float kp, float ki, float kd);

    // 启用/关闭自适应增益调节。
    void enableAdaptivePid(bool enable);

    // 设置自适应 PID 的参数阈值。
    void setAdaptivePidConfig(const PidAutoTuneConfig& config);

    // 根据当前速度、加速度和跟随误差动态修正 PID 参数。
    void updateAdaptivePid(float speed_rps, float acceleration_rps2, float follow_error);

    // 开启/关闭循环频率统计功能。
    void enableLoopStats(bool enable);

    // 获取控制环运行频率统计。
    void getLoopFrequencyStats(LoopFrequencyStats* stats) const;

    // 清空 frequency stats。
    void resetLoopFrequencyStats();

    // 读取当前闭环位置步数。
    int32_t getPositionSteps() const;

    // 最近一次 rampUpdate 周期内采集到的外部 PUL/DIR 有符号脉冲增量
    // （由 pulse_capture_read_delta() 在 rampUpdate 开头消费）
    int32_t getExternalPulseDelta() const;

    // 每圈微步数 = 200 步/圈 x 驱动器实际微步数（默认 200x64 = 12800）。
    // [PUL] 自检行的 deg、上位机“脉冲转角度”都应该用它，不要再写死 51200。
    uint32_t getStepsPerRound() const;

    // ===================== 【外部 PUL/DIR 脉冲跟随】=====================
    // 开启后，PB4/PB5 采集到的脉冲成为“命令位置”（1 脉冲 = 1 微步），由内部 DDA 重发：
    //   闭环补偿关（0x107=0）→ 开环，按限速把积压发完，1:1 转发
    //   闭环补偿开（0x107=1）→ 在上面叠位置/速度环 PID 修正，用编码器保证实际到位
    // 与 0x11E/0x11F 的规划运动互斥（跟随时 startMotion() 直接忽略）。
    void enableExternalPulseFollow(bool enable);
    bool isExternalPulseFollowEnabled() const;
    // 跟随限速（步/s）。0 = 跟随 angle_max_rpm_ 换算出的能力上限（推荐）。
    void setExternalPulseMaxStepRate(uint32_t steps_per_second);
    uint32_t getExternalPulseMaxStepRate() const;
    int32_t getExternalCmdSteps() const;      // 外部累计命令步（有符号）
    int32_t getExternalEmittedSteps() const;  // 已重发出去（有符号）
    uint32_t getExternalBacklogSteps() const; // 当前积压（命令 - 已发）
    uint32_t getExternalPeakBacklogSteps() const;
    uint32_t getExternalDroppedSteps() const; // 因超限没及时发出的累计步数
    void resetExternalPulseFollowCounters();

    // 读取目标位置步数。
    int32_t getTargetSteps() const;
    float getEncoderZero() const;
    void setEncoderZero(float zero_angle);
    float getEncoderFilteredAngle() const;
    float getEncoderAngleMilliDegrees() const;
    bool isMagneticFieldHigh() const;
    bool isMagneticFieldLow() const;

    // 返回当前位置与目标之间的跟随误差。
    float getFollowError() const;

    // 读取当前相电流和 loop 频率，用于 USB telemetry 回传。
    float getPhaseCurrentTelemetryA() const;
    float getPhaseCurrentTelemetryB() const;
    uint32_t getPositionLoopHz() const;
    uint32_t getVelocityLoopHz() const;
    uint32_t getCurrentLoopHz() const;
    bool readCalibrationTablePair(uint8_t table, uint16_t pair_index, uint32_t* packed_values) const;
    bool readCalibrationTableChecksum(uint32_t* checksum) const;
    uint16_t lookupCorrectedRaw(uint16_t raw_value) const;
    uint16_t lookupOriginalRaw(uint16_t corrected_value) const;

    void calibrateEncoder();

private:
    void updateLoopFrequencyStats(uint64_t time_ns);
    // 外部跟随的脉冲重发通道，在 rampUpdate 里与 DDA 二选一执行
    void runExternalPulseFollow(uint64_t delta_ramp_ns);
    void updateEncoderCalibration(uint16_t raw);
    void finishEncoderCalibration(bool save_table);
    bool startEncoderCalibrationMove(uint16_t index);
    StepperDriver* driver_;
    AngleEncoder* encoder_;
    ClosedLoopDriverProtocol* protocol_;
    LoadAngle load_angle_;
    uint16_t load_angle_point_index_;
    uint16_t load_angle_point_speed_rpm_;
    int32_t load_angle_point_mdeg_;
    uint16_t load_angle_query_speed_rpm_;
    bool load_angle_fit_saved_;
    PidController position_pid_;
    PidController velocity_pid_;
    PidAutoTuner pid_auto_tuner_;

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
    volatile float target_angle_deg_;
    volatile float closed_loop_angle_error_deg_;
    volatile bool closed_loop_angle_mode_enabled_;
    volatile float angle_position_tolerance_deg_;
    volatile float motion_start_rpm_;
    volatile float motion_max_rpm_;
    volatile float motion_accel_rpm_s_;
    volatile float angle_max_rpm_;
    volatile float angle_accel_rpm_s_;
    volatile uint32_t motion_pulse_count_;
    volatile MotionMode motion_mode_;
    volatile bool closed_loop_compensation_enabled_;
    volatile bool motion_running_;
    volatile bool motion_first_run_;
    volatile bool motion_paused_;
    volatile float motion_speed_rpm_;
    volatile float encoder_speed_rpm_;
    volatile float motion_position_deg_;
    volatile float motion_follow_error_deg_;
    volatile float motion_commanded_travel_deg_;
    volatile int64_t motion_encoder_travel_raw_;
    volatile uint16_t motion_encoder_previous_raw_;
    volatile bool motion_encoder_reference_valid_;
    volatile uint64_t motion_last_ramp_time_ns_;
    volatile uint64_t motion_steps_emitted_;
    volatile int32_t motion_steps_emitted_signed;  // 指的是距离总运动位移脉冲数，有方向。等于是出发至今的带方向的累计脉冲数
    // 已经“真的发完”的带方向累计步数，跟随误差必须用它算命令位置。
    // 原因：motion_steps_emitted_signed 在窗口开头就把整窗步数加满了，但脉冲要 50us
    // 才发完（平滑模式），直接相减会让命令位置凭空领先一个窗口 ≈ 几十步 ≈ 0.3°。
    volatile int32_t motion_steps_delivered_signed = 0;
    // 刚交给 TMR2/DMA、还没到下一拍确认的步数（下一个 rampUpdate 才并入 delivered）
    volatile int32_t motion_dma_pending_signed = 0;
    volatile uint32_t motion_dma_pending_steps_;
    volatile uint32_t motion_leg_pulse_count_;
    volatile double motion_step_accumulator_;
    volatile double compensated_motion_step_accumulator_;
    volatile int8_t motion_direction_;
    volatile bool motion_leg_reversed_;
    volatile bool motion_direction_change_pending_;
    volatile bool motion_step_high_;
    volatile uint64_t step_pulse_width_ns_;
    volatile uint32_t k_step_pulse_ticks;
    volatile uint32_t step_period_us_;
    volatile float encoder_zero_;
    volatile float encoder_filtered_angle_;
    volatile uint16_t encoder_filtered_raw_;
    // 磁编码器滑动平均窗口（样本数），0 表示关闭滤波。
    // 上位机写进来的值先记在 encoder_filter_window_，真正的重配放到低速槽位
    // （ramp_loop_idx % 20000 == 0，20kHz 下约 1s 一次）做，避免运动中途改滤波把跟随误差抖出台阶。
    volatile uint16_t encoder_filter_window_ = EncoderFilterConfig::kDefaultWindow;
    volatile uint16_t encoder_filter_window_active_ = EncoderFilterConfig::kDefaultWindow;
    volatile bool magnetic_field_high_;
    volatile bool magnetic_field_low_;
    volatile uint32_t last_process_time_us_;
    volatile uint8_t last_step_state_;
    volatile uint8_t last_dir_state_;
    volatile uint8_t last_en_state_;
    volatile int32_t external_pulse_delta_ = 0;  // 外部 PUL/DIR 指令脉冲增量（每 rampUpdate 刷新）

    // ===== 外部脉冲跟随 =====
    volatile bool external_follow_enabled_ = false;
    // 使能后的第一拍要把使能前积压的历史脉冲丢掉，否则一上来就飞车
    volatile bool external_prime_pending_ = false;
    volatile int32_t external_cmd_steps_ = 0;      // 外部累计命令步（有符号，1 脉冲 = 1 微步）
    volatile int32_t external_emitted_steps_ = 0;  // 已 1:1 转发出去（有符号，不含 PID 修正）
    // PID 修正量不能算进“已转发”，否则积分把多发的步数又当成积压退回，
    // 位置环和积压环会互相推着振荡。这里单独记“转发基准”。
    volatile float external_base_residue_ = 0.0f;  // 低速时不足 1 步的小数余量
    volatile uint32_t external_max_step_s_ = 0U;   // 0 = 用 angle_max_rpm_ 换算的能力上限
    volatile uint32_t external_backlog_steps_ = 0U;
    volatile uint32_t external_backlog_peak_steps_ = 0U;
    volatile uint32_t external_dropped_steps_ = 0U;
    // 外部跟随的 ARR 序列缓冲。不能复用 rampUpdate 里的那份（那是 DDA 专用、
    // 且跟随时 DDA 根本不跑），单独一份 130x4 = 520B，RAM 完全放得下。
    uint32_t external_arr_seq_[MAX_PULSE_ARR_LEN + 2];
    bool stop_on_encoder_fault_;
    bool stop_on_magnetic_fault_;
    bool encoder_fault_active_;
    bool magnetic_fault_active_;
    bool output_stopped_;
    float phase_a_current_a_;
    float phase_b_current_a_;
    volatile uint16_t cpu_usage_centi_percent_;
    volatile uint16_t control_task_usage_centi_percent_;
    uint16_t calibration_lookup_raw_value_;
    uint16_t calibration_lookup_corrected_value_;

    bool loop_stats_enabled_;
    uint64_t last_position_tick_ns_;
    uint64_t last_velocity_tick_ns_;
    uint64_t last_current_tick_ns_;
    uint32_t position_loop_hz_;
    uint32_t velocity_loop_hz_;
    uint32_t current_loop_hz_;
    uint32_t position_samples_;
    uint32_t velocity_samples_;
    uint32_t current_samples_;

    std::vector<float> motion_commanded_travel_deg_list_;
    std::vector<float> motion_encoder_travel_angle_list_;

    float closed_loop_threshold_angle_ = 1.8f;
    float compensated_rps_ = 0.0f;

    uint16_t ramp_loop_idx = 0;

    uint32_t max_step_per_ramp = 0;

    // ====================== 【补充梯形加减速预计算】======================
    // 梯形加减速预计算参数（startMotion里一次性算出）
    float motion_accel_step_s2_; // 加速度：微步/s²
    float motion_start_step_s_; // 起始速度：微步/s
    float motion_max_step_s_; // 最高速度：微步/s
    uint32_t accel_total_steps_; // 加速段总步数
    uint32_t decel_total_steps_; // 减速段总步数
    uint32_t cruise_total_steps_; // 匀速段步数
    enum MotionRampStage
    {
        RAMP_STAGE_ACCEL,
        RAMP_STAGE_CRUISE,
        RAMP_STAGE_DECEL,
        // 脉冲已经全部交给 TMR2/DMA，但电机轴还没追上（负载角）。
        // 这个阶段只做两件事：等最后一批脉冲真的发完、等编码器静止，然后才收尾。
        // 否则 stopMotion() 会立刻关掉 TMR2，把尾巴上那几十个脉冲连同对应角度一起丢掉。
        RAMP_STAGE_SETTLE,
        RAMP_STAGE_DONE
    } motion_ramp_stage_; // 当前阶段：加速/匀速/减速/沉降确认/结束
    volatile uint64_t motion_settle_start_ns_ = 0ULL;   // 进入沉降阶段的时刻
    volatile uint32_t motion_settle_stable_ramps_ = 0U;  // 编码器连续的“不动”拍数
    volatile uint16_t motion_settle_last_raw_ = 0U;      // 上一拍的编码器原始角，用来判静止
    float current_step_speed_; // 当前瞬时速度，微步/s
    uint32_t steps_to_decel_; // 剩余多少步必须开始减速
    bool motion_zero_speed_recovery_logged_;

    enum EncoderCalibrationStage : uint8_t
    {
        CALIBRATION_IDLE,
        CALIBRATION_HOMING,
        CALIBRATION_SAMPLING,
        CALIBRATION_MOVING,
        CALIBRATION_SAVING
    };
    volatile EncoderCalibrationStage calibration_stage_;
    volatile uint16_t calibration_index_;
    volatile uint16_t calibration_sample_count_;
    volatile uint16_t calibration_sample_attempts_;
    volatile int32_t calibration_sample_min_;
    volatile int32_t calibration_sample_max_;
    volatile uint16_t calibration_sample_anchor_;
    volatile int32_t calibration_sample_sum_;
    volatile uint16_t calibration_home_stable_count_;
    volatile uint32_t calibration_home_wait_ticks_;
    float calibration_home_previous_angle_;
    float calibration_saved_start_rpm_;
    float calibration_saved_max_rpm_;
    float calibration_saved_accel_rpm_s_;
    uint32_t calibration_saved_pulse_count_;
    uint64_t calibration_saved_pulse_width_ns_;
    MotionMode calibration_saved_motion_mode_;
    bool calibration_saved_compensation_enabled_;
};

#endif
