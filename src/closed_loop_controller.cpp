#include "closed_loop_controller.h"
#include "at32f403a_407_board.h"
#include "pulse_capture.h"
#include "step_sequencer.h"

#define STEP_PERIOD_US_DEFAULT  5000U
#define FULL_STEPS_PER_ROUND    200U  // 1.8度步进
#define DEFAULT_MICROSTEPS      64U
#define DEFAULT_STEP_PULSE_NS   2000ULL
// 脉宽的硬上下限已改为 ClosedLoopController::kMinStepPulseNs / kMaxStepPulseNs，
// 因为 ParameterRouter 做 clamp 时也要用同一组值（避开两份宏各自漂移）。

// STEP 时序三档模式与谐波注入参数的完整说明已搬到 step_sequencer.h，
// ARR 序列的生成与 TMR2/DMA 启动已搬到 step_sequencer / step_output_channel。
// 本文件不再直接看 STEP_SEQ_MODE。

// ===========================================================================
// 运动收尾的沉降确认（RAMP_STAGE_SETTLE）
// ---------------------------------------------------------------------------
// 问题：“本窗最后一批脉冲已经交给 TMR2/DMA” ≠ “电机轴已经走到位”。
//   1) 硬件侧：脉冲还在 DMA 序列里排队，最后一个窗口要 50us 才能发完；
//   2) 机械侧：电机轴经过负载/皮带总比命令脉冲滞后一个负载角。
// 原来在“计步数达到总数”的那一拍就直接 stopMotion()，它内部会立刻
// stepper_stop_motion_timer() 关掉 TMR2 + DMA，把尾巴上还没发完的几十个脉冲
// 直接砍掉 —— 高转速1200rpm下一个窗口就 50 多步 ≈ 0.36°，于是“总是少一点点”。
// 现在改成：步数发完后先进入沉降阶段，等最后一批脉冲真的发完 + 编码器连续
// 若干拍不动，再判定运动结束。这段时间的编码器行程照样计入统计。
// ===========================================================================
// 最短沉降时间：必须大于一个控制窗口(50us) 加上若干机械时间常数。
// 给 15ms 是因为负载角回位/皮带回弹通常在 10ms 量级，太短会在轴还在动时就收尾。
#ifndef STEP_SETTLE_MIN_US
#define STEP_SETTLE_MIN_US 15000U
#endif

// 编码器连续多少拍“不动”才算真正静止（一拍 = 20kHz = 50us）
#ifndef STEP_SETTLE_STABLE_RAMPS
#define STEP_SETTLE_STABLE_RAMPS 8U
#endif

// 多少 LSB 之内算“不动”。滤波后的原始角在跳变沿上会来回抹 1~2 LSB，
// 卡到 1 LSB 的话可能永远凑不满连续 8 拍，反而每次都要等满超时。
// 2 LSB = 0.011°，比机械负载角小两个数量级，不会把“还在爬行”误判成“已静止”。
#ifndef STEP_SETTLE_QUIET_LSB
#define STEP_SETTLE_QUIET_LSB 2
#endif

// 沉降超时：轴一直因惯性/共振轻微抖动时也不能永久挂着，到点强制收尾。
// 这个值就是“最坏情况下每次运动多花的时间”，所以别给太大。
#ifndef STEP_SETTLE_TIMEOUT_US
#define STEP_SETTLE_TIMEOUT_US 200000U
#endif

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

    // DWT 时间戳的溢出跟踪状态。只有这一个翻译单元持有，不再每个 TU 一份。
    uint32_t g_dwt_overflow_count = 0;
    uint32_t g_last_dwt_value = 0;
    bool g_dwt_initialized = false;
}

// DWT 高精度时间戳（头文件里只留声明，理由见 closed_loop_controller.h）。
uint64_t get_hw_time_ns(void)
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
    return total_cycles * ns_per_cycle;
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

ClosedLoopController::ClosedLoopController()
        : driver_(nullptr), encoder_(nullptr), protocol_(nullptr), load_angle_point_index_(0U),
            load_angle_point_speed_rpm_(0U), load_angle_point_mdeg_(0), load_angle_query_speed_rpm_(0U),
            load_angle_fit_saved_(false),
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
    motion_steps_emitted_(0U),
    motion_leg_pulse_count_(0U), motion_step_accumulator_(0.0f), motion_direction_(1),
    motion_leg_reversed_(false), motion_direction_change_pending_(false),
      motion_step_high_(false), step_pulse_width_ns_(DEFAULT_STEP_PULSE_NS),
      step_period_us_(STEP_PERIOD_US_DEFAULT), encoder_zero_(0U), encoder_filtered_angle_(0.0F), encoder_filtered_raw_(0),
      magnetic_field_high_(false), magnetic_field_low_(false), last_process_time_us_(0U),
      last_step_state_(0U), last_dir_state_(0U), last_en_state_(0U),
    phase_a_current_a_(0.0f),
    phase_b_current_a_(0.0f), cpu_usage_centi_percent_(0U), control_task_usage_centi_percent_(0U),
    calibration_lookup_raw_value_(0U), calibration_lookup_corrected_value_(0U),
    calibration_stage_(CALIBRATION_IDLE), calibration_index_(0U),
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
    // PB4(DIR)/PB5(PUL) 不做软件轮询，由 pulse_capture_init() 配置成 TMR3 复用输入，
    // 硬件捕获 + DMA 逐脉冲快照，避免 DIR 在 rampUpdate 里采样时已经翻转。
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

    load_angle_fit_saved_ = load_angle_.loadFromFlash();
    // 【诊断】负载角表到底有没有标定过：
    //   fitted=0 -> 从未标定（或 Flash 里没有有效表）-> 负载角前馈和 0x151 遥测恒为 0
    //   fitted=1 -> 已加载，看 coeff/intercept/exp 是否合理
    printf("[LOAD_ANGLE] fitted=%d coeff=%.6f intercept=%.4f exp=%.4f\r\n",
           load_angle_.isFitted() ? 1 : 0,
           static_cast<double>(load_angle_.powerCoefficient()),
           static_cast<double>(load_angle_.interceptDeg()),
           static_cast<double>(load_angle_.exponent()));

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
    fault_monitor_.setPolicy(stop_on_encoder_fault, stop_on_magnetic_fault);
}

void ClosedLoopController::reportEncoderFault(bool active)
{
    fault_monitor_.reportEncoderFault(active);
}

void ClosedLoopController::reportMagneticFieldAlarm(bool active)
{
    fault_monitor_.reportMagneticFieldAlarm(active);
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

    // 本拍的档位在这里消费掉，消费完立刻清空：
    // stopMotion / enableExternalPulseFollow 这些非节拍路径也会调 syncProtocolTelemetry，
    // 它们拿到的就是 kNone，不会把已经刷过的档位再刷一遍。
    const TelemetryScheduler::Slot slot = telemetry_slot_;
    telemetry_slot_ = TelemetryScheduler::Slot::kNone;

    // 具体刷哪些寄存器见 src/telemetry_reporter.cpp（94 行寄存器字典已搬过去）。
    telemetry_reporter_.report(slot);
}

bool ClosedLoopController::writeParameter(uint16_t reg, uint32_t value)
{
    // 寄存器映射表已搬到 ParameterRouter（friend，可直接读写控制器状态）。
    // 具体分支见 src/parameter_router.cpp。
    return parameter_router_.write(reg, value);
}

bool ClosedLoopController::readParameter(uint16_t reg, uint32_t* value)
{
    // 读取映射表同样已搬到 ParameterRouter，具体分支见 src/parameter_router.cpp。
    return parameter_router_.read(reg, value);
}

void ClosedLoopController::updateLoopFrequencyStats(uint64_t time_ns)
{
    // 频率统计已搬到 LoopStats（include/loop_stats.h）：
    // 12 个成员 + 500ms 窗口平均算法都在那边，这里只转发一下。
    loop_stats_.update(time_ns);
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

    if (external_pulse_follower_.isEnabled())
    {
        printf("[EXT_FOLLOW] setTargetAngleDeg ignored: external pulse follow is active\r\n");
        return;
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
    compensated_motion_step_accumulator_ = 0.0f;
    compensated_rps_ = 0.0f;
    motion_encoder_previous_raw_ = encoder_ != nullptr ? encoder_->readFilteredRaw() : encoder_filtered_raw_;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
    if (driver_ != nullptr)
    {
        driver_->setEnable(true);
    }
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
    // 跟随模式下不接受规划运动：命令位置完全由外部脉冲决定，两套逻辑同时跑必然打架
    if (external_pulse_follower_.isEnabled())
    {
        printf("[EXT_FOLLOW] startMotion ignored: external pulse follow is active\r\n");
        return;
    }

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
    compensated_motion_step_accumulator_ = 0.0f;
    compensated_rps_ = 0.0f;
    motion_step_high_ = false;
    motion_zero_speed_recovery_logged_ = false;
    motion_leg_reversed_ = false;
    motion_direction_change_pending_ = true;
    motion_commanded_travel_deg_ = 0.0f;
    motion_encoder_travel_raw_ = 0;
    motion_follow_error_deg_ = 0.0f;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
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

    if (driver_ != nullptr && !fault_monitor_.encoderFaultActive() && !fault_monitor_.magneticFaultActive())
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
        printf("positionPID: %f,%f,%f\n", position_pid_.kp, position_pid_.ki, position_pid_.kd);
        printf("velocityPID: %f,%f,%f\n", velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd);

        // 关键标记：剩余步数 <= steps_to_decel_ 就进入减速阶段
        steps_to_decel_ = decel_total_steps_;
        current_step_speed_ = motion_start_step_s_; // 初始速度

        // 复位步累积器，用于固定频率定时器里的脉冲生成（经典DDA微分器思路）
        motion_step_accumulator_ = 0.0f;
        compensated_motion_step_accumulator_ = 0.0f;
        compensated_rps_ = 0.0f;
        encoder_filtered_angle_ = 0.0f;
        // ===================================================================

        max_step_per_ramp = 0;

        // 全部参数准备好再开始
        motion_first_run_ = true;
        motion_running_ = true;
    }
}

// ARR 序列生成（generate_step_sequence / add_dir_to_step_sequence / 谐波相位）
// 已整体搬到 step_sequencer.h / step_sequencer.cpp。
// TMR2 + DMA1_CH2 的序列装载与启动已搬到 StepOutputChannel。

// ===================== 【rampUpdate 分段实现】=====================
// 原来 rampUpdate 一个函数 640 行，里面同时住着"外部跟随 / 角度模式 / 梯形规划 / 校准修饰"
// 四套逻辑，从任意一行都看不出当前是哪种模式在跑。
// Phase 0 先把三段边界最清晰的抽出来（执行顺序、读写顺序与拆分前逐行等价）：
//   collectExternalPulseCommand  : TMR3 采集 → 跟随命令累加 → 窗内换向统计
//   refreshEncoderFeedback       : 编码器快照 → 跟随误差 → 已发完步数记账
//   updateExternalPulseChannel   : 外部跟随模式的整拍输出
// 后续阶段会把这三段整体搬进 ExternalPulseFollower。

void ClosedLoopController::collectExternalPulseCommand()
{
    // 外部 PUL/DIR 指令采集：DMA 只搬原始快照，这里一次性批量解析本轮的脉冲。
    // 采集用 TMR3（PB4/PB5），内部 STEP 输出用 TMR2（PB10/PB11），互不干扰。
    // abs 是本窗脉冲的绝对值和，只用来判断"窗内有没有换向"，不参与任何输出。
    int32_t external_pulse_abs = 0;
    external_pulse_delta_ = pulse_capture_read_delta_ex(&external_pulse_abs);

    if (!external_pulse_follower_.isEnabled())
    {
        return;
    }

    // 使能后的第一拍要丢弃这一窗的脉冲：那是使能前积压的历史脉冲，
    // 一上位就补发出去会直接飞车。accumulateCommand 内部会顺手清掉 prime 标志。
    const bool prime = external_pulse_follower_.primePending();
    if (prime)
    {
        external_pulse_delta_ = 0;
        external_pulse_abs = 0;
    }
    // 命令位置累加 + 窗内换向统计（abs != |净和| 说明窗内 DIR 翻过）
    external_pulse_follower_.accumulateCommand(external_pulse_delta_, external_pulse_abs, prime);
}

float ClosedLoopController::refreshEncoderFeedback()
{
    // 上一窗口交给 TMR2/DMA 的脉冲，到这一拍已经全部发完了（窗口 50us，下一拍在它之后）。
    // 只有把它并入"已发完步数"，下面的命令位置才能和真实脉冲对齐，
    // 否则跟随误差会凭空多出一个窗口的滞后（高速下约 0.3~0.4°）。
    if (motion_dma_pending_signed != 0)
    {
        motion_steps_delivered_signed += motion_dma_pending_signed;
        motion_dma_pending_signed = 0;
    }

    float encoder_delta_deg = 0.0f;
    if (encoder_ == nullptr)
    {
        return encoder_delta_deg;
    }

    const float previous_encoder_angle = encoder_filtered_angle_;
    encoder_filtered_raw_ = encoder_->readFilteredRaw();
    encoder_filtered_angle_ = encoder_->readFilteredAngle();
    motion_position_deg_ = encoder_filtered_angle_;
    magnetic_field_high_ = encoder_->magneticFieldHigh();
    magnetic_field_low_ = encoder_->magneticFieldLow();
    reportMagneticFieldAlarm(magnetic_field_high_ || magnetic_field_low_);

    encoder_delta_deg = normalize_signed_delta_deg(encoder_filtered_angle_ - previous_encoder_angle);

    if (motion_running_ && motion_encoder_reference_valid_)
    {
        const int16_t encoder_delta_raw = normalize_signed_delta_deg(static_cast<int32_t>(encoder_filtered_raw_) - static_cast<int32_t>(motion_encoder_previous_raw_));
        motion_encoder_travel_raw_ += encoder_delta_raw;
        // 下一步才计算上一步的理论步数（因为用了定时器后，脉冲是异步发出的， 直接算就等于理论步数领先实际了，所以放到下一ramp才计算理论跑了多少
        const float step_angle_deg = 360.0f / static_cast<float>(getMicroStepsPerRound(driver_));
        // 用"已确认发完"的步数，而不是"已交给 DMA"的步数，命令位置才和真实脉冲同步
        motion_commanded_travel_deg_ = static_cast<float>(motion_steps_delivered_signed) * step_angle_deg;

        auto motion_encoder_travel_angle_ = static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F;
        motion_follow_error_deg_ = normalize_signed_angle_error_deg(
            motion_encoder_travel_angle_ - motion_commanded_travel_deg_);  // 比如命令要走3度，结果才走了2度，那跟随误差就是-1度。实际-目标
    }
    motion_encoder_previous_raw_ = encoder_filtered_raw_;

    return encoder_delta_deg;
}

void ClosedLoopController::updateExternalPulseChannel(uint64_t now_ns, float encoder_delta_deg)
{
    // 注意：delta_ramp_ns 在主路径稍后才算，这里自己取一次，
    // 并限幅——首拍或长时间没进 rampUpdate 时不能拿一个巨量时间去补发。
    const uint64_t external_delta_ns = now_ns - motion_last_ramp_time_ns_;
    // 跟随分支直接 return 了，主路径里那段更新 encoder_speed_rpm_ 的代码跑不到，
    // 但闭环补偿的速度环要的就是它，所以在这里补一次（同一个 0.9/0.1 平滑）。
    if (external_delta_ns > 0U && external_delta_ns <= 1000000ULL && encoder_ != nullptr)
    {
        const float dt = static_cast<float>(external_delta_ns) / 1.0e9f;
        const float measured_rpm = (encoder_delta_deg / 360.0f) * (60.0f / dt);
        encoder_speed_rpm_ = encoder_speed_rpm_ * 0.9f + measured_rpm * 0.1f;
    }
    runExternalPulseFollow(external_delta_ns);
    motion_last_ramp_time_ns_ = now_ns;
    syncProtocolTelemetry();
}

void ClosedLoopController::updateSettleStage(uint64_t now_ns, float encoder_delta_deg)
{
    // 沉降阶段也刷新一次实测转速，否则上位机的"即时转速"会停在进入沉降前的旧值上
    const uint64_t settle_dt_ns = now_ns - motion_last_ramp_time_ns_;
    if (settle_dt_ns > 0U && encoder_ != nullptr)
    {
        const float dt = static_cast<float>(settle_dt_ns) / 1.0e9f;
        const float measured_rpm = (encoder_delta_deg / 360.0f) * (60.0f / dt);
        encoder_speed_rpm_ = encoder_speed_rpm_ * 0.9f + measured_rpm * 0.1f;
    }

    const uint64_t settle_elapsed_ns = now_ns - motion_settle_start_ns_;
    const uint64_t settle_min_ns = static_cast<uint64_t>(STEP_SETTLE_MIN_US) * 1000ULL;
    const uint64_t settle_timeout_ns = static_cast<uint64_t>(STEP_SETTLE_TIMEOUT_US) * 1000ULL;

    if (settle_elapsed_ns >= settle_min_ns && encoder_ != nullptr)
    {
        // 编码器原始角差 1 LSB = 0.0055°，滤波后的正常抖动不超过 1~2 LSB
        const int32_t raw_step = normalize_signed_delta_deg(
            static_cast<int32_t>(encoder_filtered_raw_) - static_cast<int32_t>(motion_settle_last_raw_));
        if (raw_step >= -STEP_SETTLE_QUIET_LSB && raw_step <= STEP_SETTLE_QUIET_LSB)
        {
            motion_settle_stable_ramps_++;
        }
        else
        {
            motion_settle_stable_ramps_ = 0U;
        }
        motion_settle_last_raw_ = encoder_filtered_raw_;
    }

    const bool settled = motion_settle_stable_ramps_ >= STEP_SETTLE_STABLE_RAMPS;
    const bool timed_out = settle_elapsed_ns >= settle_timeout_ns;
    if (settled || timed_out)
    {
        // 已经等够了：脉冲发完了，轴也不再动了。现在才允许真停定时器、清零状态。
        printf("[SETTLE] done in %.3f ms (%s), commanded=%.4f deg, encoder=%.4f deg, follow_error=%.4f deg\r\n",
               static_cast<double>(settle_elapsed_ns) / 1.0e6,
               settled ? "encoder quiet" : "timeout",
               static_cast<double>(motion_commanded_travel_deg_),
               static_cast<double>(motion_encoder_travel_raw_) * 360.0 / 65536.0,
               static_cast<double>(motion_follow_error_deg_));
        motion_running_ = false;
        motion_ramp_stage_ = RAMP_STAGE_DONE;
        current_step_speed_ = 0.0f;
        stopMotion();
        syncProtocolTelemetry();
        return;
    }

    motion_last_ramp_time_ns_ = now_ns;
    syncProtocolTelemetry();
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
    // 推进遥测节拍（1Hz / 100Hz / 本拍不刷），结果留给 syncProtocolTelemetry 消费。
    telemetry_slot_ = telemetry_scheduler_.tick();

    // 更新FOC频率信息
    updateLoopFrequencyStats(now_ns);

    // ① TMR3 采集 + 跟随命令累加（含窗内换向统计）。
    //    无论是否开启跟随都要读取：不开跟随时这个 delta 仍要给 [PUL] 自检用。
    collectExternalPulseCommand();

    if (driver_ == nullptr)
    {
        return;
    }

    // ② 编码器快照 + 跟随误差 + 上一窗已发完步数记账
    const float encoder_delta_deg = refreshEncoderFeedback();

    // ③ 校准状态机（每拍驱动；HOMING/MOVING 阶段会反过来修饰速度规划）
    updateEncoderCalibration(encoder_filtered_raw_);

    // ④ 模式分派：外部跟随独占输出通道，与下面的梯形规划互斥。
    //    放在沉降收尾之前：跟随时命令位置完全由外部脉冲决定，
    //    不能进“发满 motion_pulse_count_ 就收尾”那套状态机。
    if (external_pulse_follower_.isEnabled())
    {
        updateExternalPulseChannel(now_ns, encoder_delta_deg);
        return;
    }

    // 沉降阶段不参与 PID 评分：这段误差是在收敛中的尾巴，计进去会把评分刷得很漂亮
    // （结尾误差在变小），反而掩盖真实的中段跟随性能。
    if (motion_running_ && !closed_loop_angle_mode_enabled_ && closed_loop_compensation_enabled_ &&
        motion_ramp_stage_ != RAMP_STAGE_SETTLE)
    {
        const bool latter_half = motion_steps_emitted_ >= motion_pulse_count_ / 2U;
        pid_auto_tuner_.addSample(-motion_follow_error_deg_, latter_half);
    }

    // 如果当前没有运动在运行，直接退出
    if (!motion_running_ && !closed_loop_angle_mode_enabled_)
    {
        syncProtocolTelemetry();
        return;
    }

    // ⑤ 沉降收尾：脉冲发完 ≠ 轴到位
    if (motion_ramp_stage_ == RAMP_STAGE_SETTLE)
    {
        updateSettleStage(now_ns, encoder_delta_deg);
        return;
    }

    if (motion_first_run_)
    {
        motion_last_ramp_time_ns_ = now_ns;
        // printf("fuck0 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
        motion_first_run_ = false;
    }

    // 获取两次rampUpdate生成之间真实流逝的纳秒
    uint64_t delta_ramp_ns = now_ns - motion_last_ramp_time_ns_;
    if (delta_ramp_ns > 0U && encoder_ != nullptr)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        const float measured_rpm = (encoder_delta_deg / 360.0f) * (60.0f / dt);
        // 平滑计算电机的实际转速
        encoder_speed_rpm_ = encoder_speed_rpm_ * 0.9f + measured_rpm * 0.1f;
        // encoder_speed_rpm_ = measured_rpm;
    }

    // 【负载角前馈】实时计算，供遥测（0x151）、网页「误差不计负载角」和下面的补偿段共用。
    // 必须每拍都算：补偿段只在 closed_loop_compensation_enabled_ 为真时执行，
    // 如果把计算放在那里，用户不勾闭环补偿时网页的负载角就永远是 0。
    // 代价：每拍一次 powf（约 100 周期 = 0.4us），且只有标定过负载角表才执行。
    motion_load_angle_deg_ = 0.0f;
    if (load_angle_.isFitted())
    {
        const float load_speed_abs = fast_abs(encoder_speed_rpm_);
        const auto load_speed_u16 = static_cast<uint16_t>(
            load_speed_abs > 65535.0f ? 65535.0f : load_speed_abs);
        motion_load_angle_deg_ =
            static_cast<float>(load_angle_.getLoadAngleMilliDegrees(load_speed_u16)) / 1000.0f;
    }

    if (closed_loop_angle_mode_enabled_)
    {
        // 角度模式已搬到 MotionPlanner（阶段 A）。
        // 位置环 -> 速度环 -> 限速 -> 加减速限制 -> current_step_speed_，
        // 具体实现和注释见 src/motion_planner.cpp。
        motion_planner_.updateAngleMode(delta_ramp_ns);
    }
    else if (motion_planner_.updateTrapezoid(delta_ramp_ns, now_ns))
    {
        // 返程换向/运动结束/进入沉降：本拍已经处理完，不再跑后面的补偿和 DDA。
        // 对应原代码里那两个 return。
        return;
    }

    if (closed_loop_compensation_enabled_ && motion_running_ && !closed_loop_angle_mode_enabled_ &&
        encoder_ != nullptr && delta_ramp_ns > 0U && motion_max_step_s_ > 0.0f)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;

        // 【阶段 2：负载角前馈】电机轴永远落后于命令脉冲，这个滞后角就是负载角，
        // 是物理必然，不应该被 PID 当成误差去“追”——追不上会让积分项无限累积，
        // 最后表现为脉冲暴增、电机停摆。
        //
        // 做法：用 load_angle_ 表（按速度拟合的幂律模型）算出“理论上应该落后多少”，
        // 从跟随误差里减掉，PID 只处理“超出预期的偏差”。
        //
        // 符号约定：
        //   motion_follow_error_deg_ = 实际 - 命令 → 落后时为负数
        //   getLoadAngleMilliDegrees() 返回正值（落后多少度）
        //   所以稳态时 follow_error + load_angle ≈ 0，effective_error 就是残差。
        // 负载角已在上面每拍算好（motion_load_angle_deg_），这里直接取用；
        // 符号：motion_follow_error_deg_ = 实际 - 命令（落后时为负），
        //       motion_load_angle_deg_ 为正值（落后多少度）。
        const float load_angle_deg = motion_load_angle_deg_;
        const float effective_error_deg = motion_follow_error_deg_ + load_angle_deg;

        // 残差死区：负载角模型不可能完全准（拟合误差 + 机械差异），
        // 剩下的零点几度不该让积分项持续爬升。死区外的部分才送进 PID。
        constexpr float kResidualDeadbandDeg = 0.05f;
        float pid_error_deg = 0.0f;
        if (fast_abs(effective_error_deg) >= kResidualDeadbandDeg)
        {
            pid_error_deg = effective_error_deg -
                (effective_error_deg > 0.0f ? kResidualDeadbandDeg : -kResidualDeadbandDeg);
        }

        // 位置跟随误差为正时表示实际超前，应该减速；取反后送入位置 PID。
        const float position_reference_rps = position_pid_.update(-pid_error_deg / 360.0f, dt);
        const float measured_motion_rps = encoder_speed_rpm_ / 60.0f;
        // 总期望速度 = 基础速度 + 位置环修正量
        // 【修复 BUG 1】原来是 static const，只初始化一次：
        // 改细分（32→64→128）后补偿量的换算比例就永远停在第一次那个值上。
        const uint32_t micro_steps_per_round = getStepsPerRound();
        const float base_speed_rps = current_step_speed_ / static_cast<float>(micro_steps_per_round);
        const float total_ref_rps = base_speed_rps + position_reference_rps;
        const float velocity_error_rps = total_ref_rps - measured_motion_rps;
        const float velocity_correction_rps = velocity_pid_.update(velocity_error_rps, dt);
        const float max_rps = motion_max_rpm_ / 60.0f;

#ifdef CLAMP_FOC_SPEED
        const float unconstrained_rps = fast_clamp(velocity_correction_rps,
                                                               -max_rps,
                                                               max_rps);
        const float directed_rps = unconstrained_rps * target_direction < 0.0f
            ? 0.0f
            : unconstrained_rps;
        const float accel_limit_rps2 = angle_accel_rpm_s_ / 60.0f;
        const float previous_rps = motion_speed_rpm_ / 60.0f;
        const float max_delta_rps = accel_limit_rps2 * dt;
        const float desired_rps = fast_clamp(directed_rps,
                                             previous_rps - max_delta_rps,
                                             previous_rps + max_delta_rps);
#else
#endif

        // if (now_ns % 1000 == 0)
        //     printf("motion_follow_error_deg_: %f, position_reference_rps: %f, "
        //            "measured_motion_rps: %f, commanded_speed_rps: %f, velocity_error_rps: %f, velocity_correction_rps: %f "
        //            "added_rps: %f, "
        //            // "correction_rps: %f, correction_step_s: %f, max_correction_step_s: %f, "
        //            "limited_correction_step_s:%f, "
        //            "current_step_speed_:%f \n",
        //            motion_follow_error_deg_, position_reference_rps,
        //            measured_motion_rps, base_speed_rps, velocity_error_rps, velocity_correction_rps,
        //            added_rps,
        //            // correction_rps, correction_step_s, max_correction_step_s,
        //            limited_correction_step_s,
        //            current_step_speed_);

        // current_step_speed_ = fast_clamp(current_step_speed_ + limited_correction_step_s,
        //                                  0.0f,
        //                                  motion_max_step_s_ * 2);
        // current_step_speed_ = current_step_speed_ + velocity_correction_rps * target_direction * static_cast<float>(micro_steps_per_round);


        // const float unconstrained_rps = fast_clamp(velocity_correction_rps,
        //                                            -max_rps,
        //                                            max_rps);

        // TODO 命令的目标角度本身就有问题，先修复那个
        // const float directed_rps =  velocity_correction_rps * target_direction < 0.0f
        //     ? 0.0f
        //     : velocity_correction_rps;
        // 【修复 BUG 2】位置环的贡献已经体现在 total_ref_rps 里，速度环输出是
        // “相对这个参考的修正量”。原来最后又把 position_reference_rps 加了一遍，
        // 等于位置环增益被放大一倍 —— 这就是“稍微调大一点 Kp 就无限超调”的来源。
        //
        // 【修复 BUG 3】去掉了 target_direction 乘法。position_reference_rps 和
        // velocity_correction_rps 本身都带符号（误差为正就输出为正），再乘一个 ±1
        // 会让结果永远为正 —— 只能加速不能减速，误差反向时更会失控。
        const float compensated_rps = position_reference_rps + velocity_correction_rps;

        // 【修复 BUG 4】必须限幅。原来 compensated_rps_ = directed_rps 没有任何上限，
        // PID 一冲高就是每窗几十上百步，电机脉冲太多直接停摆。
        // max_rps 已在上面算好，限到“电机最高转速”之内，再叠一层加速度限制。
        const float limited_rps = fast_clamp(compensated_rps, -max_rps, max_rps);
        const float max_delta_rps = (angle_accel_rpm_s_ / 60.0f) * dt;
        const float previous_rps = compensated_rps_;
        compensated_rps_ = fast_clamp(limited_rps, previous_rps - max_delta_rps, previous_rps + max_delta_rps);
    }

    // 剩余部分（转速反算 + DDA 累加器 + 脉冲出口）已整体搬到 MotionPlanner（阶段 C）。
    // 返回 true = 出口拒绍（窗口连头尾 guard 都放不下）：本窗不发，时间戳已由内部前移，
    // 累加器里的步数留到下一拍慢慢补。
    if (motion_planner_.emitSteps(delta_ramp_ns, now_ns))
    {
        return;
    }

    // 更新本次ramp的时间戳，作为下一次调用的“上一次时间点”
    motion_last_ramp_time_ns_ = now_ns;
    syncProtocolTelemetry();
}

void ClosedLoopController::stopMotion()
{
    // 【调试打印，默认关闭】
    // 这两行带 %f，115200 波特下输出约 100 字节 ≈ 7ms。
    // stopMotion() 的调用点包括：每次运动沉降完成（就在 control_task 里）、
    // 每次关闭外部跟随、使能/失能驱动、故障停机 —— 也就是说每跑完一次运动，
    // 控制环都会在这里停摆 7ms，跟随时积压会瞬间涨 700+ 步。
    // 需要看这两个量时把下面这行取消注释重新编译。
    // #define DEBUG_MOTION_STOP_TRACE 1
#ifdef DEBUG_MOTION_STOP_TRACE
    printf("max_step_per_ramp: %lu\n", static_cast<unsigned long>(max_step_per_ramp));
    printf("motion stopping!!! motion_commanded_travel_deg_: %f, motion_encoder_travel_deg_: %f\n", motion_commanded_travel_deg_, static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F);
#endif
    motion_running_ = false;
    motion_first_run_ = false;
    motion_paused_ = false;
    motion_speed_rpm_ = 0.0f;
    encoder_speed_rpm_ = 0.0f;
    target_velocity_rps_ = 0.0f;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0U;
    motion_steps_delivered_signed = 0;
    motion_dma_pending_signed = 0;
    motion_settle_start_ns_ = 0ULL;
    motion_settle_stable_ramps_ = 0U;
    motion_commanded_travel_deg_ = 0;
    motion_encoder_travel_raw_ = 0U;
    motion_leg_pulse_count_ = 0U;
    motion_leg_reversed_ = false;
    motion_direction_change_pending_ = false;
    motion_step_accumulator_ = 0.0f;
    compensated_motion_step_accumulator_ = 0.0f;
    // 【必须清零】这三个都是有“记忆”的状态，不清理会让下一次启动从错误速度起跑：
    //
    // compensated_rps_（补偿速度 rps）：自从加了加速度限制后，它每拍的值是
    //   clamp(目标, 上一拍 ± accel*dt) —— 变成了带状态的东西。
    //   运动中按停止时它保留着停止那一刻的补偿量，再启动时规划速度从 0 开始，
    //   但这个残留值仍然叠加在 current_step_speed_ 上，且需要好几拍才能衰减掉，
    //   表现为“以停止时的速度启动”。旧代码每拍直接重算（无限幅）所以没这个残留。
    //
    // current_step_speed_（微步/s）：startMotion() 只重算 motion_start_step_s_ /
    //   motion_max_step_s_ / steps_to_decel_，完全不会摸这个值。
    //
    // motion_ramp_stage_：如果上次停在 DECEL，下次启动第一拍会先减速而不是从 0 加速。
    compensated_rps_ = 0.0f;
    current_step_speed_ = 0.0f;
    motion_ramp_stage_ = RAMP_STAGE_ACCEL;
    motion_last_ramp_time_ns_ = 0ULL;
    // printf("fuck3 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
    motion_step_high_ = false;
    step_sequencer::resetHarmonicPhase();  // 下一局运动重新从 0 相位开始注入
    if (driver_ != nullptr)
    {
        step_output_.stop();
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
    loop_stats_.setEnabled(enable);
}

void ClosedLoopController::getLoopFrequencyStats(LoopFrequencyStats* stats) const
{
    if (stats == nullptr)
    {
        return;
    }

    stats->position_loop_hz = loop_stats_.positionLoopHz();
    stats->velocity_loop_hz = loop_stats_.velocityLoopHz();
    stats->current_loop_hz = loop_stats_.currentLoopHz();
    stats->position_samples = loop_stats_.positionSamples();
    stats->velocity_samples = loop_stats_.velocitySamples();
    stats->current_samples = loop_stats_.currentSamples();
}

void ClosedLoopController::resetLoopFrequencyStats()
{
    loop_stats_.reset();
}

int32_t ClosedLoopController::getPositionSteps() const
{
    return actual_step_;
}

int32_t ClosedLoopController::getExternalPulseDelta() const
{
    return external_pulse_delta_;
}

uint32_t ClosedLoopController::getStepsPerRound() const
{
    return getMicroStepsPerRound(driver_);
}

// ===================== 【外部 PUL/DIR 跟随】=====================
// 命令位置 = 外部脉冲累计（1 脉冲 = 1 微步），本窗把“命令 - 已发”的积压按限速重发。
// 走的是和 DDA 完全相同的 ARR 序列出口，区别只在于“本窗该发几步”不再由梯形规划决定。
void ClosedLoopController::runExternalPulseFollow(uint64_t delta_ramp_ns)
{
    // 整段跟随输出已搬到 ExternalPulseFollower::update()（friend，直接读写
    // motion_steps_emitted_signed / dma_pending_signed / direction / speed 以及积压计数）。
    external_pulse_follower_.update(delta_ramp_ns);
}

void ClosedLoopController::enableExternalPulseFollow(bool enable)
{
    if (external_pulse_follower_.isEnabled() == enable)
    {
        return;
    }

    // 切换时先把内部运动状态清干净：跟随和规划共用 DDA/TMR2，
    // 残留的积压或半途的梯形都会在下一拍变成莫名其妙的脉冲。
    motion_running_ = false;
    motion_ramp_stage_ = RAMP_STAGE_DONE;
    closed_loop_angle_mode_enabled_ = false;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0;
    motion_steps_delivered_signed = 0;
    motion_dma_pending_signed = 0;
    motion_commanded_travel_deg_ = 0.0f;
    motion_encoder_travel_raw_ = 0;
    motion_follow_error_deg_ = 0.0f;
    motion_step_accumulator_ = 0.0f;
    compensated_motion_step_accumulator_ = 0.0f;
    compensated_rps_ = 0.0f;
    current_step_speed_ = 0.0f;
    motion_speed_rpm_ = 0.0f;
    motion_direction_change_pending_ = true;
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    resetExternalPulseFollowCounters();

    if (!enable)
    {
        external_pulse_follower_.setEnabled(false);
        external_pulse_follower_.setPrimePending(false);
        stopMotion();
        printf("[EXT_FOLLOW] disabled\r\n");
        return;
    }

    // 脉宽 tick 数：DDA 路径是 startMotion() 里算的，跟随路径没人替它算，
    // 漏了这一句 k_step_pulse_ticks 就是个未初始化值，ARR 序列直接乱掉。
    k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) /
                                   static_cast<float>(stepper_common::target_tick_time));

    motion_encoder_previous_raw_ = encoder_ != nullptr ? encoder_->readFilteredRaw() : encoder_filtered_raw_;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
    // 让跟随误差（编码器行程 - 命令行程）照常计算：0x121 和波形图都靠它
    motion_running_ = true;
    external_pulse_follower_.setPrimePending(true);
    external_pulse_follower_.setEnabled(true);

    if (driver_ != nullptr && !fault_monitor_.encoderFaultActive() && !fault_monitor_.magneticFaultActive())
    {
        driver_->setEnable(true);
        driver_->setDirection(true);
    }
    printf("[EXT_FOLLOW] enabled: 1 pulse = 1 microstep, %lu steps/rev, compensation=%u\r\n",
           static_cast<unsigned long>(getMicroStepsPerRound(driver_)),
           closed_loop_compensation_enabled_ ? 1U : 0U);
}

bool ClosedLoopController::isExternalPulseFollowEnabled() const
{
    return external_pulse_follower_.isEnabled();
}

void ClosedLoopController::setExternalPulseMaxStepRate(uint32_t steps_per_second)
{
    external_pulse_follower_.setMaxStepRate(steps_per_second);
}

uint32_t ClosedLoopController::getExternalPulseMaxStepRate() const
{
    return external_pulse_follower_.maxStepRate();
}

int32_t ClosedLoopController::getExternalCmdSteps() const
{
    return external_pulse_follower_.cmdSteps();
}

int32_t ClosedLoopController::getExternalEmittedSteps() const
{
    return external_pulse_follower_.emittedSteps();
}

uint32_t ClosedLoopController::getExternalBacklogSteps() const
{
    return external_pulse_follower_.backlogSteps();
}

uint32_t ClosedLoopController::getExternalPeakBacklogSteps() const
{
    return external_pulse_follower_.peakBacklogSteps();
}

uint32_t ClosedLoopController::getExternalDroppedSteps() const
{
    return external_pulse_follower_.droppedSteps();
}

uint32_t ClosedLoopController::getExternalWindowReversals() const
{
    return external_pulse_follower_.windowReversals();
}

void ClosedLoopController::resetExternalPulseFollowCounters()
{
    external_pulse_follower_.resetCounters();
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
    return loop_stats_.positionLoopHz();
}

uint32_t ClosedLoopController::getVelocityLoopHz() const
{
    return loop_stats_.velocityLoopHz();
}

uint32_t ClosedLoopController::getCurrentLoopHz() const
{
    return loop_stats_.currentLoopHz();
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
    if (driver_ == nullptr || encoder_ == nullptr || fault_monitor_.encoderFaultActive() || fault_monitor_.magneticFaultActive())
    {
        printf("[CALIBRATION] start rejected: driver/encoder unavailable or fault active\r\n");
        return;
    }

    encoder_->clearCalibration();  // 避免raw读成以前校准表的结果影响新校准

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
    printf("[CALIBRATION] moving to index: %u from angle: %f\r\n", index, encoder_->readFilteredAngle());
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

void ClosedLoopController::updateEncoderCalibration(const uint16_t raw)
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
    } else
    {
        printf("[CALIBRATION] invalid encoder sample: %d at index=%u, attempt=%u\r\n",
            raw,
            static_cast<unsigned>(calibration_index_),
            static_cast<unsigned>(calibration_sample_attempts_));
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
