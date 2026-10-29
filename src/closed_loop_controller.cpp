#include "closed_loop_controller.h"
#include "at32f403a_407_board.h"
#include "pulse_capture.h"

#define STEP_PERIOD_US_DEFAULT  5000U
#define FULL_STEPS_PER_ROUND    200U  // 1.8度步进
#define DEFAULT_MICROSTEPS      64U
#define MIN_STEP_PULSE_NS       100ULL
#define DEFAULT_STEP_PULSE_NS   2000ULL
#define MAX_STEP_PULSE_NS       20000ULL

// ===========================================================================
// STEP 时序模式（三档，用 STEP_SEQ_MODE 选）
// ---------------------------------------------------------------------------
// 背景：DDA 每 20kHz 窗口只决定“本窗发几步”，脉冲怎么落在窗口内由这里决定。
//
// 0 = 恒速突发（原方案）：窗口开头用 min_step_ticks(400ns) 把本窗 N 步一口气打完。
//     优点：简单、永不丢步；缺点：位置是 20kHz 的硬台阶，谐波能量一直排到几十 kHz，
//     落进机械共振带就会“嗡嗡”响。
// 1 = 平滑铺满：把本窗 N 步均匀铺到整个 50us 窗口，台阶被摊平，位置/速度连续得多。
//     这是降噪的主力，先只开这一档听效果。
// 2 = 平滑 + 谐波注入：在 1 的基础上，把每步间隔乘 (1 + depth*sin(2*pi*f_h*t))，
//     即对速度做正弦调频(FM)，把窄带能量摊到 f_h 两侧的边带上。
//
// 注意 1 和 2 的算法都保证“整窗总时长严格等于窗口预算”（剩余预算法，见
// generate_step_sequence），所以窗口边界不会产生位置漂移 —— 这一点是上一版
// 用 remaining/(n*(1+depth)) 缩放的死穴，那版每窗只用掉约 83% 的时间，边界必跳变。
// ===========================================================================
#ifndef STEP_SEQ_MODE
#if defined(STEP_SEQ_HARMONIC_ENABLE) && (STEP_SEQ_HARMONIC_ENABLE)
#define STEP_SEQ_MODE 2  // 兼容旧的开关写法
#else
#define STEP_SEQ_MODE 1  // 默认平滑：无任何副作用，可直接听/测效果
#endif
#endif

// ---------------------------------------------------------------------------
// 谐波注入参数（仅 STEP_SEQ_MODE == 2 生效）
//
// 1) f_h 不是“步频的谐波”，而是要对准的那条机械共振峰。它是电机+负载的机械属性，
//    和步频/微步数无关。小步进的中频共振典型 100~300Hz，机架结构共振 200~800Hz。
//    扫描方法：固定转速巡航，f_h 从 80Hz 间隔 20Hz 扫到 400Hz，选噪声最小的那个。
// 2) depth 是速度调制深度，1.0 = ±100%。实测代价公式（单边位置纹波）：
//        theta ≈ depth * v / (2*pi*f_h)   [rad]，v 单位 rad/s
//    例：1200rpm = 20 rev/s，f_h=180Hz，depth=0.05 → 单边 0.0018 rev ≈ 0.64°。
//    上一版用的 40Hz/depth=0.2 算出来是 5.7°，这就是“跟随误差跑到 3 度”的来源。
// 3) 参数取法：先 STEP_SEQ_MODE=1 确认平滑有效，再切 2、把 depth 设 0（等价 1），
//    然后只加 depth 到 0.05，最后扫 f_h。
// ---------------------------------------------------------------------------
#ifndef STEP_SEQ_HARMONIC_FREQ_HZ
#define STEP_SEQ_HARMONIC_FREQ_HZ 180.0f
#endif

#ifndef STEP_SEQ_HARMONIC_DEPTH
#define STEP_SEQ_HARMONIC_DEPTH 0.05f
#endif

// 平滑铺满的预算比例（千分比）。不能给 1000：每个窗口都擦着边界结束的话，
// 控制任务偶尔晚几微秒，下一窗 stop_timer_dma_for_reload() 就会把没发完的尾巴砍掉，
// 那是真丢步，跟随误差会反而变大。850~950 比较稳，800 留给任务抖动很大的场合。
#ifndef STEP_SEQ_SPREAD_PERMILLE
#define STEP_SEQ_SPREAD_PERMILLE 900
#endif

// 编译期自证：pio run 的日志里会打印这一段，可以直接确认烧进去的是哪一档。
#if STEP_SEQ_MODE == 0
#pragma message("[STEP_SEQ] mode 0: constant-speed burst (original)")
#elif STEP_SEQ_MODE == 1
#pragma message("[STEP_SEQ] mode 1: smooth spread")
#else
#pragma message("[STEP_SEQ] mode 2: smooth spread + harmonic injection")
#endif

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

    if (ramp_loop_idx % 20000 == 0)
    {
        // 低速槽位（20kHz 下约 1s 一次）：这里是唯一真正改编码器滤波窗口的地方。
        // 滤波器的环形缓冲/累加和正在采样中断里被改，只允许通过 requestFilterWindow
        // 登记请求，由中断自己重配；这里把控住频率，就不会让跟随误差被窗口切换抖出台阶。
        if (encoder_ != nullptr && encoder_filter_window_ != encoder_filter_window_active_)
        {
            encoder_->requestFilterWindow(encoder_filter_window_);
            encoder_filter_window_active_ = encoder_filter_window_;
        }
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW, encoder_filter_window_active_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_FAULT_STATUS, fault_flags);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_FAULT, encoder_fault_active_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_OUTPUT_STOP, output_stopped_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_POS_LOOP_HZ, position_loop_hz_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_VEL_LOOP_HZ, velocity_loop_hz_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_CUR_LOOP_HZ, current_loop_hz_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_LAST_FAULT, fault_flags);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_START_RPM, static_cast<uint32_t>(motion_start_rpm_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAX_RPM, static_cast<uint32_t>(motion_max_rpm_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACCEL_RPM_S, static_cast<uint32_t>(motion_accel_rpm_s_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_MAX_RPM, static_cast<uint32_t>(angle_max_rpm_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S, static_cast<uint32_t>(angle_accel_rpm_s_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_PULSE_COUNT, motion_pulse_count_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_SPEED_RPM,
                                      static_cast<uint32_t>(static_cast<int32_t>(encoder_speed_rpm_)));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS, step_pulse_width_ns_);
    } else if (ramp_loop_idx % 200 == 0)
    {
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_FAULT, magnetic_fault_active_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_A,
                                      static_cast<uint32_t>(static_cast<int32_t>(phase_a_current_a_ * 1000.0f)));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_AB_CURRENT_B,
                                      static_cast<uint32_t>(static_cast<int32_t>(phase_b_current_a_ * 1000.0f)));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_POSITION, static_cast<uint32_t>(target_step_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ACTUAL_POSITION, static_cast<uint32_t>(actual_step_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_FOLLOW_ERROR, static_cast<uint32_t>(follow_error_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_RAW, encoder_filtered_angle_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG, static_cast<uint32_t>(static_cast<int32_t>(getEncoderAngleMilliDegrees() * 1000.0f)));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_HIGH, magnetic_field_high_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MAGNETIC_LOW, magnetic_field_low_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_MODE, static_cast<uint32_t>(motion_mode_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_MOTION_COMMAND, motion_running_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_TARGET_ANGLE_DEG,
                                      static_cast<uint32_t>(static_cast<int32_t>(target_angle_deg_ * 1000.0f)));
        // 外部脉冲跟随遥测：命令/已发/积压/丢步（100Hz 足够上位机画图）
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE, external_follow_enabled_ ? 1U : 0U);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S, external_max_step_s_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS,
                                      static_cast<uint32_t>(external_cmd_steps_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS,
                                      static_cast<uint32_t>(external_emitted_steps_));
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG, external_backlog_steps_);
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED, external_dropped_steps_);
    } else
    {
        protocol_->setCustomParameter(TMC2209_EXT_PARAM_POSITION_DEG,
                                      static_cast<uint32_t>(static_cast<int32_t>(closed_loop_angle_error_deg_ * 1000.0f)));
    }
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
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_UPLOAD_COUNT)
    {
        load_angle_fit_saved_ = false;
        return load_angle_.beginUpload(static_cast<uint16_t>(value));
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_INDEX)
    {
        load_angle_point_index_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_SPEED_RPM)
    {
        load_angle_point_speed_rpm_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_MDEG)
    {
        load_angle_point_mdeg_ = static_cast<int32_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COMMIT)
    {
        return value != 0U && load_angle_.setPoint(load_angle_point_index_,
                                                    load_angle_point_speed_rpm_,
                                                    load_angle_point_mdeg_);
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT)
    {
        if (value == 0U)
        {
            return true;
        }
        load_angle_fit_saved_ = load_angle_.fit() && load_angle_.saveToFlash();
        return load_angle_fit_saved_;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_SPEED_RPM)
    {
        load_angle_query_speed_rpm_ = static_cast<uint16_t>(value);
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
            // 跟随时 stopMotion() 停不掉“跟随”本身：external_follow_enabled_ 还开着，
            // 下一拍就会接着发。所以停运动必须把跟随一起关掉。
            enableExternalPulseFollow(false);
            stopMotion();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATE_ENCODER)
    {
        calibrateEncoder();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        // 只登记期望值，真正的重配交给低速槽位（syncProtocolTelemetry）
        encoder_filter_window_ = value > EncoderFilterConfig::kMaxWindow
            ? EncoderFilterConfig::kMaxWindow
            : static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        enableExternalPulseFollow(value != 0U);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        // 0 = 自动（按 angle_max_rpm_ 换算）；上限按 2.56Mstep/s（128 步/窗）夹住
        setExternalPulseMaxStepRate(value > 2560000UL ? 2560000UL : value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_RESET)
    {
        if (value != 0U)
        {
            resetExternalPulseFollowCounters();
        }
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
            : (reg == TMC2209_EXT_PARAM_POSITION_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KI)
                ? 3000.0f
                : 30000.0f;  // pid最大值分别限制在30000/3000/30000
        const float gain = fast_clamp(static_cast<float>(value) / 1000.0f, 0.0f, max_gain); // 这些除以1000是因为上位机是传整形过来的，直接除以1000保留三位小数
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
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        *value = encoder_filter_window_active_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        *value = external_follow_enabled_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        *value = external_max_step_s_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS)
    {
        *value = static_cast<uint32_t>(external_cmd_steps_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS)
    {
        *value = static_cast<uint32_t>(external_emitted_steps_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG)
    {
        *value = external_backlog_steps_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED)
    {
        *value = external_dropped_steps_;
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
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_INDEX)
    {
        *value = load_angle_point_index_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_SPEED_RPM)
    {
        uint16_t speed_rpm = load_angle_point_speed_rpm_;
        int32_t load_angle_mdeg = load_angle_point_mdeg_;
        load_angle_.getPoint(load_angle_point_index_, &speed_rpm, &load_angle_mdeg);
        *value = speed_rpm;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_MDEG)
    {
        uint16_t speed_rpm = load_angle_point_speed_rpm_;
        int32_t load_angle_mdeg = load_angle_point_mdeg_;
        load_angle_.getPoint(load_angle_point_index_, &speed_rpm, &load_angle_mdeg);
        *value = static_cast<uint32_t>(load_angle_mdeg);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_SPEED_RPM)
    {
        *value = load_angle_query_speed_rpm_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_MDEG)
    {
        *value = static_cast<uint32_t>(load_angle_.getLoadAngleMilliDegrees(load_angle_query_speed_rpm_));
        return load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COUNT)
    {
        *value = load_angle_.pointCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT)
    {
        *value = load_angle_fit_saved_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_SLOPE)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(load_angle_.powerCoefficient() * 1000000.0f));
        return load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_INTERCEPT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(load_angle_.interceptDeg() * 1000.0f));
        return load_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_EXPONENT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(load_angle_.exponent() * 1000000.0f));
        return load_angle_.isFitted();
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

    if (external_follow_enabled_)
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
    if (external_follow_enabled_)
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

namespace
{
    constexpr float k_two_pi = 6.28318530718f;

#if STEP_SEQ_MODE == 2
    // 谐波注入的相位（rad），跨 ramp 窗口连续累加：
    // 如果每个窗口都从 0 相位重启，就等于 20kHz 的锯齿调频，会产生一大堆无用边带。
    float g_step_seq_phase_rad = 0.0f;
#endif

    // 填充一个 ramp 窗口内的 ARR（每步间隔）序列。
    // base_ticks  ：恒速模式下每步的固定间隔（tick，1 tick = 4ns）
    // min_ticks   ：允许的最小步距，必须 > 脉宽 tick，否则高电平被截断直接丢步
    // budget_ticks：本窗口可用的总 tick 预算（= remaining_ticks），平滑模式下用它守恒
    void generate_step_sequence(uint32_t *seq, const uint32_t seq_count, const uint32_t base_ticks,
                                const uint32_t min_ticks, const uint32_t budget_ticks)
    {
#if STEP_SEQ_MODE == 0
        // ---- 档位 0：恒速突发，整窗用最小步距一口气打完，剩余时间全留给 guard ----
        const uint32_t base = (base_ticks < min_ticks) ? min_ticks : base_ticks;
        for (uint32_t i = 1; i < seq_count - 1; i++)  // 头尾是换向 guard，不是脉冲
        {
            seq[i] = base;
        }
#else
        // ---- 档位 1/2：把本窗 N 步铺满整个预算窗口 ----
        // 逐步用“剩余预算 / 剩余步数”求均值，而不是一次性算 budget/N：
        //   1) 整数除法余数被后面的步自动吃掉，整窗总时长误差 < 1 tick(4ns)，永不累积；
        //   2) 谐波调制的摆动会被后面的步“吸回均值”，总时长严格等于预算，
        //      所以窗口边界不会产生位置漂移（上一版 /(1+depth) 缩放就是死在这里）。
        uint32_t budget = budget_ticks;
        uint32_t steps_left = seq_count - 2U;  // 头尾是换向 guard

#if STEP_SEQ_MODE == 2
        float phase = g_step_seq_phase_rad;
#endif

        for (uint32_t i = 1; i < seq_count - 1; i++)
        {
            uint32_t ticks;
            if (steps_left <= 1U)
            {
                // 最后一步把预算里剩下的（含除法余数）全部吃掉，保证与窗口严格对齐
                ticks = budget;
            }
            else
            {
                const uint32_t avg = budget / steps_left;
                ticks = avg;

#if STEP_SEQ_MODE == 2
                ticks = static_cast<uint32_t>(
                    static_cast<float>(avg) * (1.0f + STEP_SEQ_HARMONIC_DEPTH * sinf(phase)) + 0.5f);
                // 相位按“真实流逝时间”推进（用调制后的 ticks），物理上才是正确的调频；
                // 注入频率只由 f_h 决定，与步频无关，不存在自激问题。
                phase += k_two_pi * STEP_SEQ_HARMONIC_FREQ_HZ *
                         (static_cast<float>(ticks) *
                          static_cast<float>(stepper_common::target_tick_time) * 1.0e-9f);
                if (phase >= k_two_pi)
                {
                    phase -= k_two_pi;
                }
#endif

                // 给后面每一步至少留 min_ticks，否则总线必然超窗（下一个窗口会砍掉没发完的脉冲）
                const uint32_t reserve = (steps_left - 1U) * min_ticks;
                const uint32_t max_this = (budget > reserve) ? (budget - reserve) : min_ticks;
                if (ticks > max_this)
                {
                    ticks = max_this;
                }
                if (ticks < min_ticks)
                {
                    ticks = min_ticks;
                }
            }

            seq[i] = ticks;
            budget -= ticks;
            steps_left--;
        }

#if STEP_SEQ_MODE == 2
        g_step_seq_phase_rad = phase;
#endif
#endif
    }

    void reset_step_sequence_harmonic(void)
    {
#if STEP_SEQ_MODE == 2
        g_step_seq_phase_rad = 0.0f;
#endif
    }
} // namespace

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
    ramp_loop_idx++;

    // 更新FOC频率信息
    updateLoopFrequencyStats(now_ns);

    // 外部 PUL/DIR 指令采集：DMA 只搬原始快照，这里一次性批量解析本轮的脉冲。
    // 采集用 TMR3（PB4/PB5），内部 STEP 输出用 TMR2（PB10/PB11），互不干扰。
    external_pulse_delta_ = pulse_capture_read_delta();

    // 跟随模式：采集到的脉冲直接累加成“命令位置”（1 脉冲 = 1 驱动器微步）。
    // 开环/闭环都走这一条命令，区别只在后面要不要叠 PID 修正量。
    if (external_follow_enabled_)
    {
        if (external_prime_pending_)
        {
            // 使能后的第一拍：把使能之前积压的历史脉冲扔掉，
            // 否则一上位就先把刚才调试用的那几千个脉冲补发出去，直接飞车。
            external_prime_pending_ = false;
            external_pulse_delta_ = 0;
        }
        external_cmd_steps_ += external_pulse_delta_;
    }

    if (driver_ == nullptr)
    {
        return;
    }

    float encoder_delta_deg = 0.0f;

    // 上一窗口交给 TMR2/DMA 的脉冲，到这一拍已经全部发完了（窗口 50us，下一拍在它之后）。
    // 只有把它并入“已发完步数”，下面的命令位置才能和真实脉冲对齐，
    // 否则跟随误差会凭空多出一个窗口的滞后（高速下约 0.3~0.4°）。
    if (motion_dma_pending_signed != 0)
    {
        motion_steps_delivered_signed += motion_dma_pending_signed;
        motion_dma_pending_signed = 0;
    }

    // 更新编码器信息
    if (encoder_ != nullptr)
    {
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
            // 用“已确认发完”的步数，而不是“已交给 DMA”的步数，命令位置才和真实脉冲同步
            motion_commanded_travel_deg_ = static_cast<float>(motion_steps_delivered_signed) * step_angle_deg;

            auto motion_encoder_travel_angle_ = static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F;
            motion_follow_error_deg_ = normalize_signed_angle_error_deg(
                motion_encoder_travel_angle_ - motion_commanded_travel_deg_);  // 比如命令要走3度，结果才走了2度，那跟随误差就是-1度。实际-目标


            // if (motion_steps_emitted_ < 300)
            // {
            //     motion_commanded_travel_deg_list_.emplace_back(motion_commanded_travel_deg_);
            //     motion_encoder_travel_angle_list_.emplace_back(motion_encoder_travel_angle_);
            // } else if (motion_pulse_count_ - motion_steps_emitted_ < 300)
            // {
            //     motion_commanded_travel_deg_list_.emplace_back(motion_commanded_travel_deg_);
            //     motion_encoder_travel_angle_list_.emplace_back(motion_encoder_travel_angle_);
            // }
        }
        motion_encoder_previous_raw_ = encoder_filtered_raw_;
    }

    updateEncoderCalibration(encoder_filtered_raw_);

    // ===================== 【外部 PUL/DIR 跟随：独立输出通道】=====================
    // 放在梯形规划/沉降收尾之前：跟随时命令位置完全由外部脉冲决定，
    // 不能进“发满 motion_pulse_count_ 就收尾”那套状态机。
    if (external_follow_enabled_)
    {
        // 注意：delta_ramp_ns 在本函数稍后才会算，这里自己取一次，
        // 并限幅——首拍或长时间没进 rampUpdate 时不能拿一个巨量时间去补发。
        const uint64_t external_delta_ns = now_ns - motion_last_ramp_time_ns_;
        // 这里提前 return 了，下面那段更新 encoder_speed_rpm_ 的代码跑不到，
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

    // ===================== 【运动收尾：沉降确认】=====================
    // 脉冲全部交给 TMR2/DMA 不等于电机已到位：DMA 序列里还排着最后一个窗口的脉冲，
    // 机械轴也还差一个负载角。这里挂着等两者都结束，再让后面的代码收尾。
    // 注意：这段时间 motion_running_ 仍为 true，所以 0x011F 仍上报“运行中”，
    // 上位机会继续采样，波形图上能看到跟随误差从负值收敛回稳态负载角。
    if (motion_ramp_stage_ == RAMP_STAGE_SETTLE)
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

    if (closed_loop_angle_mode_enabled_)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        // 以真实机械角度误差做闭环控制，单位是度/圈；
        // 微分步数只影响输出脉冲密度，不改变 PID 的误差定义，
        // 因此不同驱动或不同细分设置不会让同一组 PID 失效。
        const float desired_error_deg = normalize_signed_angle_error_deg(target_angle_deg_ - motion_position_deg_);  // 目标-实际
        closed_loop_angle_error_deg_ = desired_error_deg;

        if (dt > 0.0f)
        {
            const float position_error_rev = desired_error_deg / 360.0f;
            const float position_reference_rps = position_pid_.update(position_error_rev, dt);  // 根据误差角算出位置环的速度修正量
            // const float velocity_error_rps = position_reference_rps - (encoder_speed_rpm_ / 60.0f);
            const float base_speed_rps = current_step_speed_ / static_cast<float>(getMicroStepsPerRound(driver_));
            const float total_ref_rps = base_speed_rps + position_reference_rps;  // 总期望速度 = 基础速度 + 位置环修正量
            const float velocity_error_rps = total_ref_rps - (encoder_speed_rpm_ / 60.0f);  // 总期望速度和实际速度的速度差
            const float velocity_correction_rps = velocity_pid_.update(velocity_error_rps, dt); // 根据总期望速度和实际速度的速度差和当前的速度计算速度环修正量

            const float max_rps = angle_max_rpm_ / 60.0f;
            const float unconstrained_rps = fast_clamp(position_reference_rps + velocity_correction_rps,
                                                       -max_rps,
                                                       max_rps);

            const float target_direction = desired_error_deg >= 0.0f ? 1.0f : -1.0f;
            const float directed_rps =  unconstrained_rps * target_direction < 0.0f
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
                compensated_motion_step_accumulator_ = 0.0f;
                compensated_rps_ = 0.0f;
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
            // if (driver_ != nullptr)
            // {
            //     driver_->setDirection(motion_direction_ > 0);
            // }
        }

        // 注释掉这段，这段代码不能启用，因为我需要它摇摆才能测好pid，但是以后需要强制定在某个角度可能有用
        // if (fast_abs(closed_loop_angle_error_deg_) <= angle_position_tolerance_deg_)
        // {
        //     current_step_speed_ = 0.0f;
        //     motion_speed_rpm_ = 0.0f;
        //     motion_step_accumulator_ = 0.0f;
        //     compensated_motion_step_accumulator_ = 0.0f;
        //     compensated_rps_ = 0.0f;
        //     position_pid_.resetIntegral();
        //     position_pid_.resetDeriv();
        //     velocity_pid_.resetIntegral();
        //     velocity_pid_.resetDeriv();
        // }
    }
    else
    {
        // 往返摇摆运动模式
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
            compensated_motion_step_accumulator_ = 0.0f;
            compensated_rps_ = 0.0f;
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
                motion_running_ = false;
                motion_first_run_ = false;
                current_step_speed_ = 0.0f;
                motion_speed_rpm_ = 0.0f;
                motion_step_accumulator_ = 0.0f;
                compensated_motion_step_accumulator_ = 0.0f;
                compensated_rps_ = 0.0f;
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
            // 关键：这里不能直接 stopMotion()。最后一个窗口的脉冲才刚进 TMR2/DMA 序列，
            // 立即关定时器会把尾巴上几十个脉冲连同对应角度一起砍掉（高速下 ≈0.36°），
            // 这就是上位机总是统计成“少了一点点角度”的原因。
            // 先挂起 DDA 并进入沉降阶段，等脉冲真正发完 + 轴静止，再由沉降块收尾。
            motion_ramp_stage_ = RAMP_STAGE_SETTLE;
            current_step_speed_ = 0.0f; // 运动结束强制把当前速度清零，防止下次运动残留速度
            motion_speed_rpm_ = 0.0f;
            motion_step_accumulator_ = 0.0f;
            compensated_motion_step_accumulator_ = 0.0f;
            compensated_rps_ = 0.0f;
            motion_settle_start_ns_ = now_ns;
            motion_settle_stable_ramps_ = 0U;
            motion_settle_last_raw_ = encoder_filtered_raw_;
            motion_last_ramp_time_ns_ = now_ns;
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
        const float position_reference_rps = position_pid_.update(-motion_follow_error_deg_ / 360.0f, dt);
        const float measured_motion_rps = encoder_speed_rpm_ / 60.0f;
        // 总期望速度 = 基础速度 + 位置环修正量
        static const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        const float base_speed_rps = current_step_speed_ / static_cast<float>(micro_steps_per_round);
        const float total_ref_rps = base_speed_rps + position_reference_rps;
        const float velocity_error_rps = total_ref_rps - measured_motion_rps;
        const float velocity_correction_rps = velocity_pid_.update(velocity_error_rps, dt);
        const float max_rps = motion_max_rpm_ / 60.0f;

        const float target_direction = (-motion_follow_error_deg_) >= 0.0f ? 1.0f : -1.0f;
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
        const float limited_correction_step_s = velocity_correction_rps * static_cast<float>(motion_direction_) *
                                        static_cast<float>(micro_steps_per_round);
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
        const float directed_rps = (position_reference_rps + velocity_correction_rps) * target_direction;
        const float accel_limit_rps2 = angle_accel_rpm_s_ / 60.0f;
        const float previous_rps = motion_speed_rpm_ / 60.0f;
        const float max_delta_rps = accel_limit_rps2 * dt;
        // const float desired_rps = fast_clamp(directed_rps,
        //                                      previous_rps - max_delta_rps,
        //                                      previous_rps + max_delta_rps);

        compensated_rps_ = directed_rps;
    }

    const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                                    motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;

    if (!closed_loop_angle_mode_enabled_ && driver_ != nullptr)
    {
        const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
        motion_speed_rpm_ = current_step_speed_ * 60.0f / static_cast<float>(micro_steps_per_round);
    }

    // ========== 2.DDA微分累加器：生成步进脉冲，核心部分 ==========
    // DDA原理：步进步数增量 = 瞬时速度(step/s) × 流逝时间(s)
    // 即使任务被抢占延迟很久，delta_step_ns会记录真实流逝时间，累加器累积需要输出的步数
    // while循环一次性输出多个脉冲，做到调度抖动下不丢脉冲

    // 时间单位换算：纳秒 → 秒
    const double dt_step_s = static_cast<double>(delta_ramp_ns) / 1.0e9f;

    // 累加本次时间内应该产生的步数（浮点数，允许小数累积）
    motion_step_accumulator_ += fast_abs(current_step_speed_) * dt_step_s;

    const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
    const auto compensated_step_speed_ = compensated_rps_ * static_cast<float>(micro_steps_per_round);

    compensated_motion_step_accumulator_ += fast_abs(current_step_speed_ + compensated_step_speed_) * dt_step_s;

    if (closed_loop_angle_mode_enabled_ && motion_step_accumulator_ > 1.0)
    {
        motion_step_accumulator_ = 1.0;
        compensated_motion_step_accumulator_ = 1.0;
        compensated_rps_ = 0.0;
    }

    // TMR2 是 32 位计数器，因此 ARR 序列必须是 32 位。
    // 这里复用同一份周期表，正转和反转都只需要切换 DIR 输出 + 重新装载同一块 RAM，
    // 这样既能完成“正转一圈 -> 反转一圈”，又不会把 RAM 翻倍消耗掉。

    if ((compensated_motion_step_accumulator_ >= 1.0f) &&
        (closed_loop_angle_mode_enabled_ || motion_steps_emitted_ < motion_pulse_count_))
    {
        // 人为规定第一个和最后arr周期是用来提前和延后换向的，
        // 如果是同向，这两个的arr周期为0，否则arr为guard_tick
        static uint32_t arr_seq_cycle[MAX_PULSE_ARR_LEN + 2];
        auto num_steps = static_cast<uint32_t>(floor(compensated_motion_step_accumulator_));

        if (num_steps > max_step_per_ramp)
            max_step_per_ramp = num_steps;

        auto num_original_command_steps = static_cast<uint32_t>(floor(motion_step_accumulator_));
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
        if (delta_ramp_ns <= overhead_ns)
        {
            motion_last_ramp_time_ns_ = now_ns;
            syncProtocolTelemetry();
            return;
        }
        const uint64_t remaining_time = delta_ramp_ns - overhead_ns;
        const uint64_t remaining_ticks = remaining_time / stepper_common::target_tick_time;

        const uint32_t min_step_ticks = k_step_pulse_ticks * 2U;  // 最少脉宽两倍，留足高电平脉宽之余的低电平脉宽
        uint32_t step_ticks = min_step_ticks;
        bool spread_enabled = false;  // true = 本窗用预算铺满（档位 1/2），false = 恒速突发

#if STEP_SEQ_MODE != 0
        // 高速兑底：步频超过 1/(2*脉宽) = 2.5 Mstep/s 时，连 min_step_ticks 都塞不下，
        // 平滑没有意义且必定超窗（下一窗会砍掉没发完的脉冲 → 真丢步），退回恒速突发。
        // 另：只铺满窗口预算的 STEP_SEQ_SPREAD_PERMILLE，剩下的余量留给控制任务
        // 调度抖动和 start_step_sequence 的软件开销，不让每个窗口都擦着边界结束。
        const uint64_t spread_budget_ticks =
            (remaining_ticks * static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000ULL;
        const uint64_t spread_need_ticks = static_cast<uint64_t>(num_steps) * min_step_ticks;
        if ((num_steps != 0U) && (spread_budget_ticks >= spread_need_ticks))
        {
            spread_enabled = true;
            // 仅用于日志/调试，实际铺开由 generate_step_sequence 在预算内逐步决定
            step_ticks = static_cast<uint32_t>(spread_budget_ticks / num_steps);
        }
#endif

        // 丢步直接丢了，不打印b报错，因为打印反而会加剧丢步
        // uint64_t max_iter_time_ns = (2 * stepper_common::guard_tick + step_ticks * num_steps) * stepper_common::target_tick_time;
        // // 因为硬件TMR定时器+DMA工作是异步的，耗时必须短于软件定时器迭代时间，否则就会输出延迟
        // if (delta_step_ns < max_iter_time_ns)
        // {
        //     printf("step_ticks: %lu, num_steps: %lu, now_ns: %.6fus, motion_last_ramp_time_ns_: %.6fus, delta_step_ns: %.6fus\n", step_ticks, num_steps, now_ns / 1000.f, motion_last_ramp_time_ns_ / 1000.f, delta_step_ns / 1000.f);
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
        //     printf("step_ticks: %lu, num_steps: %lu, now_ns: %.6fus, motion_last_ramp_time_ns_: %.6fus, delta_step_ns: %.6fus\n", step_ticks, num_steps, now_ns / 1000.f, motion_last_ramp_time_ns_ / 1000.f, delta_step_ns / 1000.f);
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
        // 注意这里仅支持每次rampupdate最多一次换向。后续需要根据换向次数和位置提前规划好换向点并安排好guard ticks
        //
        // 预算窗口转 uint32（任务阻塞极端情况下 remaining_ticks 可能很大，封顶防溢出）。
        // spread_enabled = false 时（档位 0 或高速兑底）budget 参数不被使用。
        const uint64_t budget_ticks_wide = spread_enabled
                                               ? (remaining_ticks * static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000ULL
                                               : static_cast<uint64_t>(num_steps) * min_step_ticks;
        const uint32_t budget_ticks = (budget_ticks_wide > 0x3FFFFFFFULL)
                                          ? 0x3FFFFFFFU
                                          : static_cast<uint32_t>(budget_ticks_wide);
        generate_step_sequence(arr_seq_cycle, seq_count, step_ticks, min_step_ticks, budget_ticks);
        add_dir_to_step_sequence(arr_seq_cycle, seq_count, stepper_common::guard_tick,
                     motion_direction_change_pending_);
        start_step_sequence(arr_seq_cycle, seq_count, stepper_common::k_psc, stepper_common::k_step_pulse_ticks, current_direction);
#endif
        motion_direction_change_pending_ = false;

        if (!closed_loop_angle_mode_enabled_)
        {
            motion_steps_emitted_ += num_original_command_steps;
            motion_steps_emitted_signed += motion_direction_ * num_original_command_steps;  // 正向就加步数，反向就减
            // 本窗这批脉冲刚交给 DMA，要等下一拍（+50us）确认真的发完，
            // 那时才并进 motion_steps_delivered_signed，用作跟随误差的命令位置。
            motion_dma_pending_signed = motion_direction_ * static_cast<int32_t>(num_original_command_steps);
        }
        motion_step_accumulator_ -= num_original_command_steps;
        compensated_motion_step_accumulator_ -= num_steps;
    }

    // 更新本次ramp的时间戳，作为下一次调用的“上一次时间点”
    motion_last_ramp_time_ns_ = now_ns;
    syncProtocolTelemetry();
}

void ClosedLoopController::stopMotion()
{
    // for (int i = 0; i < 300; i++)
    // {
    //     printf("fuck:commanded_travel_deg: %.3f, encoder_travel_angle: %.3f\n", motion_commanded_travel_deg_list_[i], motion_encoder_travel_angle_list_[i]);
    // }
    // for (int i = motion_commanded_travel_deg_list_.size() - 300; i < motion_commanded_travel_deg_list_.size(); i++)
    // {
    //     printf("shit:commanded_travel_deg: %.3f, encoder_travel_angle: %.3f\n", motion_commanded_travel_deg_list_[i], motion_encoder_travel_angle_list_[i]);
    // }
    // motion_commanded_travel_deg_list_.clear();
    // motion_encoder_travel_angle_list_.clear();
    // motion_commanded_travel_deg_list_.resize(0);
    // motion_encoder_travel_angle_list_.resize(0);

    printf("max_step_per_ramp: %lu\n", static_cast<unsigned long>(max_step_per_ramp));
    printf("motion stopping!!! motion_commanded_travel_deg_: %f, motion_encoder_travel_deg_: %f\n", motion_commanded_travel_deg_, static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F);
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
    motion_last_ramp_time_ns_ = 0ULL;
    // printf("fuck3 motion_last_ramp_time_ns_: %f\n", motion_last_ramp_time_ns_/1e9);
    motion_step_high_ = false;
    reset_step_sequence_harmonic();  // 下一局运动重新从 0 相位开始注入
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
    if (driver_ == nullptr || delta_ramp_ns == 0U || delta_ramp_ns > 1000000ULL)
    {
        // 首拍（motion_last_ramp_time_ns_ 还是 0）或调度卡顿超过 1ms 时不补发：
        // 积压会留在 external_cmd_steps_ 里由后面几拍慢慢追，比一次吐上百步安全得多。
        return;
    }

    // 故障保护：编码器/磁场报警时驱动已被 reportXxxFault() 失能，
    // 这里再发脉冲只会空转 TMR2（且积压白白变成“丢步”），直接不输出。
    if (output_stopped_)
    {
        motion_speed_rpm_ = 0.0f;
        return;
    }

    const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
    const float dt_s = static_cast<float>(delta_ramp_ns) / 1.0e9f;

    // 脉宽可能在跟随时被上位机改（0x123），每窗按最新值刷一遍。
    // DDA 路径是 startMotion() 里算的，跟随路径没人管，漏了就是一个陈旧的 min_step_ticks。
    k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) /
                                   static_cast<float>(stepper_common::target_tick_time));

    // ---- 1. 本窗要“1:1 转发”的步数：积压、限速、序列长度三者取小 ----
    const int32_t backlog = external_cmd_steps_ - external_emitted_steps_;
    const int32_t backlog_abs = backlog >= 0 ? backlog : -backlog;

    // 限速：0 表示用 angle_max_rpm_ 换算出的驱动能力上限（默认，1200RPM x 12800 = 256k step/s）。
    // 宁可比外部慢一点，也不要一次性灌满让电机失步——失步是真正找不回来的。
    uint32_t rate_step_s = external_max_step_s_;
    if (rate_step_s == 0U)
    {
        const float rpm_cap = angle_max_rpm_ > 0.0f ? angle_max_rpm_ : 1200.0f;
        const float step_s_cap = rpm_cap * static_cast<float>(micro_steps_per_round) / 60.0f;
        rate_step_s = static_cast<uint32_t>(step_s_cap > 1.0f ? step_s_cap : 1.0f);
    }

    float budget_steps = static_cast<float>(rate_step_s) * dt_s;
    if (budget_steps < 1.0f)
    {
        // 低速时也至少允许 1 步，否则小数部分会被限速反复截断，永远凑不满一整步
        budget_steps = 1.0f;
    }
    if (budget_steps > static_cast<float>(backlog_abs))
    {
        budget_steps = static_cast<float>(backlog_abs);
    }

    // 限速值不是“整步/窗”时（如 256k step/s 就是 12.8 步/窗），
    // 直接截断会每窗少 0.8 步 = 长期恒定落后 6%，必须靠余量补回来。
    float base_acc = budget_steps + external_base_residue_;
    uint32_t num_base = static_cast<uint32_t>(base_acc);
    if (num_base > static_cast<uint32_t>(backlog_abs))
    {
        num_base = static_cast<uint32_t>(backlog_abs);
    }
    external_base_residue_ = base_acc - static_cast<float>(num_base);
    if (external_base_residue_ > 1.0f)
    {
        external_base_residue_ = 1.0f;
    }
    else if (external_base_residue_ < 0.0f)
    {
        external_base_residue_ = 0.0f;
    }

    const int8_t base_sign = backlog >= 0 ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    const float signed_base = static_cast<float>(num_base) * static_cast<float>(base_sign);

    // ---- 2. 闭环补偿（0x107=1 才生效）：位置环 -> 速度环 -> 本窗修正步数 ----
    // 开环时这一段整体跳过，signed_base 就是全部输出，实现严格 1:1 转发。
    float pid_steps = 0.0f;
    if (closed_loop_compensation_enabled_ && encoder_ != nullptr)
    {
        // 位置环输入必须是“命令 - 实际”：motion_follow_error_deg_ 定义是“实际 - 命令”，要取反
        const float position_ref_rps = position_pid_.update(-motion_follow_error_deg_ / 360.0f, dt_s);
        const float base_rps = signed_base / static_cast<float>(micro_steps_per_round) / dt_s;
        const float measured_rps = encoder_speed_rpm_ / 60.0f;
        const float velocity_correction_rps =
            velocity_pid_.update(base_rps + position_ref_rps - measured_rps, dt_s);
        pid_steps = velocity_correction_rps * static_cast<float>(micro_steps_per_round) * dt_s;
        // PID 没调好时别让它一个窗口把整段序列吃光（真正超速由下面的限速和丢步计数兜）
        const float pid_cap = static_cast<float>(MAX_PULSE_ARR_LEN) * 0.5f;
        if (pid_steps > pid_cap)
        {
            pid_steps = pid_cap;
        }
        else if (pid_steps < -pid_cap)
        {
            pid_steps = -pid_cap;
        }
        // 让 0x121 等遥测继续看得到补偿量（开环时是 0）
        compensated_rps_ = velocity_correction_rps;
    }
    else
    {
        compensated_rps_ = 0.0f;
    }

    // ---- 3. 合成本窗的脉冲数 ----
    float signed_steps = signed_base + pid_steps;
    if (signed_steps > static_cast<float>(MAX_PULSE_ARR_LEN))
    {
        signed_steps = static_cast<float>(MAX_PULSE_ARR_LEN);
    }
    else if (signed_steps < -static_cast<float>(MAX_PULSE_ARR_LEN))
    {
        signed_steps = -static_cast<float>(MAX_PULSE_ARR_LEN);
    }
    const uint32_t num_steps = static_cast<uint32_t>(fast_abs(signed_steps));

    if (num_steps == 0U)
    {
        // 没积压、补偿也是 0：保持使能，安静等下一拍（这不等于停止跟随）
        external_backlog_steps_ = static_cast<uint32_t>(backlog_abs);
        motion_speed_rpm_ = 0.0f;
        return;
    }

    const int8_t direction = signed_steps >= 0.0f ? 1 : -1;
    // 外部 DIR 什么时候翻我们不知道，只能逐窗比：换向那一窗必须在序列头尾插 guard，
    // 否则贴着发的脉冲会被驱动器认错方向（甚至整步丢掉）。
    if (motion_direction_ != direction)
    {
        motion_direction_change_pending_ = true;
    }
    motion_direction_ = direction;
    motion_speed_rpm_ = static_cast<float>(num_steps) / dt_s * 60.0f /
                        static_cast<float>(micro_steps_per_round);

    // ---- 4. 铺 ARR 序列并启动 TMR2/DMA（与 DDA 段同一套出口/同一种换向 guard）----
    const uint64_t guard_ns =
        2ULL * stepper_common::guard_tick * stepper_common::target_tick_time + 5ULL;
    if (delta_ramp_ns > guard_ns)
    {
        const uint64_t remaining_ticks = (delta_ramp_ns - guard_ns) / stepper_common::target_tick_time;
        const uint32_t min_step_ticks = k_step_pulse_ticks * 2U;

#if STEP_SEQ_MODE != 0
        const uint64_t spread_budget_ticks =
            (remaining_ticks * static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000ULL;
        const uint64_t spread_need_ticks = static_cast<uint64_t>(num_steps) * min_step_ticks;
        const bool spread_enabled = spread_budget_ticks >= spread_need_ticks;
#else
        const uint64_t spread_budget_ticks = 0ULL;
        const bool spread_enabled = false;
#endif
        const uint64_t budget_wide = spread_enabled
            ? spread_budget_ticks
            : static_cast<uint64_t>(num_steps) * min_step_ticks;
        const uint32_t budget_ticks = budget_wide > 0x3FFFFFFFULL
            ? 0x3FFFFFFFU
            : static_cast<uint32_t>(budget_wide);
        const uint32_t step_ticks = spread_enabled
            ? static_cast<uint32_t>(budget_ticks / num_steps)
            : min_step_ticks;

        const uint32_t seq_count = num_steps + 2U;
        generate_step_sequence(external_arr_seq_, seq_count, step_ticks, min_step_ticks, budget_ticks);
        add_dir_to_step_sequence(external_arr_seq_, seq_count, stepper_common::guard_tick,
                                 motion_direction_change_pending_);
        start_step_sequence(external_arr_seq_, seq_count, stepper_common::k_psc,
                            stepper_common::k_step_pulse_ticks, direction > 0);
        motion_direction_change_pending_ = false;
    }

    // ---- 5. 记账：命令位置走 delivered，不凭空领先一个窗口 ----
    // 注意只把 num_base 计入“已转发”，PID 那部分不算——否则位置环多发的步数
    // 会被积压逻辑当成“发多了”反过来退回，两个环互相推着振荡。
    external_emitted_steps_ += static_cast<int32_t>(num_base) * static_cast<int32_t>(base_sign);
    const int32_t emitted = direction * static_cast<int32_t>(num_steps);
    motion_steps_emitted_signed += emitted;
    motion_dma_pending_signed += emitted;

    const int32_t new_backlog = external_cmd_steps_ - external_emitted_steps_;
    const uint32_t new_backlog_abs =
        static_cast<uint32_t>(new_backlog >= 0 ? new_backlog : -new_backlog);
    external_backlog_steps_ = new_backlog_abs;
    if (new_backlog_abs > external_backlog_peak_steps_)
    {
        external_backlog_peak_steps_ = new_backlog_abs;
    }
    // 告警口径：积压超过一圈 = 电机已经跟不上输入频率了。只把超出部分累加，
    // 所以“正常几拍的延迟、下一窗就补回来”不会触发；这个数持续增长才是真的超速。
    if (new_backlog_abs > micro_steps_per_round)
    {
        external_dropped_steps_ += new_backlog_abs - micro_steps_per_round;
    }
}

void ClosedLoopController::enableExternalPulseFollow(bool enable)
{
    if (external_follow_enabled_ == enable)
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
        external_follow_enabled_ = false;
        external_prime_pending_ = false;
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
    external_prime_pending_ = true;
    external_follow_enabled_ = true;

    if (driver_ != nullptr && !encoder_fault_active_ && !magnetic_fault_active_)
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
    return external_follow_enabled_;
}

void ClosedLoopController::setExternalPulseMaxStepRate(uint32_t steps_per_second)
{
    external_max_step_s_ = steps_per_second;
}

uint32_t ClosedLoopController::getExternalPulseMaxStepRate() const
{
    return external_max_step_s_;
}

int32_t ClosedLoopController::getExternalCmdSteps() const
{
    return external_cmd_steps_;
}

int32_t ClosedLoopController::getExternalEmittedSteps() const
{
    return external_emitted_steps_;
}

uint32_t ClosedLoopController::getExternalBacklogSteps() const
{
    return external_backlog_steps_;
}

uint32_t ClosedLoopController::getExternalPeakBacklogSteps() const
{
    return external_backlog_peak_steps_;
}

uint32_t ClosedLoopController::getExternalDroppedSteps() const
{
    return external_dropped_steps_;
}

void ClosedLoopController::resetExternalPulseFollowCounters()
{
    external_cmd_steps_ = 0;
    external_emitted_steps_ = 0;
    external_base_residue_ = 0.0f;
    external_backlog_steps_ = 0U;
    external_backlog_peak_steps_ = 0U;
    external_dropped_steps_ = 0U;
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
