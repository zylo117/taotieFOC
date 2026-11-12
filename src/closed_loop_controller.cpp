#include "closed_loop_controller.h"
#include "at32f403a_407_board.h"
#include "param_store.h"
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
//   2) 机械侧：电机轴经过负载/皮带总比命令脉冲滞后一个滞后角。
// 原来在“计步数达到总数”的那一拍就直接 stopMotion()，它内部会立刻
// stepper_stop_motion_timer() 关掉 TMR2 + DMA，把尾巴上还没发完的几十个脉冲
// 直接砍掉 —— 高转速1200rpm下一个窗口就 50 多步 ≈ 0.36°，于是“总是少一点点”。
// 现在改成：步数发完后先进入沉降阶段，等最后一批脉冲真的发完 + 编码器连续
// 若干拍不动，再判定运动结束。这段时间的编码器行程照样计入统计。
// ===========================================================================
// 最短沉降时间：必须大于一个控制窗口(50us) 加上若干机械时间常数。
// 给 15ms 是因为滞后角回位/皮带回弹通常在 10ms 量级，太短会在轴还在动时就收尾。
#ifndef STEP_SETTLE_MIN_US
#define STEP_SETTLE_MIN_US 15000U
#endif

// 编码器连续多少拍“不动”才算真正静止（一拍 = 20kHz = 50us）
#ifndef STEP_SETTLE_STABLE_RAMPS
#define STEP_SETTLE_STABLE_RAMPS 8U
#endif

// 多少 LSB 之内算“不动”。滤波后的原始角在跳变沿上会来回抹 1~2 LSB，
// 卡到 1 LSB 的话可能永远凑不满连续 8 拍，反而每次都要等满超时。
// 2 LSB = 0.011°，比机械滞后角小两个数量级，不会把“还在爬行”误判成“已静止”。
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
        : driver_(nullptr), encoder_(nullptr), protocol_(nullptr), lag_angle_point_index_(0U),
            lag_angle_point_speed_rpm_(0U), lag_angle_point_mdeg_(0), lag_angle_query_speed_rpm_(0U),
            lag_angle_fit_saved_(false),
      base_position_kp_(60.0f), base_position_ki_(10.0f), base_position_kd_(0.0f),
      // 【速度环口径：网页值 = 实际增益，不再有 ×0.01 的隐藏倍率】
      // 历史上角度模式把 P 项乘 0.01（那时 PID 输出就是全部速度），所以 30 表示 0.3。
      // 串级改造删掉 p_scale 以后 30 变成真·30 —— 速度环增益比设计值大 100 倍，
      // 一改就抖、一起步就把规划的速度斜坡顶成"直接满速"，都是这一个原因。
      // 现在统一用实际增益：0.3 才是"大约相当于当年 30"的那个点。
      base_velocity_kp_(0.30f), base_velocity_ki_(0.0f), base_velocity_kd_(0.0f),
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
    // 校准专用状态（index_ / forward_raw_ / sample_* / home_*）已搬进
    // EncoderCalibrator，它的成员都有类内默认值，不再在这里初始化；
    // 本类只保留阶段（与 motion_planner 的握手）与运动参数备份。
    calibration_stage_(CALIBRATION_IDLE),
    calibration_saved_start_rpm_(0.0f),
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

    // 【上电恢复】从 Flash 读回上次保存的 PID / 微步 / 脉宽。
    // 固件原来完全没有参数持久化，掉电后 PID 全丢、回到编译期默认值；
    // 现在只要用户点过一次「写入参数」，重启就能自动恢复。
    uint16_t boot_microsteps = 0U;
    uint16_t boot_enable_mode = ENABLE_MODE_HOST;
    uint32_t boot_motion_source = MOTION_SOURCE_EXTERNAL;   // 上电默认：外部脉冲跟随
    uint16_t boot_res_lo = 300U;
    uint16_t boot_res_hi = 600U;
    {
        PersistedMotorParams params = {};
        if (param_store_load(&params))
        {
            position_pid_.kp = static_cast<float>(params.position_kp) / 1000.0f;
            position_pid_.ki = static_cast<float>(params.position_ki) / 1000.0f;
            position_pid_.kd = static_cast<float>(params.position_kd) / 1000.0f;
            velocity_pid_.kp = static_cast<float>(params.velocity_kp) / 1000.0f;
            velocity_pid_.ki = static_cast<float>(params.velocity_ki) / 1000.0f;
            velocity_pid_.kd = static_cast<float>(params.velocity_kd) / 1000.0f;
            // 【必须同步“基准值”】多处代码会在开关闭环/启动运动时调用
            //   setGains(base_position_kp_, ...) / setGains(base_velocity_kp_, ...)
            // 如果只改 PID 实例的系数而不改 base_*，那些调用会把刚从 Flash 读出的值
            // 覆盖回编译期默认值 —— 表现就是“写入成功但重连读不到”。
            base_position_kp_ = position_pid_.kp;
            base_position_ki_ = position_pid_.ki;
            base_position_kd_ = position_pid_.kd;
            base_velocity_kp_ = velocity_pid_.kp;
            base_velocity_ki_ = velocity_pid_.ki;
            base_velocity_kd_ = velocity_pid_.kd;
            // 【微步必须一起恢复】它是"一圈多少微步"的唯一来源：丢了它，
            // getStepsPerRound() 会退回驱动器默认值（历史默认 32），
            // 而脉冲数、滞后角表、网页行程全按 64 口径 —— 行程正好差一倍。
            if (params.microsteps >= 1U && params.microsteps <= 256U)
            {
                boot_microsteps = params.microsteps;
            }
            if (params.en_mode <= 1U)
            {
                boot_enable_mode = params.en_mode;
            }
            // v5：运动源 + 共振带（旧记录迁移时已填默认值）
            boot_motion_source = (params.motion_source <= 1U) ? params.motion_source
                                                              : MOTION_SOURCE_EXTERNAL;
            boot_res_lo = params.resonance_lo_rpm;
            boot_res_hi = params.resonance_hi_rpm;
            // 滞后角模型参数：旧记录（v1/v2/v3）在迁移时已经填了默认值
            lag_accel_coef_mdeg_per_rpm_s_ = params.lag_accel_coef_mdeg_per_rpm_s;
            lag_deadband_mdeg_ = (params.lag_deadband_mdeg >= 10U) ? params.lag_deadband_mdeg : 150U;
            // 【v6 堵转阈值恢复】迁移已填默认值 200/100，但仍校验域，坏值退回默认，
            // 绝不让一个 0 或超界值让堵转保护失效或疯狂误报。
            if (params.stall_quiet_ms >= 50U && params.stall_quiet_ms <= 3000U)
            {
                stall_quiet_ms_ = params.stall_quiet_ms;
            }
            if (params.stall_move_lsb >= 40U && params.stall_move_lsb <= 1500U)
            {
                stall_move_lsb_ = params.stall_move_lsb;
            }
            // 【v7 磁钢极性恢复】校准探针鉴别出反装后随校准表一起固化；
            // 读数坐标系与非线性表必须同域，所以这里要在校准表生效前就位。
            if (encoder_ != nullptr && params.encoder_invert != 0U)
            {
                encoder_->setInvertRaw(true);
                printf("[ENCODER] magnet polarity INVERTED restored from flash\r\n");
            }
            if (params.step_pulse_ns >= kMinStepPulseNs && params.step_pulse_ns <= kMaxStepPulseNs)
            {
                step_pulse_width_ns_ = params.step_pulse_ns;
            }
            // 角度限速/加速度不从 Flash 恢复：它们是“本次运行设定”，每次在界面上调，
            // 存进 Flash 会让一个旧值在重启后静默生效。
            printf("[PARAM] loaded from flash: pid %.3f/%.3f/%.3f %.3f/%.3f/%.3f ms=%u width=%uns\r\n",
                   position_pid_.kp, position_pid_.ki, position_pid_.kd,
                   velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd,
                   static_cast<unsigned>(boot_microsteps),
                   static_cast<unsigned>(step_pulse_width_ns_));
        }
        else
        {
            printf("[PARAM] no saved params, using compile-time defaults\r\n");
        }
    }

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

    // 外部 EN 输入（PC13，低电平使能）：上拉输入，悬空读高 = 失能，安全侧默认。
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_init_type ext_en_gpio;
    gpio_default_para_init(&ext_en_gpio);
    ext_en_gpio.gpio_mode = GPIO_MODE_INPUT;
    ext_en_gpio.gpio_pull = GPIO_PULL_UP;
    ext_en_gpio.gpio_pins = EXT_EN_PIN;
    gpio_init(EXT_EN_PORT, &ext_en_gpio);
    external_en_level_ = (gpio_input_data_bit_read(EXT_EN_PORT, EXT_EN_PIN) != RESET) ? 1U : 0U;
    // 消抖状态以当前电平起步（stable/candidate 一致，change_count=0）。
    external_en_stable_ = external_en_level_;
    external_en_candidate_ = external_en_level_;
    external_en_change_count_ = 0U;

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

    lag_angle_fit_saved_ = lag_angle_.loadFromFlash();
    // 【诊断】滞后角表到底有没有标定过：
    //   fitted=0 -> 从未标定（或 Flash 里没有有效表）-> 滞后角前馈和 0x151 遥测恒为 0
    //   fitted=1 -> 已加载，看 coeff/intercept/exp 是否合理
    printf("[LAG_ANGLE] fitted=%d coeff=%.6f intercept=%.4f exp=%.4f\r\n",
           lag_angle_.isFitted() ? 1 : 0,
           static_cast<double>(lag_angle_.powerCoefficient()),
           static_cast<double>(lag_angle_.interceptDeg()),
           static_cast<double>(lag_angle_.exponent()));

    if (driver_ != nullptr)
    {
        driver_->init();
        // 恢复 Flash 里的微步设置。
        // 【注意】本固件里 TMC UART 是关的（tmc2209_driver.cpp 的 writeRegister 直接返回
        // false），所以 setMicrosteps 只改"软件口径"：一圈按多少微步折算脉冲。
        // 它必须和驱动器面板/引脚决定的**实际细分**一致，否则行程会正好差整数倍
        //（历史上默认 32 而硬件是 64 时，命令 0.5 圈实际只走 0.25 圈）。
        if (boot_microsteps >= 1U)
        {
            driver_->setMicrosteps(boot_microsteps);
        }
        driver_->setEnable(false);
        driver_enabled_ = false;
        driver_->setDirection(false);
        driver_->setStepState(false);
        printf("[PARAM] microsteps active = %u (steps/rev = %lu)\r\n",
               static_cast<unsigned>(driver_->config().microsteps),
               static_cast<unsigned long>(getMicroStepsPerRound(driver_)));
        // 恢复使能来源（必须在 driver init 之后：setEnableMode 会写一次 EN）。
        if (boot_enable_mode == ENABLE_MODE_EXTERNAL_PIN)
        {
            setEnableMode(ENABLE_MODE_EXTERNAL_PIN);
            printf("[PARAM] enable mode = external EN pin (PC13, low-active), level=%u\r\n",
                   static_cast<unsigned>(external_en_level_));
        }
    }

    // 【上电应用运动源 + 共振带】flash 读取失败也要应用编译期默认值
    // （外部跟随 + 300~600 RPM 共振带）。
    // 【开机=进模式，不带电——带电与否看外部 EN 引脚（2026-11-10 用户明确）】
    // 跟随模式下 EN 权威永远是 PC13：上电后最多 200µs（消抖 4 拍）内
    // updateEnableFromExternalPin 按引脚电平决定通不通电流。PC13 内部上拉，
    // 未接 PLC 时悬空读高 = 失能（不发热）；PLC 拉低即刻可使能，无需任何
    // 网页操作。原 energize 参数已删——"进外部模式"和"给电流"彻底解耦。
    resonance_lo_rpm_ = boot_res_lo;
    resonance_hi_rpm_ = boot_res_hi;
    setMotionSource(boot_motion_source);

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

    if (external_pulse_follower_.isEnabled() || motion_source_ == MOTION_SOURCE_EXTERNAL)
    {
        printf("[SOURCE] setTargetAngleDeg rejected: motion source is EXTERNAL follow "
               "(write 0x16D=0 / switch to internal plan first)\r\n");
        return;
    }

    // 【角度模式限幅必须自己非零】updateAngleMode 里 max_rps = angle_max_rpm_/60，
    // 加速度限幅用 angle_accel_rpm_s_。这两个为 0 时 PID 输出会被全部钳成 0，
    // 表现为“设了目标角度但电机不动/极慢”。
    // 原来只兜底了 motion_*（位置模式用），没兜底 angle_*（角度模式实际用）。
    if (angle_max_rpm_ <= 0.0f)
    {
        angle_max_rpm_ = 1200.0f;
    }
    if (angle_accel_rpm_s_ <= 0.0f)
    {
        angle_accel_rpm_s_ = 6000.0f;
    }
    if (motion_max_rpm_ <= 0.0f)
    {
        motion_max_rpm_ = angle_max_rpm_;
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
    // 注意：closed_loop_angle_mode_enabled_ / motion_running_ 是"发布写"，
    // 统一挪到函数末尾（见结尾【发布写】注释），中间态必须让控制任务看成 idle。
    // angle_position_tolerance_deg_ = 0.1f;
    step_pulse_width_ns_ = 1000U;  // TMC2209 STEP 输入规格 ≥1µs（原 200ns 规格外）
    k_step_pulse_ticks = std::ceil(static_cast<float>(step_pulse_width_ns_) / static_cast<float>(stepper_common::target_tick_time));
    closed_loop_angle_error_deg_ = 0.0f;
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    motion_paused_ = false;
    motion_first_run_ = true;
    motion_steps_emitted_ = 0U;
    motion_steps_emitted_signed = 0U;
    motion_step_accumulator_ = 0.0f;
    compensated_motion_step_accumulator_ = 0.0f;
    compensated_rps_ = 0.0f;
    // 【同一个坑：其余共享状态也要在启动时归零】
    // 上位机启动只用 0x11F=1，**不会先调 stopMotion**，所以上次运动/角度模式留下的：
    //   current_step_speed_      → 起跑速度直接跳变（从旧速度接着跑）
    //   motion_ramp_stage_       → 可能是 DONE/SETTLE，导致启动后不动
    //   motion_commanded_travel_deg_ / motion_encoder_travel_raw_ → follow_error 从旧偏差异位开始
    // 都必须在 startMotion 里自己清，不能指望 stopMotion 已经被调过。
    current_step_speed_ = 0.0f;
    motion_speed_rpm_ = 0.0f;
    motion_ramp_stage_ = RAMP_STAGE_ACCEL;
    motion_commanded_travel_deg_ = 0.0f;
    motion_encoder_travel_raw_ = 0U;
    motion_leg_reversed_ = false;
    motion_direction_change_pending_ = false;
    // 【规划运动启动也要清 PID 状态】角度模式跑完一轮后，位置/速度环的积分和一阶低通
    // 微分状态里都还留着旧值。规划运动开头几拍会带着这些残留输出一个方向错误的修正量，
    // 叠加在刚起步的规划速度上，表现为起跑抽一下或前期抖动。
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    motion_encoder_previous_raw_ = encoder_ != nullptr ? encoder_->readFilteredRaw() : encoder_filtered_raw_;
    motion_encoder_reference_valid_ = encoder_ != nullptr;
    if (driver_ != nullptr)
    {
        driver_->setEnable(true);
    }
    // 【发布写】所有字段准备完毕，最后各用单个对齐字写原子放行（USB 任务随时可能被
    // TMR4 拍抢占，中间的半配置状态绝不能带上 enabled/running，否则控制任务会用
    // 旧的累加器/脉宽跑一拍 —— 表现为"设角度后第一下抽一下"）。顺序别改。
    closed_loop_angle_mode_enabled_ = true;
    motion_running_ = true;
}

float ClosedLoopController::getTargetAngleErrorDeg() const
{
    return closed_loop_angle_error_deg_;
}

void ClosedLoopController::setTargetVelocity(float rps)
{
    target_velocity_rps_ = rps;
}

// ===========================================================================
// 使能来源：上位机 / 外部 EN 引脚（PC13，低电平使能）
//
// 为什么要有这个开关：外部脉冲跟随模式下，主控（PLC/上位机）那根 EN 才是
// 真正的急停/使能线；而网页上也有一个"使能/失能"按钮。两者直接抢同一个
// 驱动器 EN 脚，就会出现"网页点了使能，外部 EN 还是高电平，芯片到底听谁的"。
// 规则（2026-11-10 与用户敲定）：
//   · 运动源=外部跟随（0x16D=1，上电默认）：**引脚自动是权威**，开机只是进模式，
//     通不通电流看 PC13 电平（上拉悬空=高=失能不发热；PLC 拉低=使能）。
//   · 运动源=内部规划：按 0x161 选——HOST（默认，只有 0x124/0x125 能动 EN）
//     或 EXTERNAL（引脚权威，同跟随模式）。
// 故障（编码器/磁场/堵转）和失能请求永远能压住使能，不受这些开关影响。
// ===========================================================================
void ClosedLoopController::setEnableMode(uint32_t mode)
{
    enable_mode_ = (mode == ENABLE_MODE_EXTERNAL_PIN) ? ENABLE_MODE_EXTERNAL_PIN : ENABLE_MODE_HOST;
    // 切换来源时先关 EN：避免"外部引脚是低"时一改配置电机就自己转起来。
    // 之后由主机显式 0x124，或由外部引脚拉低来重新使能。
    if (driver_ != nullptr)
    {
        driver_->setEnable(false);
    }
    driver_enabled_ = false;
    external_en_level_ = (gpio_input_data_bit_read(EXT_EN_PORT, EXT_EN_PIN) != RESET) ? 1U : 0U;
    // 切换使能来源：去抖状态也按当前引脚电平重置，避免带着旧模式的候选电平进入新模式。
    external_en_stable_ = external_en_level_;
    external_en_candidate_ = external_en_level_;
    external_en_change_count_ = 0U;
}

uint32_t ClosedLoopController::getEnableMode() const
{
    return enable_mode_;
}

bool ClosedLoopController::isDriverEnabled() const
{
    return driver_enabled_;
}

uint32_t ClosedLoopController::getExternalEnableLevel() const
{
    // 引脚没有参与控制时返回 2 = 不适用（只有内部模式 + host 使能才是这种情况）
    return enableAuthorityIsExternalPin() ? static_cast<uint32_t>(external_en_level_) : 2U;
}

// 【EN 使能权威判定】两种情况下外部 EN 引脚（PC13）是使能权威：
//   ① 用户显式选择"外部 EN 引脚"模式（0x161=1，任何运动源下都有效）；
//   ② 运动源是外部脉冲跟随（0x16D=1，上电默认）——跟随模式的语义就是
//     "PLC 说了算"：开机/切换只决定通道归谁，通不通电流看 PLC 的 EN 线
//     （2026-11-10 用户明确：不是开机强制使能，是否使能看外部 EN 电平）。
// 权威是引脚时，上位机只能"撤销失能锁"（0x124 清 user_disabled_），
// 不能隔着 PLC 强行使能；失能（0x125/急停/故障）永远有效。
bool ClosedLoopController::enableAuthorityIsExternalPin() const
{
    return enable_mode_ == ENABLE_MODE_EXTERNAL_PIN || motion_source_ == MOTION_SOURCE_EXTERNAL;
}

// 上位机/故障路径统一走这里：引脚权威模式下"使能"请求被忽略（引脚说了算），
// 但"失能"永远有效（安全动作不能被配置挡住）。
void ClosedLoopController::requestDriverEnable(bool want)
{
    if (driver_ == nullptr)
    {
        return;
    }
    if (enableAuthorityIsExternalPin() && want)
    {
        return;
    }
    driver_->setEnable(want);
    driver_enabled_ = want;
}

// 每拍调用（rampUpdate 开头）。只有外部模式才真正做事，而且只在**去抖后**电平变化时写 GPIO。
void ClosedLoopController::updateEnableFromExternalPin()
{
    const uint8_t level = (gpio_input_data_bit_read(EXT_EN_PORT, EXT_EN_PIN) != RESET) ? 1U : 0U;
    external_en_level_ = level;

    // 【消抖】连续 kEnDebounceSamples 拍读到同一个新电平才更新 stable_。
    // external_en_level_（原始值）仍然每拍刷新，供 0x164 观察毛刺；stable_ 才是控制依据。
    if (level != external_en_stable_)
    {
        if (level == external_en_candidate_)
        {
            if (++external_en_change_count_ >= kEnDebounceSamples)
            {
                external_en_stable_ = level;
                external_en_change_count_ = 0U;
            }
            else
            {
                // 还没数够：毛刺可能马上就回去了，先不动 GPIO。
                return;
            }
        }
        else
        {
            external_en_candidate_ = level;
            external_en_change_count_ = 1U;
            return;
        }
    }
    else
    {
        // 回到稳定电平：撤销未完成的候选。候选存在过又没数满就回去了 = 毛刺，
        // 计数让上位机看得见（0x16C）：现场干扰排查全靠它判断 EN 线质量。
        if (external_en_change_count_ > 0U)
        {
            en_glitch_count_++;
        }
        external_en_change_count_ = 0U;
        external_en_candidate_ = external_en_stable_;
    }

    if (driver_ == nullptr || !enableAuthorityIsExternalPin())
    {
        return;
    }
    const bool want = (external_en_stable_ == 0U) && !user_disabled_ &&
                      !fault_monitor_.outputStopped() && !stall_detected_;
    if (want != driver_enabled_)
    {
        // 【这里绝不能 printf】rampUpdate 是 20kHz 控制任务，一行 printf 就是几毫秒停摆。
        driver_->setEnable(want);
        driver_enabled_ = want;
    }
}

void ClosedLoopController::requestQuickStop()
{
    // 角度模式（位置伺服）没有"固定行程终点"可以慢慢减到 0，
    // 对它而言"减速停止"就是立即停：直接走 stopMotion（脉冲断、状态清，EN 保持）。
    if (closed_loop_angle_mode_enabled_)
    {
        stopMotion();
        return;
    }
    // 规划运动：置标志，由 updateTrapezoid 在下个节拍边界原子消费（本拍是 USB 任务，
    // 不能直接改梯形状态机）。非运动态的请求**直接拒绝**：锁存下来会让下一次
    // startMotion 的第一拍就急停，表现成"启动后立刻抽搐一下就停"，极难排查。
    if (!motion_running_)
    {
        printf("[QUICKSTOP] ignored: no motion is running\r\n");
        return;
    }
    quick_stop_requested_ = true;
}

bool ClosedLoopController::isQuickStopActive() const
{
    return motion_quick_stop_;
}

// 急停（0x16E=1，网页右下角红钮）：立即断脉冲 + 失能驱动器。
// 为什么失能：步进电机在高速运行中被"断脉冲但保持励磁"时，绕组里感应的
// 反电动势会经 H 桥泵电流回驱动芯片（伤机的那一下）；EN 拉低后 H 桥高阻，
// 转子带着绕组自由滑行，电气上最温和。机械代价是失去保持矩（垂直轴会掉），
// 这是"急停"本身的语义，不是本实现的选择。
void ClosedLoopController::emergencyStop()
{
    closed_loop_angle_mode_enabled_ = false;
    stopMotion();
    // 与 0x125 相同的"用户主动失能"锁：之后自动测试循环里的 0x11F=1 不会
    // 把电流偷偷打开，必须显式 0x124 重新使能。
    user_disabled_ = true;
    requestDriverEnable(false);
    // 调用方是 USB/串口任务（0x16E 处理路径），printf 安全。
    printf("[ESTOP] emergency stop: pulses cut, driver disabled\r\n");
}

// 切换运动源（0x16D）。外部↔内部是**信号通道的交还/接管**：
// enableExternalPulseFollow() 本身已经做了两套状态机的互清（共用 DDA/TMR2），
// 这里补的是"意图"层：外部模式下 startMotion/setTargetAngleDeg 会被
// follower 的 isEnabled() 门拒绝（那里已有），0x149 与本寄存器保持一致由
// ParameterRouter 在两侧同步。
// 【切换不碰 EN】进入外部模式只是"进模式"，驱动器通不通电流由外部 EN 引脚
// （PC13）决定（见 enableExternalPulseFollow）；退出到内部模式也不动 EN，
// 由 0x161 使能来源策略接管（host：等 0x124；external：仍看引脚）。
void ClosedLoopController::setMotionSource(uint32_t source)
{
    const uint32_t src = (source == 0U) ? MOTION_SOURCE_INTERNAL : MOTION_SOURCE_EXTERNAL;
    if (src == motion_source_ &&
        external_pulse_follower_.isEnabled() == (src == MOTION_SOURCE_EXTERNAL))
    {
        return;   // 状态一致，不折腾（避免每次连接重放一遍切换日志）
    }
    motion_source_ = src;
    if (src == MOTION_SOURCE_EXTERNAL)
    {
        // 外部模式优先保证"通道立即交还"：运动中切过来会瞬间断脉冲，
        // 反电动势风险同 emergencyStop 的说明，但"切换"本身就是用户主动动作。
        stopMotion();
        enableExternalPulseFollow(true);
    }
    else
    {
        enableExternalPulseFollow(false);
    }
    printf("[SOURCE] motion source = %s\r\n",
           src == MOTION_SOURCE_EXTERNAL ? "EXTERNAL PULSE FOLLOW (internal motion locked, EN by pin)"
                                         : "INTERNAL PLAN (external PUL ignored)");
}

// 校准/整定入口的公共前置：外部跟随模式下接管运动通道切到内部规划。
// 与手动切换（router 0x16D）走同一个 setMotionSource，互斥语义完全一致；
// 区别只有两点：① 由固件在校准开始时自动执行；② 打印 [CALIB] 让日志能解释
// "为什么运动源自己变了"。调用方：0x136 磁编码器校准。网页的滞后角/冲击/PID
// 整定入口在前端先写 0x16D=0（带界面同步），固件侧不需要再兜一层。
void ClosedLoopController::ensureInternalSourceForCalibration(const char* who)
{
    if (motion_source_ == MOTION_SOURCE_INTERNAL && !external_pulse_follower_.isEnabled())
    {
        return;   // 已在内部模式，不重放日志
    }
    printf("[CALIB] %s: motion channel taken for INTERNAL (was external follow)\r\n", who);
    setMotionSource(MOTION_SOURCE_INTERNAL);
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

// ===========================================================================
// 启动一次规划运动（由 0x11F=1 触发）。
//
// 做三件事，顺序不能换：
//   ① 退出其它模式：强制 closed_loop_angle_mode_enabled_ = false，
//      并 step_output_.stop() 停掉残留的 DMA 脉冲序列。
//      角度模式会把该标志置 true 并让 updateAngleMode 每拍接管速度输出，
//      不清就会和 updateTrapezoid 抢 current_step_speed_（实测严重抖动）。
//   ② 状态归零：上位机启动只发 0x11F=1，不会先调 stopMotion，
//      所以上次运动/角度模式留下的一切（速度、阶段、行程累计、待发 DMA、换向标志、
//      PID 积分与微分）都必须在这里自己清，不能指望别处。
//   ③ 参数兜底与重算：motion_* 为 0 时给默认值，再用它们重算
//      motion_max_step_s_ / motion_accel_step_s2_ / k_step_pulse_ticks。
//      这样即使上位机漏发参数，也不会出现“电机完全不动”。
// ===========================================================================
void ClosedLoopController::startMotion()
{
    // 【必须退出角度模式】角度模式（setTargetAngleDeg）会把 closed_loop_angle_mode_enabled_
    // 置 true 并让 updateAngleMode 每拍接管速度输出。规划运动启动时如果不把它清掉，
    // updateAngleMode 和 updateTrapezoid 会同时改 current_step_speed_，互相争夺：
    // 实测表现为“开过一次角度模式后，开环规划运动严重抖动，必须重启硬件才恢复”。
    // 【诊断，默认关闭】这行约 90 字节，115200 下阻塞约 8ms。
    // 磁编码器校准每段移动都会调 startMotion，1024 段就是 8 秒多纯串口输出，
    // 实测表现为“校准变得很慢”。需要排查残留状态时把下面宏打开（或临时取消注释）。
    // #define DEBUG_MOTION_START_TRACE 1
#ifdef DEBUG_MOTION_START_TRACE
    printf("[START] ang=%u cur=%.0f max=%.0f acc=%.0f stage=%u dma=%ld deliv=%ld\r\n",
           static_cast<unsigned>(closed_loop_angle_mode_enabled_),
           static_cast<double>(current_step_speed_),
           static_cast<double>(motion_max_step_s_),
           static_cast<double>(motion_accel_step_s2_),
           static_cast<unsigned>(motion_ramp_stage_),
           static_cast<long>(motion_dma_pending_signed),
           static_cast<long>(motion_steps_delivered_signed));
#endif

    closed_loop_angle_mode_enabled_ = false;

    // 【急停/换向残留必须在这里清】motion_quick_stop_ 被 stopMotion/故障打断时可能
    // 停在 DECEL 段没走完：残留 true 会让下次运动的 DECEL 尾把 motion_pulse_count_
    // 截成"已发步数"，表现为 25220/25600 那类欠步；残留的 steps_to_decel_=0xFFFFFFFF
    // 会让 ACCEL 分支（没有哨兵检查）第一拍就转 DECEL；turnaround_skip 残留会让
    // 下次运动前 100ms 的评分被莫名屏蔽。一次运动不该继承上一次的任何"意图"。
    quick_stop_requested_ = false;
    motion_quick_stop_ = false;
    motion_turnaround_skip_ramps_ = 0U;

    // 【堵转锁存】判定堵转后必须显式清（0x15F=1）才能再启动，否则自动测试循环
    // 会无视故障继续发脉冲 —— 电机被按住时这就等于一直灌电流发热。
    if (stall_detected_)
    {
        printf("[SAFETY] startMotion rejected: stall latched (write 0x15F=1 to clear)\r\n");
        return;
    }
    // 每次启动重新布防堵转判据：上一个动作结束时的静默计时不能带进新动作。
    stall_armed_ = false;

    // 【外部 EN 模式】引脚是高电平 = 驱动器被外部禁用，这时不能启动运动：
    // 否则固件会照常发脉冲（电机没电流不转），堵转保护反而会误判。
    if (enable_mode_ == ENABLE_MODE_EXTERNAL_PIN && external_en_level_ != 0U)
    {
        printf("[SAFETY] startMotion rejected: external EN pin is HIGH (disabled)\r\n");
        return;
    }

    // 【必须停掉残留的 DMA 脉冲序列】step_output_.stop() 原来只在 stopMotion 里调，
    // 而实际使用中很容易“进了角度模式不点停止就直接切回规划运动”。
    // 此时 TMR2/DMA 里还排着角度模式最后几拍的脉冲序列，规划运动一开始发新序列
    // 就会和旧序列冲突 -> 实测表现为“用过角度模式后规划模式一直抖，重启才恢复”。
    step_output_.stop();

    // 【DMA 记账也必须归零】上一步只停了定时器，但：
    //   motion_dma_pending_signed : 上一拍交给 TMR2/DMA 但还没并入“已发完”的脉冲数
    //   motion_steps_delivered_signed : 已发完的累计值
    // 两个不清的话，startMotion 后第一次 refreshEncoderFeedback 会把上次的待发步数
    // 直接加进 motion_steps_delivered_signed —— 命令行程瞬间跳一大截，
    // follow_error 从错误基准开始算，闭环补偿跟着抽。
    motion_dma_pending_signed = 0;
    motion_dma_pending_steps_ = 0U;
    motion_steps_delivered_signed = 0;

    // 【参数兜底】下面会用 motion_max_rpm_ / motion_accel_rpm_s_ 重算 step 速度，
    // 这两个为 0 时会算出 0 -> 电机完全不动。上位机正常每次都发，
    // 但之前的 setTargetAngleDeg 已经有同样的兜底，两个入口保持一致。
    if (motion_max_rpm_ <= 0.0f)
    {
        motion_max_rpm_ = 1200.0f;
    }
    if (motion_accel_rpm_s_ <= 0.0f)
    {
        motion_accel_rpm_s_ = 6000.0f;
    }
    if (motion_start_rpm_ <= 0.0f)
    {
        motion_start_rpm_ = motion_max_rpm_;
    }

    // 【安全锁】用户主动失能后拒绝任何启动（含自动测试循环的 0x11F=1），
    // 必须重新收到 MOTOR_ENABLE 才能清除。见 parameter_router 里 MOTOR_DISABLE 分支。
    if (user_disabled_)
    {
        printf("[SAFETY] startMotion rejected: motor disabled by user\r\n");
        return;
    }

    // 【运动源绝对互斥】外部跟随模式（0x16D=1，上电默认）下拒绝一切内部运动：
    // 命令位置归属外部 PUL/DIR，两套逻辑同时跑必然打架。
    // 要用规划/角度模式，先写 0x16D=0（网页切「内部规划模式」）。
    if (external_pulse_follower_.isEnabled() || motion_source_ == MOTION_SOURCE_EXTERNAL)
    {
        printf("[SOURCE] startMotion rejected: motion source is EXTERNAL follow "
               "(write 0x16D=0 / switch to internal plan first)\r\n");
        return;
    }

    // 【连续运行模式】速度模式 + pulseCount=0 表示“无限跑”：
    // 滞后角标定 / PID 扫速需要电机一直转，随时改 0x11B 改速度、改 0x11E 改方向，
    // 不能每到一段就停机重启（那样每点都要重新从 0 加速，完全没法用）。
    // 用 0xFFFFFFFF 会让 updateTrapezoid 里的“剩余步数”永远远大于减速点，
    // 所以永远不进 DECEL，直到上位机主动改速度或停止。
    if ((motion_mode_ == MOTION_MODE_VELOCITY_FORWARD || motion_mode_ == MOTION_MODE_VELOCITY_REVERSE) &&
        motion_pulse_count_ == 0U)
    {
        motion_pulse_count_ = 0xFFFFFFFFU;
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
    // 对齐起始速与目标速（原来方向反了：max<start 时把 max 抬到 start——
    // 用户设 10RPM 被悄悄抬成 60，表现为"减速指令无效、永远卡在 60"）。
    // 速度模式的语义是"目标=用户意图"，应该把起步降下来对齐。
    if (motion_max_rpm_ < motion_start_rpm_)
    {
        motion_start_rpm_ = motion_max_rpm_;
    }

    // 【共振带＝信息标记，不钳位】（2026-11-12 改版，用户设计裁定）
    // 旧版把巡航目标落进带内悄悄钳到 lo——换了好电机后用户设 550 被钳 300，
    // 看起来像转速 bug。带是"某些工况下检测到的热区记录"，不是运动否决权：
    // 任何正常运动（内部规划/外部跟随/角度模式）一律照用户设定跑，进带只打印
    // 警告（转动中不打印：这里在 startMotion 起点、脉冲未发，允许）。
    // 0x16F/0x170 保留（扫描写入、UI 显示）；0x171 旁路位成为历史（无钳可旁）。
    if (resonance_hi_rpm_ > resonance_lo_rpm_ &&
        motion_max_rpm_ > static_cast<float>(resonance_lo_rpm_) &&
        motion_max_rpm_ < static_cast<float>(resonance_hi_rpm_) &&
        calibration_stage_ == CALIBRATION_IDLE)
    {
        printf("[RESONANCE] cruise %u rpm inside marked band [%u,%u] - running anyway (band is informational)\r\n",
               static_cast<unsigned>(motion_max_rpm_),
               static_cast<unsigned>(resonance_lo_rpm_),
               static_cast<unsigned>(resonance_hi_rpm_));
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
    // 起步屏蔽：让位置/速度环在最初的 20ms 里不动作（见头文件注释）
    motion_start_blank_ramps_ = kMotionStartBlankRamps;
    const uint32_t micro_steps_per_round = getMicroStepsPerRound(driver_);
    const float rpm2step = static_cast<float>(micro_steps_per_round) / 60.0f;

    if (driver_ != nullptr && !fault_monitor_.encoderFaultActive() && !fault_monitor_.magneticFaultActive())
    {
        if (user_disabled_)
        {
            printf("[SAFETY] setTargetAngleDeg rejected: motor disabled by user\r\n");
            return;
        }
        requestDriverEnable(true);
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
    // [DEBUG-OFF] printf("v0: %.6f, vmax: %.6f, a: %.6f\n", v0, vmax, a);
        const bool alternating_motion = motion_mode_ == MOTION_MODE_ALTERNATING_FORWARD ||
                        motion_mode_ == MOTION_MODE_ALTERNATING_REVERSE;
        motion_leg_pulse_count_ = alternating_motion ? (motion_pulse_count_ + 1U) / 2U : motion_pulse_count_;
        accel_total_steps_ = static_cast<uint32_t>((vmax * vmax - v0 * v0) / (2.0f * a));

        // 减速段：从vmax减速到0，加速度大小同样a
        decel_total_steps_ = static_cast<uint32_t>((vmax * vmax) / (2.0f * a));
    // [DEBUG-OFF] printf("original accel_total_steps_: %lu\n", accel_total_steps_);
    // [DEBUG-OFF] printf("original decel_total_steps_: %lu\n", decel_total_steps_);

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

            // 【默认关闭】校准每段短行程都走三角曲线分支，这两行在控制任务里
            // 每段阻塞约 8ms（startMotion 由校准器在 rampUpdate 里调）。
#ifdef DEBUG_MOTION_START_TRACE
            printf("v_peak_sq: %.6f\n", v_peak_sq);
            printf("v_peak: %.6f\n", v_peak);
#endif
        }
// [DEBUG-OFF] printf("total_req_steps: %lu\n", total_req_steps);
    // [DEBUG-OFF] printf("accel_total_steps_: %lu\n", accel_total_steps_);
    // [DEBUG-OFF] printf("decel_total_steps_: %lu\n", decel_total_steps_);
    // [DEBUG-OFF] printf("cruise_total_steps_: %lu\n", cruise_total_steps_);
    // [DEBUG-OFF] printf("positionPID: %f,%f,%f\n", position_pid_.kp, position_pid_.ki, position_pid_.kd);
    // [DEBUG-OFF] printf("velocityPID: %f,%f,%f\n", velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd);

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

        // 速度模式一律走连续 PWM，**与计数无关**：带计数时的"跑完"判定靠
        // motion_steps_emitted_ 账本（两条出口共用同一账本，机制不变）。
        // 旧条件限定 count=0 才启用——网页启动永远写非零脉冲数，速度模式被
        // 打回窗口路径，撞容量墙 440k pps≈2062RPM@64细分，表现为"网页最高
        // 2000 转、脚本能上 2700"（2026-11-12 实测定位）。
        motion_planner_.velocity_pwm_active_ =
            (motion_mode_ == MOTION_MODE_VELOCITY_FORWARD ||
             motion_mode_ == MOTION_MODE_VELOCITY_REVERSE);
        motion_planner_.velocity_pwm_running_ = false;
        // 预同步沉降：速度模式启动后先静置（不发脉冲），让使能拉入振荡熄灭，
        // 再进梯形斜坡——消除冷启动一次性拉入丢步（~43°，A/B 实测，见 0x179）。
        presync_remaining_ns_ = motion_planner_.velocity_pwm_active_
            ? static_cast<uint64_t>(start_dwell_ms_) * 1000000ULL : 0ULL;

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

float ClosedLoopController::refreshEncoderFeedback(uint64_t now_ns)
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

    // 速度估计放在这里：所有模式（角度/规划/外部跟随/沉降）都先经过 refreshEncoderFeedback，
    // 所以整个控制器只有一份速度估计，不再有"每个分支各自做一次 0.9/0.1 低通"的三份实现。
    updateSpeedEstimate(now_ns);

    if (motion_running_ && motion_encoder_reference_valid_)
    {
        const int16_t encoder_delta_raw = normalize_signed_delta_deg(static_cast<int32_t>(encoder_filtered_raw_) - static_cast<int32_t>(motion_encoder_previous_raw_));
        motion_encoder_travel_raw_ += encoder_delta_raw;
        // 下一步才计算上一步的理论步数（因为用了定时器后，脉冲是异步发出的， 直接算就等于理论步数领先实际了，所以放到下一ramp才计算理论跑了多少
        const float step_angle_deg = 360.0f / static_cast<float>(getMicroStepsPerRound(driver_));
        // 用"已确认发完"的步数，而不是"已交给 DMA"的步数，命令位置才和真实脉冲同步
        motion_commanded_travel_deg_ = static_cast<float>(motion_steps_delivered_signed) * step_angle_deg;

        // 【已知精度限制（长时运行才暴露）】motion_encoder_travel_raw_ 是 int64（可跑 147 年不溢出），
        // 但这里转成 float 算角度：float 只有 24 位尾数，超过约 1.6e7 LSB（合 244 圈）后
        // 分辨率就到不了 1 LSB，再往上（比如几百万圈）follow_error 会明显失真。
        // 当前所有使用场景都是短时运动（几十秒、几千圈以内），精度损失 < 0.1°。
        // 若将来要做“连续运行几天”的长时定位，需要把 travel_angle / commanded_travel_deg /
        // follow_error 整条链改成 int64 毫度（或 double）——只在显示时转 float，不在此处局部改。
        auto motion_encoder_travel_angle_ = static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F;
        motion_follow_error_deg_ = normalize_signed_angle_error_deg(
            motion_encoder_travel_angle_ - motion_commanded_travel_deg_);  // 比如命令要走3度，结果才走了2度，那跟随误差就是-1度。实际-目标
    }
    motion_encoder_previous_raw_ = encoder_filtered_raw_;

    return encoder_delta_deg;
}

// ===========================================================================
// 速度估计：约 1ms 固定窗口上的滑窗差分
//
// 为什么不能用「相邻两拍角差 / dt + 0.9/0.1 低通」：
//   编码器 16 位/圈 -> 1 LSB = 0.0055°。50us 内走 1 LSB 就是 110°/s = 18.3RPM，
//   也就是说单拍微分在低速段的分辨率只有 18RPM，而低通只能压 RMS、压不掉台阶，
//   代价还是 1ms 量级的群延迟。速度环拿这种反馈去整定，永远是"要么抖要么迟钝"。
//
// 现在：把最近 1ms 的角度差除以真实经过时间。量化噪声按窗口长度缩小 20 倍
// （18.3 -> 约 0.9RPM），群延迟固定为半个窗口（0.5ms）且是线性的，速度环的
// 增益才有明确意义。窗口里用的是滤波后的浮点角（含亚 LSB 小数），不是整数 raw。
// ===========================================================================
float ClosedLoopController::updateSpeedEstimate(uint64_t now_ns)
{
    const uint32_t now32 = static_cast<uint32_t>(now_ns);
    speed_angle_ring_[speed_ring_index_] = encoder_filtered_angle_;
    speed_time_ring_[speed_ring_index_] = now32;
    const uint16_t newest = speed_ring_index_;
    speed_ring_index_ = static_cast<uint16_t>((speed_ring_index_ + 1U) % kSpeedWindowTicks);
    if (speed_ring_count_ < kSpeedWindowTicks)
    {
        speed_ring_count_++;
    }

    // 窗口还没攒够 4 拍时，单拍差分的噪声太大，宁可直接返回上一拍的值。
    if (speed_ring_count_ < 4U)
    {
        return encoder_speed_rpm_;
    }

    const uint16_t oldest = static_cast<uint16_t>(
        (speed_ring_index_ + kSpeedWindowTicks - speed_ring_count_) % kSpeedWindowTicks);
    const uint32_t window_ns = speed_time_ring_[newest] - speed_time_ring_[oldest];
    // 时间窗异常（首拍 / 任务被长时间阻塞）时不要把噪声当速度：保持上一拍。
    if (window_ns < 20000U || window_ns > 5000000U)
    {
        return encoder_speed_rpm_;
    }

    const float delta_deg = normalize_signed_delta_deg(
        encoder_filtered_angle_ - speed_angle_ring_[oldest]);
    const float rpm_raw = delta_deg * (60.0f / 360.0f) * 1.0e9f / static_cast<float>(window_ns);
    if (!std::isfinite(rpm_raw))
    {
        return encoder_speed_rpm_;
    }

    // 【为什么还要再做一次 EMA】
    // 1ms 端点差分仍把编码器的“每步振铃”当速度：实测静止时速度读数 σ≈7RPM、
    // 匀速 100RPM 时 σ≈25RPM（对照：同时刻的位置误差只有 0.19°，说明那不是真在变速，
    // 是转子每走一个微步的机械振铃 + 传感器噪声）。速度环拿这种反馈，
    // 增益一大就把噪声放大成脉冲抖动 —— 这正是“速度环动一点点就抖”的另一半原因。
    // 3ms 一阶平滑把 100Hz 量级的振铃压掉，同时只增加 3ms 延迟：
    // 速度环本来就只做阻尼，带宽不需要高。
    constexpr float kControlSpeedAlpha = 0.0167f;   // 50us / 3ms
    encoder_speed_rpm_ += (rpm_raw - encoder_speed_rpm_) * kControlSpeedAlpha;

    // 显示/波形用的再慢一档（20ms）：上位机 25Hz 采样看到的转速不再是一团毛刺。
    constexpr float kDisplaySpeedAlpha = 0.0025f;   // 50us / 20ms
    encoder_speed_display_rpm_ += (encoder_speed_rpm_ - encoder_speed_display_rpm_) * kDisplaySpeedAlpha;

    // 滞后角表用的速度：5ms 平滑。
    // 【曾经是 30ms，太慢了】短行程（1 圈 0.4s）里起步段的滞后角估计要 100ms 才跟上，
    // 这段时间 PID 以为"落后"而多发脉冲 —— 实测 1 圈在 120→1000RPM 参数下多转 0.76°，
    // 2 圈以上反而不明显（暂态占比小）。5ms 足够压噪声，又能在 15ms 内跟上速度阶跃。
    constexpr float kLagSpeedAlpha = 0.01f;
    lag_angle_speed_rpm_ += (encoder_speed_rpm_ - lag_angle_speed_rpm_) * kLagSpeedAlpha;
    return encoder_speed_rpm_;
}

void ClosedLoopController::resetSpeedEstimate()
{
    speed_ring_index_ = 0U;
    speed_ring_count_ = 0U;
    encoder_speed_rpm_ = 0.0f;
    encoder_speed_display_rpm_ = 0.0f;
    lag_angle_speed_rpm_ = 0.0f;
    lag_angle_applied_deg_ = 0.0f;
    motion_lag_angle_deg_ = 0.0f;
    last_measured_along_rps_ = 0.0f;
    last_angle_mode_measured_rps_ = 0.0f;
    velocity_correction_lpf_ = 0.0f;
    follow_error_lpf_deg_ = 0.0f;
}

// ===========================================================================
// 滞后角（lag angle）：把“物理上必然的滞后”从跟随误差里剔除
//
// 关键澄清（这次改名要解决的就是这个误解）：
//   步进电机稳态运行时，转子必须落后指令磁场一个角度才能产生转矩拖动负载。
//   这个角只由转速/转矩决定，**不是丢步**，脉冲发完以后转子自己会追上来。
//   如果把跟随误差里的这部分也交给 PID 去“补偿”，PID 就会持续多发脉冲，
//   转子于是永远跑在“指令位置 - 滞后角”这个不存在的目标前面 —— 也就是用户
//   观察到的“多走了 / 超前了”。
//   正确做法是：把表值当作**已知的、不该被修正的偏差**减掉，PID 只处理残差。
//
// 这里的处理细节：
//   - 速度用 lag_angle_speed_rpm_（8ms 平滑），不用 20kHz 瞬时值：表是稳态映射，
//     跟着噪声抖只会把噪声灌进误差通道；
//   - 低于 6RPM 线性淡出到 0：拟合截距/噪声在这个区间会让符号翻转，
//     而低速时真实滞后角本来就只有零点零几度；
//   - 一阶平滑 + 40°/s 的变化率限幅：换向/起步时符号会变，不能让一个 3° 的
//     阶跃直接跳进控制环；
//   - 表未标定时恒为 0（isFitted()==false），行为等于“不剔除”，与旧版一致。
// ===========================================================================
void ClosedLoopController::updateLagAngleFeedforward(float measured_rps, float commanded_rps)
{
    float target_deg = 0.0f;
    if (lag_angle_.isFitted())
    {
        // 【速度取"指令速度"和"实测速度"里较大的那个】
        // 表的物理含义是"在某个磁场转速下转子要落后多少"，而磁场转速就是**指令速度**
        //（脉冲频率），不是实测速度。只用实测速度有两个坑：
        //  1) 起步瞬间实测还没起来，滞后角估计是 0，PID 以为落后而多发脉冲 ——
        //     实测 1 圈（start=120/max=1000）多转 0.76°，改用指令速度后降到 0.5° 以内；
        //  2) 实测速度有 ~8RPM 噪声，会顺着滞后角灌进误差通道。
        // 取 max 是为了收尾段：指令先到 0，实测还在滑行，这时仍应按实测给滞后角。
        const float speed_rpm = fmaxf(fast_abs(measured_rps), fast_abs(commanded_rps)) * 60.0f;
        constexpr float kLagFullSpeedRpm = 6.0f;
        const float weight = fast_clamp(speed_rpm / kLagFullSpeedRpm, 0.0f, 1.0f);
        const auto speed_u16 = static_cast<uint16_t>(speed_rpm > 65535.0f ? 65535.0f : speed_rpm);
        const float magnitude_deg =
            static_cast<float>(lag_angle_.getLagAngleMilliDegrees(speed_u16)) / 1000.0f;
        // 方向：优先看指令（起步时实测还是 0），停下时再看实测
        const float dir_source = (fast_abs(commanded_rps) > 1.0e-4f) ? commanded_rps : measured_rps;
        const float sign = (dir_source >= 0.0f) ? -1.0f : 1.0f;
        target_deg = sign * magnitude_deg * weight * lag_angle_feedforward_ratio_;
        // 上限钳位：防止标定数据异常或速度外推把前馈打爆（1200RPM 实测约 3°）。
        constexpr float kMaxLagAngleCompDeg = 5.0f;
        target_deg = fast_clamp(target_deg, -kMaxLagAngleCompDeg, kMaxLagAngleCompDeg);
    }

    constexpr float kSmoothing = 0.02f;      // 20kHz 下约 2.5ms
    // 变化率限幅：200°/s。原来是 40°/s（1.5° 要 37ms），短行程起步段跟不上；
    // 现在 1.5° 只要 7.5ms，配合上面 5ms 的速度平滑，起步 20ms 内滞后角就到位。
    constexpr float kMaxStepDeg = 0.01f;
    float next_deg = lag_angle_applied_deg_ + (target_deg - lag_angle_applied_deg_) * kSmoothing;
    const float delta_deg = next_deg - lag_angle_applied_deg_;
    if (delta_deg > kMaxStepDeg)
    {
        next_deg = lag_angle_applied_deg_ + kMaxStepDeg;
    }
    else if (delta_deg < -kMaxStepDeg)
    {
        next_deg = lag_angle_applied_deg_ - kMaxStepDeg;
    }
    lag_angle_applied_deg_ = next_deg;
    // 遥测/波形用的就是“实际生效值”，不是原始表值：两者不一致时以这里为准。
    motion_lag_angle_deg_ = lag_angle_applied_deg_;
}

int32_t ClosedLoopController::effectiveLagAngleMilliDeg() const
{
    return static_cast<int32_t>(motion_lag_angle_deg_ * 1000.0f);
}

// ===========================================================================
// 堵转 / 不转保护
//
// 为什么需要：自动整定会连续跑几十次运动，一旦电机被按住、相线松了、电流太小，
// 固件仍然会“按规划发脉冲”，表面数据全对，实际什么都没动 —— 整定器会把它
// 当成一次“跟随误差极大”的试验，把 PID 越调越凶。必须由编码器给出独立判据。
//
// 判据：命令速度 > 8RPM（确实在发脉冲）且连续 200ms 编码器原始角变化 < 2LSB。
//   反例校验：20RPM 时 1ms 也有 6.5LSB，200ms 就是 1300LSB，不可能落在 2LSB 内。
// 动作：停机、关脉冲、失能（避免持续灌电流发热），并置 FAULT_STALL_DETECTED；
//       必须显式写 0x15F=1 才能清除。
// ===========================================================================
void ClosedLoopController::updateStallGuard(uint64_t now_ns)
{
    if (stall_detected_)
    {
        return;
    }

    const uint32_t now32 = static_cast<uint32_t>(now_ns);
    const float command_rps = fast_abs(current_step_speed_) / static_cast<float>(getStepsPerRound());
    const bool commanding = motion_running_ && (command_rps * 60.0f) >= kStallMinCommandRpm;
    if (!commanding)
    {
        stall_armed_ = false;
        return;
    }

    if (!stall_armed_)
    {
        stall_armed_ = true;
        stall_last_raw_ = encoder_filtered_raw_;
        stall_quiet_since_ns_ = now32;
        return;
    }

    // 【判据必须放在"整段位移"上，不能放在逐拍增量上】
    // 这台编码器静止时的读数峰峰值约 26~31 LSB（传感器/EMI 噪声），逐拍增量
    // 经常超过 2 LSB —— 用"每拍变化 < 2LSB"判静止，噪声会不停把计时器清零，
    // 堵转永远触发不了（实测把电流置 0 让电机完全不动，守卫依然不动作）。
    // 现在看窗口内的**净位移**（窗口=stall_quiet_ms_，默认 200ms）：
    //   静止：噪声只在 ±15 LSB 内，净位移不会超过 ~100 LSB（阈值 stall_move_lsb_）；
    //   最低速 8RPM：200ms 也走 8/60*65536*0.2 = 1750 LSB。
    // 两者差 17 倍，阈值 100 LSB 非常安全。阈值可经 0x174/0x175 调（见成员注释）。
    const int32_t move_lsb = static_cast<int32_t>(stall_move_lsb_);
    const int32_t span = static_cast<int32_t>(encoder_filtered_raw_) -
                         static_cast<int32_t>(stall_last_raw_);
    const int32_t wrapped_span = (span > 32767) ? (span - 65536)
                               : (span < -32768) ? (span + 65536) : span;
    // 【只认命令方向上的净前进，回摆不算】—— 这是修复"原地震不报堵转"的关键。
    // 失步/共振时转子在原地来回振荡，编码器读数在 ±数百 LSB 间大幅摆动。旧判据
    // `span > +move || span < -move` 双向都重置静止计时：回摆(反号)把计时清零，
    // 前进(正号)虽也重置但把基线推到振荡上包络——基线卡在上沿后再难被更大的同向
    // 位移重置，可一旦振荡相位先走到回摆侧，计时又归零，于是静止时长永远攒不满
    // 200ms，堵转永远不锁（实测正反馈振荡的电机"该报不报"）。
    // 改成只统计命令方向投影 progress = span * dir：回摆时 progress 为负，绝不重置，
    // 计时单调累加 → 原地震必然锁存。真运动时 progress 每窗远超 move_lsb，不误报。
    const int32_t dir = (current_step_speed_ >= 0.0f) ? 1 : -1;
    const int32_t progress = wrapped_span * dir;
    if (progress > move_lsb)
    {
        stall_last_raw_ = encoder_filtered_raw_;
        stall_quiet_since_ns_ = now32;
        return;
    }

    const uint32_t quiet_ns = stall_quiet_ms_ * 1000000U;
    if ((now32 - stall_quiet_since_ns_) < quiet_ns)
    {
        return;
    }

    stall_detected_ = true;
    stall_events_++;
    printf("[STALL] no encoder motion for %lums while commanding %.1fRPM (raw=%u) -> stop + disable\r\n",
           static_cast<unsigned long>(stall_quiet_ms_),
           static_cast<double>(command_rps * 60.0f),
           static_cast<unsigned>(encoder_filtered_raw_));
    closed_loop_angle_mode_enabled_ = false;
    motion_running_ = false;
    stopMotion();
    fault_monitor_.reportStall(true);
    if (driver_ != nullptr && stall_auto_disable_)
    {
        driver_->setEnable(false);
    }
    syncProtocolTelemetry();
}

bool ClosedLoopController::isStallDetected() const
{
    return stall_detected_;
}

void ClosedLoopController::clearStall()
{
    stall_detected_ = false;
    stall_armed_ = false;
    fault_monitor_.reportStall(false);
}

void ClosedLoopController::setStallAutoDisable(bool enable)
{
    stall_auto_disable_ = enable;
}

uint32_t ClosedLoopController::getStallEvents() const
{
    return stall_events_;
}



void ClosedLoopController::updateExternalPulseChannel(uint64_t now_ns, float encoder_delta_deg)
{
    // 注意：delta_ramp_ns 在主路径稍后才算，这里自己取一次，
    // 并限幅——首拍或长时间没进 rampUpdate 时不能拿一个巨量时间去补发。
    const uint64_t external_delta_ns = now_ns - motion_last_ramp_time_ns_;
    (void)encoder_delta_deg;
    // 实测转速现在由 refreshEncoderFeedback() -> updateSpeedEstimate() 统一更新，
    // 跟随分支不再单独维护一份 0.9/0.1 低通（三份实现已经合并成一份）。
    runExternalPulseFollow(external_delta_ns);
    motion_last_ramp_time_ns_ = now_ns;
    syncProtocolTelemetry();
}

void ClosedLoopController::updateSettleStage(uint64_t now_ns, float encoder_delta_deg)
{
    // 沉降阶段的速度同样由 updateSpeedEstimate() 统一维护，
    // 否则上位机的"即时转速"会停在进入沉降前的旧值上（旧实现为此单独补了一段低通）。
    (void)encoder_delta_deg;

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
        // 【只存快照，不 printf】这里是 20kHz 控制任务：settled 路径轴确实静止了，
        // 但 **timeout 路径轴可能还在大幅回弹**，~10ms 的 printf 停摆会被计进沉降
        // 观察窗、饿掉补偿拍。报告由 telemetry_reporter 的 kSlow 档在确认真停后打印。
        settle_report_.elapsed_ms = static_cast<float>(settle_elapsed_ns) / 1.0e6f;
        settle_report_.settled = settled;
        settle_report_.cmd_deg = motion_commanded_travel_deg_;
        settle_report_.enc_deg = static_cast<float>(motion_encoder_travel_raw_) * 360.0F / 65536.0F;
        settle_report_.follow_deg = motion_follow_error_deg_;
        settle_report_.emitted = motion_steps_emitted_;
        settle_report_.pulse_count = motion_pulse_count_;
        settle_report_.pending = true;
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
    // 使能来源跟随：外部 EN 模式下每拍看一次 PC13（只在变化时写 GPIO，不打印）。
    updateEnableFromExternalPin();

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

    // ② 编码器快照 + 速度估计 + 跟随误差 + 上一窗已发完步数记账
    const float encoder_delta_deg = refreshEncoderFeedback(now_ns);
    // ②b 堵转/不转保护：有脉冲在发、编码器却不动 —— 先停机失能，别让驱动一直灌电流
    updateStallGuard(now_ns);

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
    // 换向后 100ms 也不采样：那时命令方向刚翻转、机械还在减速，
    // follow_error 会拉到几十度（物理必然），计进去会把评分 RMS 抬到 58~104°。
    if (motion_turnaround_skip_ramps_ > 0U)
    {
        --motion_turnaround_skip_ramps_;
    }
    const bool tune_sampling_allowed = (motion_turnaround_skip_ramps_ == 0U);
    if (tune_sampling_allowed && motion_running_ && !closed_loop_angle_mode_enabled_ &&
        closed_loop_compensation_enabled_ && motion_ramp_stage_ != RAMP_STAGE_SETTLE)
    {
        const bool latter_half = motion_steps_emitted_ >= motion_pulse_count_ / 2U;
        // 【评分用残差】扣掉滞后角：稳态时 follow_error 本来就应该等于 -滞后角，
        // 不扣的话所有 PID 组合的评分都被固有滞后的 1.5° 抬高，好差参数差异被压缩。
        pid_auto_tuner_.addSample(
            -(motion_follow_error_deg_ - motion_lag_angle_deg_), latter_half);
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

    // 【预同步沉降窗口（0x179）】速度模式启动后的静置段：斜坡不推进、不发脉冲，
    // 编码器反馈/遥测照常刷新（0x120 能看到拉入摆动的衰减），堵转守卫保持解除
    // （本段命令速度是起跑速度但没有任何脉冲，不能用"命令 vs 编码器"判静止）。
    if (presync_remaining_ns_ > 0ULL)
    {
        const uint64_t spent_ns = now_ns - motion_last_ramp_time_ns_;
        presync_remaining_ns_ = (presync_remaining_ns_ > spent_ns)
            ? (presync_remaining_ns_ - spent_ns) : 0ULL;
        motion_last_ramp_time_ns_ = now_ns;
        stall_armed_ = false;
        syncProtocolTelemetry();
        return;
    }

    // 获取两次rampUpdate生成之间真实流逝的纳秒
    uint64_t delta_ramp_ns = now_ns - motion_last_ramp_time_ns_;

    // 【滞后角】每拍都算，供遥测（0x151）、网页「误差不计滞后角」和补偿段共用。
    // 必须放在补偿开关之外：用户不勾闭环补偿时，网页上的滞后角显示也应该是对的。
    // 指令速度取自上一拍（updateTrapezoid 稍后才推进本拍速度，差 50us 可忽略）。
    updateLagAngleFeedforward(encoder_speed_rpm_ / 60.0f,
                              current_step_speed_ / static_cast<float>(getStepsPerRound()));

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

    // ===================== 规划运动的闭环补偿 =====================
    // 结构（2026-11 重写，修掉“改一点点速度环就抖/一起步就满速”）：
    //
    //   v_profile ──┬──────────────► + ──► v_ref ──► + ──► 限幅/斜率 ──► compensated_rps_
    //               │                 ▲              ▲
    //               │        pos_corr │     vel_corr │
    //               │     (位置环，只 │   (速度环，只 │
    //               │      出 ±30%)  │    做阻尼)    │
    //   滞后角 ─────┴─► 误差剔除 ──► 位置 PID         │
    //                                    v_meas ──────┘
    //
    // 三条硬约束（这是本次修复的核心，不是调参能解决的）：
    //   1) 两个环的输出各自限幅到“当前规划速度的百分之几十”，并且 setOutputLimit()
    //      会同时收紧积分限幅 —— 任何一组错误增益都不可能再把速度指令顶穿；
    //   2) 速度环在“沿运动方向的有符号速度”域里工作。旧代码用“速度大小 base（恒正）”
    //      去减“带符号实测速度”，反向时误差凭空多出 2 倍速度，反向补偿一直是错的；
    //   3) 滞后角先从误差里剔除再进 PID。滞后角是步进电机的物理必然（转子必须落后
    //      一个力矩角才拖得动负载），不是丢步；把它当误差去追会持续多发脉冲，
    //      表现为“越走越超前 / 停不下来”。
    if (closed_loop_compensation_enabled_ && motion_running_ && !closed_loop_angle_mode_enabled_ &&
        calibration_stage_ == CALIBRATION_IDLE &&
        encoder_ != nullptr && delta_ramp_ns > 0U && motion_max_step_s_ > 0.0f)
    {
        const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
        const float direction = (motion_direction_ >= 0) ? 1.0f : -1.0f;
        const uint32_t micro_steps_per_round = getStepsPerRound();
        const float max_rps = motion_max_rpm_ / 60.0f;
        const float profile_rps = current_step_speed_ / static_cast<float>(micro_steps_per_round);
        // 把实测速度投影到“运动方向”上：正向就是它自己，反向取反。
        // 之后所有量（参考、误差、修正）都在同一个符号域里，不再需要乘 direction。
        const float measured_along_rps = (encoder_speed_rpm_ / 60.0f) * direction;

        // 【位置误差先过一阶低通（约 2ms）】跟随误差里混着"转子每走一个微步的振铃"
        // （实测恒速段 RMS 0.18°、峰峰 0.7°，而真实跟踪误差只有零点零几度）。
        // 不滤波时 kp=80 会把这段振铃放大成 ±4% 的脉冲频率调制 —— 用户听到/看到的
        // "闭环一开就抖"主要就是它。低通只对位置环生效，速度环另有死区+输出低通。
        constexpr float kErrAlpha = 0.025f;   // 50us / 2ms
        follow_error_lpf_deg_ += (motion_follow_error_deg_ - follow_error_lpf_deg_) * kErrAlpha;

        // 【加减速项的滞后角】加速时转子还要额外提供 J·a 的转矩，真实滞后角比
        // 恒速表值大，且大小与指令加速度成正比（ΔL ≈ c·a）。不建模的话，这 0.5~0.8°
        // 会被位置环当成丢步补掉，停下来就变成"多转一点"（实测 1/2/4 圈 +0.6~0.9°）。
        // c 来自 0x166（存 Flash），0 = 关闭。单位：毫度/(RPM/s)。
        float cmd_accel_rpm_s = 0.0f;
        if (motion_ramp_stage_ == RAMP_STAGE_ACCEL)
        {
            cmd_accel_rpm_s = motion_accel_rpm_s_;
        }
        else if (motion_ramp_stage_ == RAMP_STAGE_DECEL)
        {
            cmd_accel_rpm_s = -motion_accel_rpm_s_;
        }
        const float lag_accel_world_deg =
            -direction * (static_cast<float>(lag_accel_coef_mdeg_per_rpm_s_) / 1000.0f) * cmd_accel_rpm_s;

        // ---------- 1) 剔除物理滞后角 ----------
        // 符号：motion_lag_angle_deg_ 与 motion_follow_error_deg_ 同号（见 updateLagAngleFeedforward）。
        //
        // 【必须先投影到“沿运动方向”再进 PID】这是反向补偿的关键一行：
        //   世界坐标里 follow_error = 实际 - 命令，正向落后是负数、反向落后是正数。
        //   PID 的输出语义是“沿运动方向的附加速度”，所以误差也必须换成同一个符号域：
        //       反向落后 -> 世界坐标 +L -> 投影后 -L -> PID 正输出 -> 加速（正确）
        //   不投影的话，反向落后会被当成“超前”而刹车 —— 实测反向 1 圈只走了 250°，
        //   少 110°，评分 166°（这就是整定验证里“中速反向 252°”的来源）。
        const float effective_error_deg =
            (follow_error_lpf_deg_ - motion_lag_angle_deg_ - lag_accel_world_deg) * direction;
        // 残差死区：表值不可能完全准（拟合误差 + 负载差异），死区外的部分才进 PID。
        // 死区大小来自 0x167（存 Flash），不写死 —— 它必须和"滞后角表在当前负载下有多准"
        // 匹配：表是带载标定的、负载又变了，就给它大一点；重新标定后再收小。
        const float kResidualDeadbandDeg = static_cast<float>(lag_deadband_mdeg_) / 1000.0f;
        float pid_error_deg = effective_error_deg;

        // 【起步屏蔽】前 20ms 不动作：这段误差是"脉冲频率从 0 跳到 start_rpm"的
        // 惯性暂态，PID 去补它正好多走那一截（实测 1 圈短行程 +0.55~0.9°）。
        const bool startup_blank = (motion_start_blank_ramps_ > 0U);
        if (startup_blank)
        {
            --motion_start_blank_ramps_;
            pid_error_deg = 0.0f;
        }
        else if (pid_error_deg > kResidualDeadbandDeg)
        {
            pid_error_deg -= kResidualDeadbandDeg;
        }
        else if (pid_error_deg < -kResidualDeadbandDeg)
        {
            pid_error_deg += kResidualDeadbandDeg;
        }
        else
        {
            pid_error_deg = 0.0f;
        }

        // ---------- 2) 位置环：只输出“沿运动方向的附加速度” ----------
        // 上限跟着规划速度走：低速时给 15RPM 的余量，高速时最多 30%。
        const float pos_limit = fast_clamp(profile_rps * 0.30f, 0.25f, max_rps * 0.50f);
        position_pid_.setOutputLimit(pos_limit);
        float pos_corr = position_pid_.update(-pid_error_deg / 360.0f, dt);
        pos_corr = fast_clamp(pos_corr, -pos_limit, pos_limit);

        // ---------- 3) 速度环：只做阻尼 ----------
        // 参考速度 = 规划前馈 + 位置环修正（都在“沿运动方向”域里）。
        const float v_ref_along_rps = profile_rps + pos_corr;
        // 【为什么这里必须加死区 + 输出低通】恒速段实测速度估计仍有 ~8RPM 的噪声
        // （转子每走一个微步的机械振铃 + 传感器噪声，3ms 平滑压不掉）。速度环
        // kp=0.2 会把 ±8RPM 的噪声变成 ±1.6RPM 的脉冲频率调制 —— 这就是用户看到的
        // "闭环一开，轴就抖；开环很安静"。位置环不受影响（0.15° 死区远大于 0.03° 噪声），
        // 噪声全是从速度环这条路灌进来的。
        //   死区：当前规划速度的 2%（至少 0.02rps≈1.2RPM），以内不动作；
        //   输出低通：约 3ms，只留 10~30Hz 的阻尼，几百 Hz 的噪声直接滤掉。
        const float vel_deadband = fast_clamp(profile_rps * 0.02f, 0.02f, 0.20f);
        float vel_error = v_ref_along_rps - measured_along_rps;
        if (vel_error > vel_deadband)
        {
            vel_error -= vel_deadband;
        }
        else if (vel_error < -vel_deadband)
        {
            vel_error += vel_deadband;
        }
        else
        {
            vel_error = 0.0f;
        }
        // 起步屏蔽期间速度环也不动作：否则它会按"规划速度 - 实测速度"猛推一把。
        if (startup_blank)
        {
            vel_error = 0.0f;
        }
        const float vel_limit = fast_clamp(profile_rps * 0.20f, 0.15f, max_rps * 0.35f);
        velocity_pid_.setOutputLimit(vel_limit);
        const float measured_accel_rps2 = (measured_along_rps - last_measured_along_rps_) / dt;
        last_measured_along_rps_ = measured_along_rps;
        const float vel_corr_raw = velocity_pid_.update(vel_error, dt, 1.0f,
                                                        measured_accel_rps2, true);
        constexpr float kVelCorrAlpha = 0.0167f;   // 50us / 3ms
        velocity_correction_lpf_ += (vel_corr_raw - velocity_correction_lpf_) * kVelCorrAlpha;
        const float vel_corr = fast_clamp(velocity_correction_lpf_, -vel_limit, vel_limit);

        // ---------- 4) 总修正限幅 ----------
        // 即使两个环同时顶满，也不超过规划速度的 1.5 倍；而且不允许把速度拉成反向
        // （emitSteps 里用的是“基础速度 + 修正”的模，负过头会变成反向走）。
        const float corr_limit = fast_clamp(profile_rps * 0.50f, 0.35f, max_rps * 0.60f);
        float corr_rps = fast_clamp(pos_corr + vel_corr, -corr_limit, corr_limit);
        if (profile_rps + corr_rps < 0.0f)
        {
            corr_rps = -profile_rps;
        }

        // 斜率限制：补偿量本身也不能一拍跳变（否则脉冲频率会突跳，听感/电流都不对）。
        const float max_delta_rps = (angle_accel_rpm_s_ / 60.0f) * dt;
        compensated_rps_ = fast_clamp(corr_rps,
                                      compensated_rps_ - max_delta_rps,
                                      compensated_rps_ + max_delta_rps);

        // 【诊断快照 [VLOOP]】规划运动补偿路径（与角度模式同格式，方便对比）。
        {
            auto& d = vloop_;
            d.err_deg = effective_error_deg;
            d.ref_rps = v_ref_along_rps;
            d.meas_rps = measured_along_rps;
            d.vel_err_rps = v_ref_along_rps - measured_along_rps;
            d.pos_out_rps = pos_corr;
            d.vel_p = velocity_pid_.kp * d.vel_err_rps;
            d.vel_i = velocity_pid_.ki * velocity_pid_.integral;
            d.vel_d = velocity_pid_.kd * velocity_pid_.filtered_derivative;
            d.out_rps = compensated_rps_;
            d.prof_vel_rps = profile_rps;
            d.step_speed_s = current_step_speed_;
            d.stamp++;
        }
    }

    // 连续速度模式：TMR2 恒定周期 PWM 直出（绕过窗口出口），返回 true = 本拍已处理。
    if (motion_planner_.velocityPwmTick(delta_ramp_ns))
    {
        motion_last_ramp_time_ns_ = now_ns;
        syncProtocolTelemetry();
        return;
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

// ===========================================================================
// 停止运动（由 0x11F=0 / 失能 / 校准结束 / 故障触发）。
//
// 两个必须保留的重置：
//   ① closed_loop_angle_mode_enabled_ = false
//      否则 rampUpdate 仍会走 updateAngleMode，用 PID 继续把电往 target_angle_deg_ 上拉，
//      表现为“点了停止电机还在自己转”。
//   ② current_step_speed_ / compensated_rps_ / motion_ramp_stage_
//      这三个一旦残留，下次 startMotion 的起跑速度会从旧值接着跑，
//      或因为阶段停在 DONE/SETTLE 而启动后不动。
//
// 注意：这里不断电流（setEnable 不动），只停脉冲；需要断电请用 MOTOR_DISABLE。
// ===========================================================================
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
    // 【停止必须退出角度模式】否则 rampUpdate 仍会走 updateAngleMode，
    // 用 PID 继续把电往 target_angle_deg_ 上拉 —— 表现为“点了停止电机还在自己转”、
    // 而且之后再启动规划运动时两套逻辑抢 current_step_speed_，严重抖动。
    closed_loop_angle_mode_enabled_ = false;
    motion_running_ = false;
    motion_first_run_ = false;
    motion_paused_ = false;
    // 急停意图同样不许穿越一次停止（残留会把下一次运动的 DECEL 尾截掉，
    // 见 startMotion 里同样的清理）。
    quick_stop_requested_ = false;
    motion_quick_stop_ = false;
    motion_speed_rpm_ = 0.0f;
    encoder_speed_rpm_ = 0.0f;
    target_velocity_rps_ = 0.0f;
    // 速度估计的滑窗 + 滞后角平滑状态一并清掉：否则下一次启动的前 1ms 里，
    // 速度窗里还混着上一次运动的样本，速度环会先被"上一局的残留速度"骗一下。
    resetSpeedEstimate();
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
    // 连续速度模式的恒定 PWM 输出（若在跑）一并停掉并解除接管。
    motion_planner_.velocityPwmHalt();
    presync_remaining_ns_ = 0;
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
    // 角度模式用的是独立实例，停止时同样要清干净：否则上一次锁定留下的积分会
    // 带进下一次角度模式（这正是“换个目标角度第一下就猛冲”的成因）。
    // 统一实例之后，这里清的就是真正在用的那两个（角度模式不再有独立 PID）。
    position_pid_.resetIntegral();
    position_pid_.resetDeriv();
    velocity_pid_.resetIntegral();
    velocity_pid_.resetDeriv();
    // 轮廓作废：下次进角度模式从当前实际角度重新起步。
    // 不清的话，停止前那个“已经走到一半”的轮廓会在下次启动时直接把前馈叠上去，
    // 表现为“一上电/一进角度模式就先跑一下”。
    angle_profile_valid_ = false;
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
    // 【发布顺序 = 无锁并发契约】本函数跑在 USB 任务（优先级 2），控制任务是优先级 3：
    // TMR4 拍一到就能在**任意两行之间**抢占这里。规则：中间态一律保持 motion_running_=false
    //（控制任务对 idle 状态不碰任何运动字段），所有字段准备完毕后，最后一步才用
    // 单个对齐字写（M4 上原子）把状态"发布"成运行中。反过来停止路径先写 running=false。
    // 让跟随误差（编码器行程 - 命令行程）照常计算：0x121 和波形图都靠它
    external_pulse_follower_.setPrimePending(true);
    external_pulse_follower_.setEnabled(true);
    motion_running_ = true;   // <-- 发布写必须留在最后，不要动它的位置

    // 【EN 权威 = 外部 EN 引脚（PC13），这里永远不碰 EN】（2026-11-10 用户明确）
    // 外部脉冲跟随模式的语义：开机/切换只是"进入模式"（脉冲通道交还给跟随器），
    // 通不通电流由 PLC 那根 EN 线决定：低=使能、高=失能。最多 200µs（消抖 4 拍）
    // 内由 updateEnableFromExternalPin 按引脚电平归位。
    // 内部使能按钮（0x124）在跟随模式下只能"撤销失能锁"，不能隔着 PLC 强行使能。
    if (driver_ != nullptr &&
        !fault_monitor_.encoderFaultActive() && !fault_monitor_.magneticFaultActive())
    {
        driver_->setDirection(true);
    }
    printf("[EXT_FOLLOW] armed: 1 pulse = 1 microstep, %lu steps/rev, compensation=%u (EN by pin)\r\n",
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
    // 校准进行中不能读表（否则会和写入抢 Flash）：问校准器而不是直接读裸状态。
    if (encoder_ == nullptr || packed_values == nullptr || !calibrator_.isIdle())
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

    // 【性能】原来是线性扫 65536 个原始值、每个都查一次校准表插值，
    // 实测阻塞约 13ms。这个函数由 0x13B（校准后值 -> 原始值反查）触发，
    // 网页渲染校准表时如果对每项都反查一次，就是 512 x 13ms = 6.6 秒的连续阻塞，
    // 期间 TMR4 中断被推迟、DDA 累加器堆成山，恢复后一波脉冲打出去，
    // 实测表现为“看过校准表再跑运动，follow_error 变成 9 度”。
    // 校准表是单调的，改用二分定位（16 次）+ 邻域细扫（±32）校验，总耗时降到约 50us。
    uint32_t low = 0U;
    uint32_t high = 65535U;
    while (low < high)
    {
        const uint32_t mid = (low + high) / 2U;
        const uint16_t corrected = encoder_->getCorrectedRaw(static_cast<uint16_t>(mid));
        if (corrected < corrected_value)
        {
            low = mid + 1U;
        }
        else
        {
            high = mid;
        }
    }

    // 二分只保证“第一个 >= 目标”的位置，邻域再扫一遍取最接近的（容忍表有局部噪声）
    uint16_t best_raw = static_cast<uint16_t>(low);
    uint32_t best_delta = 0xFFFFFFFFU;
    const uint32_t scan_begin = (low > 32U) ? (low - 32U) : 0U;
    const uint32_t scan_end = (low + 32U < 65535U) ? (low + 32U) : 65535U;
    for (uint32_t raw_value = scan_begin; raw_value <= scan_end; ++raw_value)
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
    if (encoder_ == nullptr || checksum == nullptr || !calibrator_.isIdle())
    {
        return false;
    }
    *checksum = encoder_->calibrationChecksum();
    return true;
}

// ===========================================================================
// 编码器校准已迁到 EncoderCalibrator（见 encoder_calibrator.cpp），这里只留两个
// “确实有外部调用者”的转发：
//   calibrateEncoder()          ← parameter_router 处理 0x144 时调
//   updateEncoderCalibration()  ← rampUpdate 每拍调
// 原来的 finishEncoderCalibration() / startEncoderCalibrationMove() 已经搬进校准器
// 并作为它的私有方法，controller 侧不再有任何调用者，所以这两个包装直接删除
// （而不是把校准器的私有方法提成 public 来迁就没人调的包装）。
//
// 保留薄包装而不改调用点，是因为调用方分处两个模块、且都在关键路径上；
// 包装层让它们一行都不用动，拆分的回归面只落在“实现搬对没有”这一件事上。
//
// 【阶段状态只有一份】calibration_stage_ 仍留在本类：
//   motion_planner.cpp 的 updateTrapezoid 会在一小段校准位移走完时把它从 MOVING
//   改成 SAMPLING（校准与规划之间唯一的握手），读表接口也用它做门禁。
// ===========================================================================
void ClosedLoopController::calibrateEncoder()
{
    // 【校准必须持有内部运动通道】校准要做"回零 + 逐段扫描"，全靠
    // setTargetAngleDeg / startMotion —— 而这两者在外部跟随模式下都被互斥门拒。
    // 上电默认外部跟随后，不接管就会"只能归零无法运动"（2026-11-10 实机反馈）。
    // 接管 = 和网页手动切内部完全相同的一次原子切换：从此刻起外部 PUL 被忽略，
    // 直到用户显式切回 —— 互斥不变量没有松动，只是校准入口替用户做了切换并留日志。
    ensureInternalSourceForCalibration("encoder calib (0x136)");
    calibrator_.start();
}

void ClosedLoopController::updateEncoderCalibration(const uint16_t raw)
{
    calibrator_.update(raw);
}
