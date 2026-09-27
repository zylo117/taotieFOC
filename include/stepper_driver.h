#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <cstdbool>
#include <cstdint>
#include <cmath>

#include "at32f403a_407.h"

#define STEP_OUTPUT_PORT         GPIOB
#define STEP_OUT_PIN             GPIO_PINS_10
#define DIR_OUTPUT_PORT          GPIOB
#define DIR_OUT_PIN              GPIO_PINS_11
#define EN_OUTPUT_PORT           GPIOA
#define EN_OUT_PIN               GPIO_PINS_3
#define EN_ACTIVE_LEVEL          false
#define DIR_FORWARD_LEVEL        true

#define TMC2209_UART_GPIO        GPIOA
#define TMC2209_UART_PIN         GPIO_PINS_4

namespace stepper_common
{
    struct StepDirCaptureEvent
    {
        uint32_t tick;
        bool step;
        bool dir;
    };

    constexpr uint32_t kStepDirCaptureRingDepth = 256U;

    // 向指定 GPIO 端口写入数字状态，封装了 AT32 的寄存器写法。
    void stepper_write_gpio(gpio_type* port, uint16_t pin, bool state);
    void stepper_write_gpio_high(gpio_type* port, uint16_t pin);
    void stepper_write_gpio_low(gpio_type* port, uint16_t pin);
    void stepper_write_gpio(gpio_type* port, uint16_t pin, bool state);
    void stepper_delay_us(uint32_t microseconds);
    void stepper_delay_ns(uint64_t nanoseconds);
    void stepper_set_direction(bool direction);
    void stepper_set_step_state(bool state);
    void stepper_send_step_pulse(uint64_t width_ns);
    void stepper_init_motion_timer(uint32_t k_psc, uint32_t k_step_pulse_ticks, bool direction);
    void stepper_start_motion_timer(uint32_t step_hz, uint32_t pulse_width_us);
    void stepper_update_motion_timer(uint32_t step_hz, uint32_t pulse_width_us);
    void stepper_stop_motion_timer(void);

    // 初始化步进输出相关 GPIO（STEP / DIR / EN），并配置 TMR2 的输入捕获/输出比较。
    void stepper_init_step_gpio(void);
    void stepper_init_tmr2_capture_and_oc(void);
    void stepper_init_capture_ring_buffer(void);
    bool stepper_push_capture_event(uint32_t tick, bool step_state, bool dir_state);
    bool stepper_pop_capture_event(StepDirCaptureEvent* output);
    void stepper_reset_capture_ring_buffer(void);
    void stepper_queue_simulated_pulse_sequence(uint32_t pulse_count, bool direction, uint32_t start_hz, uint32_t max_hz, uint32_t accel_hz_s);

    // 初始化 UART 相关 GPIO，用于 TMC 单线串口通信。
    void stepper_init_uart_gpio(gpio_type* port, uint16_t pin);

    // at32的定时器的外部时钟频率fTMRxCLK（我简称fT）是cpu频率除以2（APB总线），250Mhz就是125Mhz
    // PSC决定了定时器计数频率，越大计数越快，fC = fT/(PSC+1)
    // ARR指计数到多少就重置，指数到多少归零（周期），事件触发间隔T = (PSC+1)*(ARR+1) / fT
    // CCR指数到多少触发通道动作（翻转 / 高低电平）
    // 这里一定要搞明白，在SWITCH模式下，翻转状态是会继承的
    // 也就是你下一次计数重置并不会重置翻转状态，而是继承
    // 也就是SWITCH模式下CCR设置多少，占空比都是50%
    // 而在PWM模式下，是会重置的，不会继承！！！

    // 原始参数
    constexpr uint32_t F_APB = 125 * 1e6; // cpu主频的一半，125Mhz
    constexpr uint32_t target_tick_time = 8; // ns
    constexpr uint32_t target_pulse_width = 200; // ns, 需要保证足够宽的高电平，避免丢步
    constexpr uint32_t dir_to_step_setup_time = 20; // ns, DIR to STEP 最小提前时间
    constexpr uint32_t dir_to_step_hold_time = 20; // ns, DIR to STEP 最小保持时间

    // 推导参数
    // PSC=0 → tick = (0+1)/125M = 8ns
    constexpr uint32_t k_psc = ceil(target_tick_time * (F_APB / 1.0e9)) - 1;  // 8ns @ 125Mhz

    // 脉宽tick数/ARR都不可以大于计数器最大范围，比如16位就是2^16，32位就2^32
    // ARR需要大于脉宽tick数
    // 脉宽tick数到达ARR位置就会重置电平

    // CCR
    // 脉宽或负脉宽（取决于你，先触发就脉宽，先等待后触发就是负脉宽）
    // 脉宽tick 25, 200ns @ 125Mhz
    // 负脉宽tick数，每个周期等多久就开始触发上升沿
    constexpr uint32_t k_step_pulse_ticks = ceil(target_pulse_width / (float) target_tick_time);
    constexpr uint32_t k_dir_guard_ticks = ceil((dir_to_step_setup_time > dir_to_step_hold_time ? dir_to_step_setup_time : dir_to_step_hold_time) / (float) target_tick_time);
    constexpr uint32_t guard_tick = (k_dir_guard_ticks > 1U) ? k_dir_guard_ticks : 1U;

}

// 统一的驱动器配置参数，负责描述底层步进器的共性参数。
// 这里不依赖具体芯片型号，而是抽象出所有 TMC 系列驱动会用到的一组参数。
struct StepperDriverConfig
{
    uint16_t microsteps; // 当前微步数，例如 32, 64, 128
    uint16_t interpolation_microsteps; // 插值/目标微步，通常由 256 或更高分辨率表示
    float run_current_a; // 运行电流，单位 A
    float hold_current_a; // 保持电流，单位 A
    bool enable_hold_current; // 是否启用保持电流
    bool silent_mode; // 静音模式
    float sense_resistor_ohm; // 采样电阻值，单位 ohm
    float vref_mv; // VREF 参考电压（毫伏）
    uint32_t uart_baudrate; // UART 波特率
    bool use_single_wire_uart; // 是否使用单线 UART
    bool uart_inverted; // UART 极性是否反向
    gpio_type* uart_port; // UART GPIO 端口
    uint16_t uart_pin; // UART GPIO 引脚
};

// StepperDriver 是步进器后端抽象层，对所有 TMC 系列驱动提供统一接口。
// 这里不关心具体 TMC2208 / 2209 / 5160 的寄存器细节，只关心“驱动器能做什么”。
class StepperDriver
{
public:
    virtual ~StepperDriver() = default;

    // 初始化驱动器底层资源，例如 GPIO、UART、状态寄存器等。
    virtual bool init() = 0;

    // 以统一配置对象设置驱动器参数。
    virtual bool configure(const StepperDriverConfig& config) = 0;

    // 返回当前配置快照。
    virtual const StepperDriverConfig& config() const = 0;

    // 通用寄存器读写接口，供协议层对接 TMC 寄存器访问。
    virtual bool writeRegister(uint8_t reg, uint32_t value) = 0;
    virtual bool readRegister(uint8_t reg, uint32_t* value) = 0;

    // 使能、方向、步进状态等基本控制输入。
    virtual void setEnable(bool enable) = 0;
    virtual void setDirection(bool direction) = 0;
    virtual void setStepState(bool state) = 0;
    virtual void sendStepPulse(uint64_t width_ns) = 0;

    // 驱动器配置控制。
    virtual bool setMicrosteps(uint16_t microsteps) = 0;
    virtual bool setRunCurrent(float amps) = 0;
    virtual bool setHoldCurrent(float amps, bool enabled) = 0;
    virtual bool setSilentMode(bool enable) = 0;
    virtual bool setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps) = 0;
    virtual bool setSenseResistor(float ohm) = 0;
    virtual bool setSingleWireUart(gpio_type* port, uint16_t pin, bool enable) = 0;
};

#endif
