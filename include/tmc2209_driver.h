#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include "stepper_driver.h"

/*
 * ===================== 【TMC UART 通讯硬开关，默认永久关闭】=====================
 * 禁止开启。tmc_uart 分支（4a8e128）实现过一版单线 UART，实测烧掉了两个 TMC2209。
 * 复查后找到的致命点，开启前必须先修完：
 *   1. setUartTransmitMode() 用的是 GPIO_OUTPUT_PUSH_PULL。PDN_UART 是双向单线，
 *      推挽输出会和驱动器回包直接对拉，灌电流烧 PDN_UART。必须是开漏。
 *   2. releaseUartLine() 用 AT32 内部 40k 弱上拉，115200 下上升沿 3~6us
 *      （bit 只有 8.68us），采样点漂移 → 无限重试 → 总线冲突次数暴增。
 *   3. writeRegister 每次都要 IFCNT 前后读 + 回读验证，一轮 20~40ms，
 *      且全部包在 __disable_irq() 里。一次 applyCurrentConfig 累计关中断 100ms 级，
 *      控制环停摆、电机突然断脉冲、恢复后通知堆积导致脉冲暴冲。
 *   4. attachDriver() → syncToDriver() 在 driver_->init()（使能 DWT）之前就跑，
 *      waitUntilCycle() 依赖的 CYCCNT 恒为 0，直接死循环。
 *   5. 运行中写 CHOPCONF(MRES/intpol) 或切 spreadCycle↔stealthChop 会造电流冲击，
 *      TMC2209 要求这类参数在电机静止、输出关闭时修改。
 *
 * 现状：微步由 MS1/MS2 跳线决定、电流由 VREF 分压决定，固件不碰 TMC 寄存器。
 * 网页上的微步/电流/静音只是本地记账（用于角度换算和回显），不会下发到芯片。
 * =====================【TMC UART 通讯硬开关，默认永久关闭】=====================
 */
#ifndef TMC_UART_DRIVER_ENABLE
#define TMC_UART_DRIVER_ENABLE 0
#endif

#if TMC_UART_DRIVER_ENABLE
#error "TMC UART driver comms are forbidden: the tmc_uart implementation burned two TMC2209 drivers. Fix the five defects listed in tmc2209_driver.h before enabling."
#endif

class Tmc2209Driver : public StepperDriver
{
public:
    Tmc2209Driver();
    bool init() override;
    bool configure(const StepperDriverConfig& config) override;
    const StepperDriverConfig& config() const override;
    bool writeRegister(uint8_t reg, uint32_t value) override;
    bool readRegister(uint8_t reg, uint32_t* value) override;
    void setEnable(bool enable) override;
    void setDirection(bool direction) override;
    void setStepState(bool state) override;
    void sendStepPulse(uint64_t width_ns) override;
    bool setMicrosteps(uint16_t microsteps) override;
    bool setRunCurrent(float amps) override;
    bool setHoldCurrent(float amps, bool enabled) override;
    bool setSilentMode(bool enable) override;
    bool setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps) override;
    bool setSenseResistor(float ohm) override;
    bool setSingleWireUart(gpio_type* port, uint16_t pin, bool enable) override;

private:
    bool enabled_;
    bool direction_;
    bool single_wire_uart_;
    StepperDriverConfig config_;
    // UART 引脚只用于把 PDN_UART 保持在高电平（总线空闲），不产生任何波形。
    // 引脚号保留是为了 PCB 接线文档化，不代表这里会去驱动它。
    gpio_type* uart_port_;
    uint16_t uart_pin_;
};

#endif
