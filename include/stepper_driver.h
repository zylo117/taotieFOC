#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "at32f403a_407.h"

#define STEP_OUTPUT_PORT         GPIOA
#define STEP_OUT_PIN             GPIO_PINS_15
#define DIR_OUTPUT_PORT          GPIOA
#define DIR_OUT_PIN              GPIO_PINS_14
#define EN_OUTPUT_PORT           GPIOA
#define EN_OUT_PIN               GPIO_PINS_13

#define TMC2209_UART_GPIO        GPIOA
#define TMC2209_UART_PIN         GPIO_PINS_4

namespace stepper_common
{
// 向指定 GPIO 端口写入数字状态，封装了 AT32 的寄存器写法。
void stepper_write_gpio(gpio_type *port, uint16_t pin, bool state);

// 初始化步进输出相关 GPIO（STEP / DIR / EN），用于驱动器逻辑电平控制。
void stepper_init_step_gpio(void);

// 初始化 UART 相关 GPIO，用于 TMC 单线串口通信。
void stepper_init_uart_gpio(gpio_type *port, uint16_t pin);
}

// 统一的驱动器配置参数，负责描述底层步进器的共性参数。
// 这里不依赖具体芯片型号，而是抽象出所有 TMC 系列驱动会用到的一组参数。
struct StepperDriverConfig
{
  uint16_t microsteps;                 // 当前微步数，例如 32, 64, 128
  uint16_t interpolation_microsteps;  // 插值/目标微步，通常由 256 或更高分辨率表示
  float run_current_a;                // 运行电流，单位 A
  float hold_current_a;               // 保持电流，单位 A
  bool enable_hold_current;           // 是否启用保持电流
  bool silent_mode;                   // 静音模式
  float sense_resistor_ohm;           // 采样电阻值，单位 ohm
  float vref_mv;                      // VREF 参考电压（毫伏）
  uint32_t uart_baudrate;             // UART 波特率
  bool use_single_wire_uart;          // 是否使用单线 UART
  bool uart_inverted;                 // UART 极性是否反向
  gpio_type *uart_port;               // UART GPIO 端口
  uint16_t uart_pin;                  // UART GPIO 引脚
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
  virtual bool configure(const StepperDriverConfig &config) = 0;

  // 返回当前配置快照。
  virtual const StepperDriverConfig &config() const = 0;

  // 通用寄存器读写接口，供协议层对接 TMC 寄存器访问。
  virtual bool writeRegister(uint8_t reg, uint32_t value) = 0;
  virtual bool readRegister(uint8_t reg, uint32_t *value) = 0;

  // 使能、方向、步进状态等基本控制输入。
  virtual void setEnable(bool enable) = 0;
  virtual void setDirection(bool direction) = 0;
  virtual void setStepState(bool state) = 0;
  virtual void sendStepPulse(uint32_t width_us) = 0;

  // 驱动器配置控制。
  virtual bool setMicrosteps(uint16_t microsteps) = 0;
  virtual bool setRunCurrent(float amps) = 0;
  virtual bool setHoldCurrent(float amps, bool enabled) = 0;
  virtual bool setSilentMode(bool enable) = 0;
  virtual bool setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps) = 0;
  virtual bool setSenseResistor(float ohm) = 0;
  virtual bool setSingleWireUart(gpio_type *port, uint16_t pin, bool enable) = 0;
};

#endif
