#ifndef CLOSED_LOOP_PROTOCOL_H
#define CLOSED_LOOP_PROTOCOL_H

#include <stdint.h>

#include "stepper_driver.h"

// 这是闭环控制器对外的“协议桥接配置”，用于统一描述
// TMC 风格参数访问所需的最小工作状态。
// 通过这个结构体，控制器和底层驱动之间不直接绑定具体芯片寄存器，
// 而是通过统一协议语义进行参数映射与转换。
struct ClosedLoopDriverProtocolConfig
{
  uint16_t microsteps;
  uint16_t interpolation_microsteps;
  float run_current_a;
  float hold_current_a;
  bool enable_hold_current;
  bool silent_mode;
  float sense_resistor_ohm;
  bool use_single_wire_uart;
  gpio_type *uart_port;
  uint16_t uart_pin;
  uint32_t uart_baudrate;
};

// TMC2209 的官方寄存器表。
// 这里保留的是与 Klipper / TMC 实际寄存器地址相一致的核心位置，
// 用于在协议层把外部访问请求映射到闭环控制器的参数和底层驱动配置。
enum Tmc2209OfficialRegister
{
  TMC2209_REG_GCONF = 0x00U,
  TMC2209_REG_GSTAT = 0x01U,
  TMC2209_REG_IOIN = 0x06U,
  TMC2209_REG_DRV_CONF = 0x0AU,
  TMC2209_REG_IHOLD_IRUN = 0x10U,
  TMC2209_REG_TPOWERDOWN = 0x11U,
  TMC2209_REG_TSTEP = 0x12U,
  TMC2209_REG_TPWMTHRS = 0x13U,
  TMC2209_REG_TCOOLTHRS = 0x14U,
  TMC2209_REG_THIGH = 0x15U,
  TMC2209_REG_MSCNT = 0x6AU,
  TMC2209_REG_MSCURACT = 0x6BU,
  TMC2209_REG_CHOPCONF = 0x6CU,
  TMC2209_REG_COOLCONF = 0x6DU,
  TMC2209_REG_DCCTRL = 0x6EU,
  TMC2209_REG_DRV_STATUS = 0x6FU,
  TMC2209_REG_PWMCONF = 0x70U,
  TMC2209_REG_PWMSCALE = 0x71U,
  TMC2209_REG_ENCM_CTRL = 0x72U
};

// 自定义扩展寄存器区：0x100 起，专门承载闭环控制器特有参数。
// 这样不会污染 TMC 官方寄存器空间，且后续扩展更安全。
enum Tmc2209CustomExtensionRegister
{
  TMC2209_EXT_PARAM_MICROSTEPS = 0x100U,
  TMC2209_EXT_PARAM_INTERPOLATION = 0x101U,
  TMC2209_EXT_PARAM_RUN_CURRENT_A = 0x102U,
  TMC2209_EXT_PARAM_HOLD_CURRENT_A = 0x103U,
  TMC2209_EXT_PARAM_HOLD_ENABLED = 0x104U,
  TMC2209_EXT_PARAM_SILENT_MODE = 0x105U,
  TMC2209_EXT_PARAM_SENSE_RESISTOR = 0x106U,
  TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE = 0x107U,
  TMC2209_EXT_PARAM_FAULT_STATUS = 0x108U,
  TMC2209_EXT_PARAM_ENCODER_FAULT = 0x109U,
  TMC2209_EXT_PARAM_MAGNETIC_FAULT = 0x10AU,
  TMC2209_EXT_PARAM_OUTPUT_STOP = 0x10BU,
  TMC2209_EXT_PARAM_AB_CURRENT_A = 0x10CU,
  TMC2209_EXT_PARAM_AB_CURRENT_B = 0x10DU,
  TMC2209_EXT_PARAM_POS_LOOP_HZ = 0x10EU,
  TMC2209_EXT_PARAM_VEL_LOOP_HZ = 0x10FU,
  TMC2209_EXT_PARAM_CUR_LOOP_HZ = 0x110U,
  TMC2209_EXT_PARAM_LAST_FAULT = 0x111U
};

// ClosedLoopDriverProtocol 是闭环控制器的协议抽象接口。
// 它负责把上层“读取参数 / 写入参数”的请求，翻译成真正的底层驱动命令。
// 这使得控制器只关心闭环逻辑，而不直接依赖具体驱动芯片实现。
class ClosedLoopDriverProtocol
{
public:
  virtual ~ClosedLoopDriverProtocol() = default;

  // 初始化协议栈，并绑定底层驱动器。
  virtual bool init() = 0;

  // 配置协议所需的运行参数。
  virtual bool configure(const ClosedLoopDriverProtocolConfig &config) = 0;

  // 返回当前协议配置。
  virtual const ClosedLoopDriverProtocolConfig &config() const = 0;

  // 写入一个协议寄存器，例如 TMC 官方寄存器或扩展参数。
  virtual bool writeRegister(uint8_t reg, uint32_t value) = 0;

  // 读取一个协议寄存器的当前值。
  virtual bool readRegister(uint8_t reg, uint32_t *value) = 0;

  // 设置闭环扩展参数（例如微步数、当前、静音模式等）。
  virtual bool setCustomParameter(uint16_t id, uint32_t value) = 0;

  // 获取闭环扩展参数。
  virtual bool getCustomParameter(uint16_t id, uint32_t *value) const = 0;

  // 把实际驱动器对象挂接到协议适配器中。
  virtual void attachDriver(StepperDriver *driver) = 0;
};

// Tmc2209ProtocolAdapter 是协议适配器的具体实现。
// 它模拟了“外部主机按 TMC2209 协议访问驱动器”的行为，
// 但实际只是把这些访问重定向到闭环控制逻辑和统一驱动后端。
class Tmc2209ProtocolAdapter : public ClosedLoopDriverProtocol
{
public:
  Tmc2209ProtocolAdapter();
  bool init() override;
  bool configure(const ClosedLoopDriverProtocolConfig &config) override;
  const ClosedLoopDriverProtocolConfig &config() const override;
  bool writeRegister(uint8_t reg, uint32_t value) override;
  bool readRegister(uint8_t reg, uint32_t *value) override;
  bool setCustomParameter(uint16_t id, uint32_t value) override;
  bool getCustomParameter(uint16_t id, uint32_t *value) const override;
  void attachDriver(StepperDriver *driver) override;

  // 将当前协议配置同步到底层驱动器。
  void syncToDriver();

private:
  static bool isOfficialTmcRegister(uint8_t reg);
  static bool isCustomExtensionRegister(uint16_t id);
  static uint32_t buildRequestWord(uint8_t reg, uint32_t value);
  static uint32_t decodeResponseWord(uint8_t reg, uint32_t value);
  bool applyOfficialRegister(uint8_t reg, uint32_t value);
  bool readOfficialRegister(uint8_t reg, uint32_t *value) const;

  ClosedLoopDriverProtocolConfig config_;
  uint32_t custom_parameters_[32];
  StepperDriver *driver_;
};

#endif
