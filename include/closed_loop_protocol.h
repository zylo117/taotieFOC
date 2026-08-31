#ifndef CLOSED_LOOP_PROTOCOL_H
#define CLOSED_LOOP_PROTOCOL_H

#include <stdint.h>

#include "stepper_driver.h"

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

enum Tmc2209CustomExtensionRegister
{
  TMC2209_EXT_PARAM_MICROSTEPS = 0x100U,
  TMC2209_EXT_PARAM_INTERPOLATION = 0x101U,
  TMC2209_EXT_PARAM_RUN_CURRENT_A = 0x102U,
  TMC2209_EXT_PARAM_HOLD_CURRENT_A = 0x103U,
  TMC2209_EXT_PARAM_HOLD_ENABLED = 0x104U,
  TMC2209_EXT_PARAM_SILENT_MODE = 0x105U,
  TMC2209_EXT_PARAM_SENSE_RESISTOR = 0x106U,
  TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE = 0x107U
};

class ClosedLoopDriverProtocol
{
public:
  virtual ~ClosedLoopDriverProtocol() = default;
  virtual bool init() = 0;
  virtual bool configure(const ClosedLoopDriverProtocolConfig &config) = 0;
  virtual const ClosedLoopDriverProtocolConfig &config() const = 0;
  virtual bool writeRegister(uint8_t reg, uint32_t value) = 0;
  virtual bool readRegister(uint8_t reg, uint32_t *value) = 0;
  virtual bool setCustomParameter(uint16_t id, uint32_t value) = 0;
  virtual bool getCustomParameter(uint16_t id, uint32_t *value) const = 0;
  virtual void attachDriver(StepperDriver *driver) = 0;
};

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

  void syncToDriver();

private:
  static bool isOfficialTmcRegister(uint8_t reg);
  static bool isCustomExtensionRegister(uint16_t id);
  static uint32_t buildRequestWord(uint8_t reg, uint32_t value);
  static uint32_t decodeResponseWord(uint8_t reg, uint32_t value);
  bool applyOfficialRegister(uint8_t reg, uint32_t value);
  bool readOfficialRegister(uint8_t reg, uint32_t *value) const;

  ClosedLoopDriverProtocolConfig config_;
  uint32_t custom_parameters_[8];
  StepperDriver *driver_;
};

#endif
