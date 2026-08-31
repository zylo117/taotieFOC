#include "closed_loop_protocol.h"

namespace
{
uint16_t clamp_microsteps(uint16_t value)
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

float clamp_current_a(float value)
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
}

Tmc2209ProtocolAdapter::Tmc2209ProtocolAdapter()
  : config_{32U, 256U, 0.8f, 0.2f, true, true, 0.110f, true, TMC2209_UART_GPIO, TMC2209_UART_PIN, 115200U},
    custom_parameters_{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}, driver_(nullptr)
{
}

bool Tmc2209ProtocolAdapter::isOfficialTmcRegister(uint8_t reg)
{
  switch (reg)
  {
    case TMC2209_REG_GCONF:
    case TMC2209_REG_GSTAT:
    case TMC2209_REG_IOIN:
    case TMC2209_REG_DRV_CONF:
    case TMC2209_REG_IHOLD_IRUN:
    case TMC2209_REG_TPOWERDOWN:
    case TMC2209_REG_TSTEP:
    case TMC2209_REG_TPWMTHRS:
    case TMC2209_REG_TCOOLTHRS:
    case TMC2209_REG_THIGH:
    case TMC2209_REG_CHOPCONF:
    case TMC2209_REG_COOLCONF:
    case TMC2209_REG_DCCTRL:
    case TMC2209_REG_DRV_STATUS:
    case TMC2209_REG_PWMCONF:
    case TMC2209_REG_PWMSCALE:
    case TMC2209_REG_ENCM_CTRL:
    case TMC2209_REG_MSCNT:
    case TMC2209_REG_MSCURACT:
      return true;
    default:
      return false;
  }
}

bool Tmc2209ProtocolAdapter::isCustomExtensionRegister(uint16_t id)
{
  switch (id)
  {
    case TMC2209_EXT_PARAM_MICROSTEPS:
    case TMC2209_EXT_PARAM_INTERPOLATION:
    case TMC2209_EXT_PARAM_RUN_CURRENT_A:
    case TMC2209_EXT_PARAM_HOLD_CURRENT_A:
    case TMC2209_EXT_PARAM_HOLD_ENABLED:
    case TMC2209_EXT_PARAM_SILENT_MODE:
    case TMC2209_EXT_PARAM_SENSE_RESISTOR:
    case TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE:
      return true;
    default:
      return false;
  }
}

uint32_t Tmc2209ProtocolAdapter::buildRequestWord(uint8_t reg, uint32_t value)
{
  // Klipper/TMC UART-style requests are conceptually addressed by register number,
  // with the value packed into the low 16 bits of the word and the register in the
  // upper byte. This keeps the host-facing API consistent with how the MCU requests
  // parameter writes and reads while leaving the real driver backend unaware of the
  // protocol format.
  return (static_cast<uint32_t>(reg) << 16U) | (value & 0xFFFFUL);
}

uint32_t Tmc2209ProtocolAdapter::decodeResponseWord(uint8_t reg, uint32_t value)
{
  (void)reg;
  return value & 0xFFFFUL;
}

bool Tmc2209ProtocolAdapter::applyOfficialRegister(uint8_t reg, uint32_t value)
{
  if (driver_ == nullptr)
  {
    return false;
  }

  switch (reg)
  {
    case TMC2209_REG_IHOLD_IRUN:
      config_.run_current_a = clamp_current_a(static_cast<float>((value & 0x1FU) * 10U) / 1000.0f);
      config_.hold_current_a = clamp_current_a(static_cast<float>(((value >> 8U) & 0x1FU) * 10U) / 1000.0f);
      config_.enable_hold_current = (value & 0x80U) != 0U;
      driver_->setRunCurrent(config_.run_current_a);
      driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
      return true;

    case TMC2209_REG_CHOPCONF:
      config_.microsteps = clamp_microsteps(static_cast<uint16_t>(value & 0xFFU));
      driver_->setMicrosteps(config_.microsteps);
      return true;

    case TMC2209_REG_DRV_CONF:
      config_.silent_mode = (value & 0x01U) != 0U;
      driver_->setSilentMode(config_.silent_mode);
      return true;

    case TMC2209_REG_GCONF:
      config_.use_single_wire_uart = (value & 0x04U) != 0U;
      config_.uart_baudrate = 115200U;
      driver_->setSingleWireUart(config_.uart_port, config_.uart_pin, config_.use_single_wire_uart);
      return true;

    case TMC2209_REG_GSTAT:
      return true;

    default:
      return true;
  }
}

bool Tmc2209ProtocolAdapter::readOfficialRegister(uint8_t reg, uint32_t *value) const
{
  if (value == nullptr)
  {
    return false;
  }

  uint32_t raw_value = 0U;

  switch (reg)
  {
    case TMC2209_REG_IHOLD_IRUN:
      raw_value = static_cast<uint32_t>(config_.run_current_a * 1000.0f) & 0x1FU;
      raw_value |= (static_cast<uint32_t>(config_.hold_current_a * 1000.0f) & 0x1FU) << 8U;
      raw_value |= config_.enable_hold_current ? 0x80U : 0U;
      break;

    case TMC2209_REG_CHOPCONF:
      raw_value = static_cast<uint32_t>(config_.microsteps);
      break;

    case TMC2209_REG_DRV_CONF:
      raw_value = config_.silent_mode ? 1U : 0U;
      break;

    case TMC2209_REG_GCONF:
      raw_value = config_.use_single_wire_uart ? 0x04U : 0U;
      break;

    default:
      raw_value = 0U;
      break;
  }

  *value = decodeResponseWord(reg, raw_value);
  return true;
}

bool Tmc2209ProtocolAdapter::init()
{
  if (driver_ != nullptr)
  {
    driver_->init();
    driver_->setSingleWireUart(config_.uart_port, config_.uart_pin, config_.use_single_wire_uart);
    driver_->setSenseResistor(config_.sense_resistor_ohm);
    driver_->setMicrosteps(config_.microsteps);
    driver_->setInterpolation(config_.microsteps, config_.interpolation_microsteps);
    driver_->setRunCurrent(config_.run_current_a);
    driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
    driver_->setSilentMode(config_.silent_mode);
  }
  return true;
}

bool Tmc2209ProtocolAdapter::configure(const ClosedLoopDriverProtocolConfig &config)
{
  config_ = config;
  if (config_.sense_resistor_ohm <= 0.0f)
  {
    config_.sense_resistor_ohm = 0.110f;
  }
  if (config_.uart_baudrate == 0U)
  {
    config_.uart_baudrate = 115200U;
  }
  if (config_.microsteps == 0U)
  {
    config_.microsteps = 32U;
  }
  if (config_.interpolation_microsteps == 0U)
  {
    config_.interpolation_microsteps = 256U;
  }
  if (config_.uart_port == nullptr)
  {
    config_.uart_port = TMC2209_UART_GPIO;
  }
  if (config_.uart_pin == 0U)
  {
    config_.uart_pin = TMC2209_UART_PIN;
  }
  if (driver_ != nullptr)
  {
    syncToDriver();
  }
  return true;
}

const ClosedLoopDriverProtocolConfig &Tmc2209ProtocolAdapter::config() const
{
  return config_;
}

bool Tmc2209ProtocolAdapter::writeRegister(uint8_t reg, uint32_t value)
{
  if (!isOfficialTmcRegister(reg))
  {
    return false;
  }

  const uint32_t request_word = buildRequestWord(reg, value);
  const uint32_t payload = request_word & 0xFFFFUL;
  return applyOfficialRegister(reg, payload);
}

bool Tmc2209ProtocolAdapter::readRegister(uint8_t reg, uint32_t *value)
{
  if (!isOfficialTmcRegister(reg))
  {
    return false;
  }

  uint32_t raw_value = 0U;
  if (!readOfficialRegister(reg, &raw_value))
  {
    return false;
  }

  if (value != nullptr)
  {
    *value = decodeResponseWord(reg, raw_value);
  }
  return true;
}

bool Tmc2209ProtocolAdapter::setCustomParameter(uint16_t id, uint32_t value)
{
  if (!isCustomExtensionRegister(id))
  {
    return false;
  }
  const uint16_t index = static_cast<uint16_t>(id - 0x100U);
  if (index >= 8U)
  {
    return false;
  }
  custom_parameters_[index] = value;
  return true;
}

bool Tmc2209ProtocolAdapter::getCustomParameter(uint16_t id, uint32_t *value) const
{
  if (!isCustomExtensionRegister(id) || value == nullptr)
  {
    return false;
  }
  const uint16_t index = static_cast<uint16_t>(id - 0x100U);
  if (index >= 8U)
  {
    return false;
  }
  *value = custom_parameters_[index];
  return true;
}

void Tmc2209ProtocolAdapter::attachDriver(StepperDriver *driver)
{
  driver_ = driver;
  if (driver_ != nullptr)
  {
    syncToDriver();
  }
}

void Tmc2209ProtocolAdapter::syncToDriver()
{
  if (driver_ == nullptr)
  {
    return;
  }

  driver_->setSingleWireUart(config_.uart_port, config_.uart_pin, config_.use_single_wire_uart);
  driver_->setSenseResistor(config_.sense_resistor_ohm);
  driver_->setMicrosteps(config_.microsteps);
  driver_->setInterpolation(config_.microsteps, config_.interpolation_microsteps);
  driver_->setRunCurrent(config_.run_current_a);
  driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
  driver_->setSilentMode(config_.silent_mode);
}
