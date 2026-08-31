#include "tmc2209_driver.h"
#include <string.h>
#include "at32f403a_407_board.h"

namespace
{
constexpr uint32_t kTmc2209UartBaud = 115200U;
constexpr uint32_t kTmc2209FrameBreakBits = 8U;

uint16_t clampToMicrosteps(uint16_t microsteps)
{
  if (microsteps < 2U)
  {
    return 2U;
  }
  if (microsteps > 256U)
  {
    return 256U;
  }
  return microsteps;
}

bool isKlipperCompatibleAddress(uint8_t reg)
{
  switch (reg)
  {
    case 0x00U:
    case 0x01U:
    case 0x10U:
    case 0x11U:
    case 0x12U:
    case 0x13U:
    case 0x14U:
    case 0x15U:
    case 0x16U:
    case 0x17U:
    case 0x18U:
    case 0x19U:
    case 0x1AU:
    case 0x1BU:
    case 0x1CU:
    case 0x1DU:
    case 0x1EU:
    case 0x20U:
    case 0x21U:
    case 0x22U:
    case 0x23U:
    case 0x24U:
    case 0x25U:
    case 0x26U:
    case 0x27U:
    case 0x28U:
    case 0x29U:
    case 0x2AU:
    case 0x2BU:
    case 0x2CU:
    case 0x2DU:
    case 0x2EU:
    case 0x2FU:
      return true;
    default:
      return false;
  }
}

uint32_t buildKlipperUartFrame(uint8_t reg, uint32_t value)
{
  return ((static_cast<uint32_t>(reg) << 16U) | (value & 0xFFFFUL));
}
}

Tmc2209Driver::Tmc2209Driver()
  : enabled_(false), direction_(false), single_wire_uart_(false),
    config_{32U, 256U, 0.8f, 0.2f, true, false, 0.110f, 0.0f, 115200U, true, false, GPIOA, TMC2209_UART_PIN},
    uart_port_(GPIOA), uart_pin_(TMC2209_UART_PIN)
{
}

bool Tmc2209Driver::init()
{
  stepper_common::stepper_init_step_gpio();
  stepper_common::stepper_init_uart_gpio(uart_port_, uart_pin_);
  single_wire_uart_ = config_.use_single_wire_uart;
  return true;
}

bool Tmc2209Driver::configure(const StepperDriverConfig &config)
{
  config_ = config;
  uart_port_ = config_.uart_port != nullptr ? config_.uart_port : GPIOA;
  uart_pin_ = config_.uart_pin != 0U ? config_.uart_pin : TMC2209_UART_PIN;
  single_wire_uart_ = config_.use_single_wire_uart;
  if (config_.uart_baudrate == 0U)
  {
    config_.uart_baudrate = kTmc2209UartBaud;
  }
  if (config_.microsteps == 0U)
  {
    config_.microsteps = 32U;
  }
  if (config_.interpolation_microsteps == 0U)
  {
    config_.interpolation_microsteps = 256U;
  }
  return true;
}

const StepperDriverConfig &Tmc2209Driver::config() const
{
  return config_;
}

bool Tmc2209Driver::configureKlipperCompatible(const StepperDriverConfig &config)
{
  StepperDriverConfig cfg = config;
  cfg.uart_baudrate = 115200U;
  cfg.use_single_wire_uart = true;
  cfg.uart_port = GPIOA;
  cfg.uart_pin = TMC2209_UART_PIN;
  cfg.microsteps = clampToMicrosteps(cfg.microsteps == 0U ? 32U : cfg.microsteps);
  cfg.interpolation_microsteps = cfg.interpolation_microsteps == 0U ? 256U : cfg.interpolation_microsteps;
  return configure(cfg);
}

bool Tmc2209Driver::sendSingleWireByte(uint8_t byte, bool expect_reply)
{
  (void)expect_reply;
  const uint32_t bit_time_us = 1000000UL / config_.uart_baudrate;
  stepper_common::stepper_write_gpio(uart_port_, uart_pin_, false);
  delay_us(bit_time_us / 2U);
  for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index)
  {
    const bool bit_state = (byte & 0x01U) != 0U;
    stepper_common::stepper_write_gpio(uart_port_, uart_pin_, bit_state);
    delay_us(bit_time_us);
    byte >>= 1U;
  }
  stepper_common::stepper_write_gpio(uart_port_, uart_pin_, true);
  delay_us(bit_time_us / 2U);
  return true;
}

bool Tmc2209Driver::uartWriteByte(uint8_t byte)
{
  if (!single_wire_uart_)
  {
    return false;
  }
  return sendSingleWireByte(byte, false);
}

bool Tmc2209Driver::uartReadByte(uint8_t *byte)
{
  if (!single_wire_uart_ || byte == nullptr)
  {
    return false;
  }
  *byte = 0U;
  stepper_common::stepper_write_gpio(uart_port_, uart_pin_, true);
  delay_us(1000000UL / config_.uart_baudrate);
  return true;
}

bool Tmc2209Driver::writeRegister(uint8_t reg, uint32_t value)
{
  if (!isKlipperCompatibleAddress(reg))
  {
    return false;
  }

  const uint32_t frame = buildKlipperUartFrame(reg, value);
  const uint8_t *frame_bytes = reinterpret_cast<const uint8_t *>(&frame);
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    sendSingleWireByte(frame_bytes[index], false);
  }
  return true;
}

bool Tmc2209Driver::readRegister(uint8_t reg, uint32_t *value)
{
  if (value == nullptr || !isKlipperCompatibleAddress(reg))
  {
    return false;
  }

  uint32_t read_value = 0U;
  uint8_t *read_bytes = reinterpret_cast<uint8_t *>(&read_value);
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    uint8_t byte = 0U;
    uartReadByte(&byte);
    read_bytes[index] = byte;
  }
  *value = read_value;
  return true;
}

bool Tmc2209Driver::setMicrosteps(uint16_t microsteps)
{
  config_.microsteps = clampToMicrosteps(microsteps);
  return true;
}

bool Tmc2209Driver::setRunCurrent(float amps)
{
  config_.run_current_a = amps;
  return true;
}

bool Tmc2209Driver::setHoldCurrent(float amps, bool enabled)
{
  config_.hold_current_a = amps;
  config_.enable_hold_current = enabled;
  return true;
}

bool Tmc2209Driver::setSilentMode(bool enable)
{
  config_.silent_mode = enable;
  return true;
}

bool Tmc2209Driver::setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps)
{
  config_.microsteps = clampToMicrosteps(input_microsteps);
  config_.interpolation_microsteps = clampToMicrosteps(target_microsteps);
  return true;
}

bool Tmc2209Driver::setSenseResistor(float ohm)
{
  if (ohm <= 0.0f)
  {
    return false;
  }
  config_.sense_resistor_ohm = ohm;
  return true;
}

bool Tmc2209Driver::setSingleWireUart(gpio_type *port, uint16_t pin, bool enable)
{
  if (port == nullptr)
  {
    return false;
  }

  uart_port_ = port;
  uart_pin_ = pin;
  single_wire_uart_ = enable;
  config_.uart_port = port;
  config_.uart_pin = pin;
  config_.use_single_wire_uart = enable;
  return true;
}

void Tmc2209Driver::setEnable(bool enable)
{
  enabled_ = enable;
  stepper_common::stepper_write_gpio(GPIOA, EN_OUT_PIN, enable);
}

void Tmc2209Driver::setDirection(bool direction)
{
  direction_ = direction;
  stepper_common::stepper_write_gpio(GPIOA, DIR_OUT_PIN, direction);
}

void Tmc2209Driver::setStepState(bool state)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, state);
}

void Tmc2209Driver::sendStepPulse(uint32_t width_us)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, true);
  delay_us(width_us);
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, false);
}
