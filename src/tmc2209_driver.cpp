#include "tmc2209_driver.h"
#include "at32f403a_407_board.h"

Tmc2209Driver::Tmc2209Driver()
  : enabled_(false), direction_(false), single_wire_uart_(false),
    config_{32U, 256U, 0.8f, 0.2f, true, false, 0.110f, 0.0f, 115200U, true, false, TMC2209_UART_GPIO, TMC2209_UART_PIN},
    uart_port_(TMC2209_UART_GPIO), uart_pin_(TMC2209_UART_PIN)
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
  uart_port_ = config_.uart_port != nullptr ? config_.uart_port : TMC2209_UART_GPIO;
  uart_pin_ = config_.uart_pin != 0U ? config_.uart_pin : TMC2209_UART_PIN;
  single_wire_uart_ = config_.use_single_wire_uart;
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
  if (config_.sense_resistor_ohm <= 0.0f)
  {
    config_.sense_resistor_ohm = 0.110f;
  }
  return true;
}

const StepperDriverConfig &Tmc2209Driver::config() const
{
  return config_;
}

bool Tmc2209Driver::writeRegister(uint8_t reg, uint32_t value)
{
  (void)reg;
  (void)value;
  return true;
}

bool Tmc2209Driver::readRegister(uint8_t reg, uint32_t *value)
{
  (void)reg;
  if (value != nullptr)
  {
    *value = 0U;
  }
  return true;
}

bool Tmc2209Driver::setMicrosteps(uint16_t microsteps)
{
  config_.microsteps = microsteps == 0U ? 32U : microsteps;
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
  config_.microsteps = input_microsteps == 0U ? 32U : input_microsteps;
  config_.interpolation_microsteps = target_microsteps == 0U ? 256U : target_microsteps;
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
  stepper_common::stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, enable);
}

void Tmc2209Driver::setDirection(bool direction)
{
  direction_ = direction;
  stepper_common::stepper_write_gpio(DIR_OUTPUT_PORT, DIR_OUT_PIN, direction);
}

void Tmc2209Driver::setStepState(bool state)
{
  stepper_common::stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, state);
}

void Tmc2209Driver::sendStepPulse(uint32_t width_us)
{
  stepper_common::stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, true);
  delay_us(width_us);
  stepper_common::stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, false);
}
