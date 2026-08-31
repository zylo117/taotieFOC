#include "tmc2208_driver.h"
#include "at32f403a_407_board.h"

Tmc2208Driver::Tmc2208Driver()
  : enabled_(false), direction_(false), config_{32U, 256U, 0.8f, 0.2f, true, true, 0.110f, 0.0f, 115200U, true, false, GPIOA, TMC2209_UART_PIN}
{
}

bool Tmc2208Driver::init()
{
  stepper_common::stepper_init_step_gpio();
  return true;
}

bool Tmc2208Driver::configure(const StepperDriverConfig &config)
{
  config_ = config;
  if (config_.uart_baudrate == 0U)
  {
    config_.uart_baudrate = 115200U;
  }
  if (config_.sense_resistor_ohm <= 0.0f)
  {
    config_.sense_resistor_ohm = 0.110f;
  }
  return true;
}

const StepperDriverConfig &Tmc2208Driver::config() const
{
  return config_;
}

bool Tmc2208Driver::writeRegister(uint8_t reg, uint32_t value)
{
  (void)reg;
  (void)value;
  return true;
}

bool Tmc2208Driver::readRegister(uint8_t reg, uint32_t *value)
{
  (void)reg;
  if (value != nullptr)
  {
    *value = 0U;
  }
  return true;
}

void Tmc2208Driver::setEnable(bool enable)
{
  enabled_ = enable;
  stepper_common::stepper_write_gpio(GPIOA, EN_OUT_PIN, enable);
}

void Tmc2208Driver::setDirection(bool direction)
{
  direction_ = direction;
  stepper_common::stepper_write_gpio(GPIOA, DIR_OUT_PIN, direction);
}

void Tmc2208Driver::setStepState(bool state)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, state);
}

void Tmc2208Driver::sendStepPulse(uint32_t width_us)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, true);
  delay_us(width_us);
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, false);
}

bool Tmc2208Driver::setMicrosteps(uint16_t microsteps)
{
  config_.microsteps = microsteps == 0U ? 32U : microsteps;
  return true;
}

bool Tmc2208Driver::setRunCurrent(float amps)
{
  config_.run_current_a = amps;
  return true;
}

bool Tmc2208Driver::setHoldCurrent(float amps, bool enabled)
{
  config_.hold_current_a = amps;
  config_.enable_hold_current = enabled;
  return true;
}

bool Tmc2208Driver::setSilentMode(bool enable)
{
  config_.silent_mode = enable;
  return true;
}

bool Tmc2208Driver::setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps)
{
  config_.microsteps = input_microsteps == 0U ? 32U : input_microsteps;
  config_.interpolation_microsteps = target_microsteps == 0U ? 256U : target_microsteps;
  return true;
}

bool Tmc2208Driver::setSenseResistor(float ohm)
{
  if (ohm <= 0.0f)
  {
    return false;
  }
  config_.sense_resistor_ohm = ohm;
  return true;
}

bool Tmc2208Driver::setSingleWireUart(gpio_type *port, uint16_t pin, bool enable)
{
  if (port == nullptr)
  {
    return false;
  }
  config_.uart_port = port;
  config_.uart_pin = pin;
  config_.use_single_wire_uart = enable;
  return true;
}
