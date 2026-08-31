#include "tmc2208_driver.h"
#include "at32f403a_407_board.h"

Tmc2208Driver::Tmc2208Driver() : enabled_(false), direction_(false)
{
}

bool Tmc2208Driver::init()
{
  stepper_common::stepper_init_step_gpio();
  return true;
}

bool Tmc2208Driver::writeRegister(uint8_t reg, uint32_t value)
{
  (void)reg;
  (void)value;
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
