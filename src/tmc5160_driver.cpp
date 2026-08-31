#include "tmc5160_driver.h"
#include "at32f403a_407_board.h"

Tmc5160Driver::Tmc5160Driver() : enabled_(false), direction_(false)
{
}

bool Tmc5160Driver::init()
{
  stepper_common::stepper_init_step_gpio();
  return true;
}

bool Tmc5160Driver::writeRegister(uint8_t reg, uint32_t value)
{
  (void)reg;
  (void)value;
  return true;
}

void Tmc5160Driver::setEnable(bool enable)
{
  enabled_ = enable;
  stepper_common::stepper_write_gpio(GPIOA, EN_OUT_PIN, enable);
}

void Tmc5160Driver::setDirection(bool direction)
{
  direction_ = direction;
  stepper_common::stepper_write_gpio(GPIOA, DIR_OUT_PIN, direction);
}

void Tmc5160Driver::sendStepPulse(uint32_t width_us)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, true);
  delay_us(width_us);
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, false);
}

void Tmc5160Driver::setStepState(bool state)
{
  stepper_common::stepper_write_gpio(GPIOA, STEP_OUT_PIN, state);
}
