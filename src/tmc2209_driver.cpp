#include "tmc2209_driver.h"
#include "at32f403a_407_board.h"

Tmc2209Driver::Tmc2209Driver() : enabled_(false), direction_(false), uart_port_(GPIOA), uart_pin_(TMC2209_UART_PIN)
{
}

bool Tmc2209Driver::init()
{
  stepper_common::stepper_init_step_gpio();
  stepper_common::stepper_init_uart_gpio(uart_port_, uart_pin_);
  return true;
}

bool Tmc2209Driver::writeRegister(uint8_t reg, uint32_t value)
{
  uint32_t frame = ((uint32_t)reg << 16U) | (value & 0xFFFFUL);
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    uint8_t byte = static_cast<uint8_t>((frame >> (index * 8U)) & 0xFFU);

    stepper_common::stepper_write_gpio(uart_port_, uart_pin_, false);
    for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index)
    {
      delay_us(1000U / (115200U / 1000U));
      stepper_common::stepper_write_gpio(uart_port_, uart_pin_, (byte & 0x01U) != 0U);
      byte >>= 1U;
    }
    delay_us(1000U / (115200U / 1000U));
    stepper_common::stepper_write_gpio(uart_port_, uart_pin_, true);
  }
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
