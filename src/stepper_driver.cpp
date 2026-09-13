#include "stepper_driver.h"
#include "at32f403a_407_board.h"

namespace stepper_common
{
void stepper_write_gpio(gpio_type *port, uint16_t pin, bool state)
{
  if (state)
  {
    port->scr = pin;
  }
  else
  {
    port->clr = pin;
  }
}

void stepper_delay_us(uint32_t microseconds)
{
  const uint32_t cycles_per_us = system_core_clock / 1000000U;
  const uint32_t loop_count = (cycles_per_us / 4U) * microseconds;
  for (volatile uint32_t index = 0U; index < loop_count; ++index)
  {
    __NOP();
  }
}

void stepper_init_step_gpio(void)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = STEP_OUT_PIN | DIR_OUT_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = EN_OUT_PIN;
  gpio_init(EN_OUTPUT_PORT, &gpio_init_struct);
  stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, true);
  stepper_write_gpio(DIR_OUTPUT_PORT, DIR_OUT_PIN, false);
  stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, false);
}

void stepper_init_uart_gpio(gpio_type *port, uint16_t pin)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
  stepper_write_gpio(port, pin, true);
}
}
