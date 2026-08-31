#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "at32f403a_407.h"

#define STEP_OUTPUT_PORT         GPIOA
#define STEP_OUT_PIN             GPIO_PINS_15
#define DIR_OUTPUT_PORT          GPIOA
#define DIR_OUT_PIN              GPIO_PINS_14
#define EN_OUTPUT_PORT           GPIOA
#define EN_OUT_PIN               GPIO_PINS_13

#define TMC2209_UART_GPIO        GPIOA
#define TMC2209_UART_PIN         GPIO_PINS_4

namespace stepper_common
{
void stepper_write_gpio(gpio_type *port, uint16_t pin, bool state);
void stepper_init_step_gpio(void);
void stepper_init_uart_gpio(gpio_type *port, uint16_t pin);
}

class StepperDriver
{
public:
  virtual ~StepperDriver() = default;
  virtual bool init() = 0;
  virtual bool writeRegister(uint8_t reg, uint32_t value) = 0;
  virtual void setEnable(bool enable) = 0;
  virtual void setDirection(bool direction) = 0;
  virtual void setStepState(bool state) = 0;
  virtual void sendStepPulse(uint32_t width_us) = 0;
};

#endif
