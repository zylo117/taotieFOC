#include "angle_encoder.h"
#include "at32f403a_407_board.h"

namespace encoder_common
{
void encoder_write_gpio(gpio_type *port, uint16_t pin, bool state)
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

void encoder_gpio_config_input(gpio_type *port, uint16_t pin)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
}

void encoder_gpio_config_output(gpio_type *port, uint16_t pin)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
}

uint16_t encoder_spi2_rw16(uint16_t tx_data)
{
  while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_TDBE_FLAG) == RESET)
  {
  }
  spi_i2s_data_transmit(KTH7823_SPI, tx_data);

  while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_RDBF_FLAG) == RESET)
  {
  }

  return spi_i2s_data_receive(KTH7823_SPI);
}
}
