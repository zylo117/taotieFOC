#include "angle_encoder.h"
#include "at32f403a_407_board.h"

namespace encoder_common
{
static inline void encoder_spi_delay(void)
{
  for (volatile uint32_t count = 0U; count < 20U; ++count)
  {
    __NOP();
  }
}

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
  uint16_t rx_data = 0U;

  // KTH7823 uses SPI mode 3: clock idles high, sample on falling edge.
  encoder_write_gpio(KTH7823_SCLK_PORT, KTH7823_SCLK_PIN, true);

  // PB1/PB2/PB10/PB11 are the board's KTH7823 wiring, not a valid SPI2
  // alternate-function group on this AT32 variant. Use software SPI here.
  for (int8_t bit = 15; bit >= 0; --bit)
  {
    encoder_write_gpio(KTH7823_MOSI_PORT, KTH7823_MOSI_PIN,
                       ((tx_data >> bit) & 0x01U) != 0U);
    encoder_spi_delay();
    encoder_write_gpio(KTH7823_SCLK_PORT, KTH7823_SCLK_PIN, false);
    encoder_spi_delay();
    rx_data = static_cast<uint16_t>((rx_data << 1U) |
                                    (gpio_input_data_bit_read(KTH7823_MISO_PORT, KTH7823_MISO_PIN) ? 1U : 0U));
    encoder_write_gpio(KTH7823_SCLK_PORT, KTH7823_SCLK_PIN, true);
    encoder_spi_delay();
  }

  return rx_data;
}
}
