#include "kth7823_encoder.h"
#include <math.h>
#include "at32f403a_407_board.h"

namespace
{
uint16_t normalize_angle(uint16_t raw, uint16_t zero)
{
  if (raw >= zero)
  {
    return raw - zero;
  }
  return zero - raw;
}
}

Kth7823Encoder::Kth7823Encoder() : zero_angle_(0U)
{
}

bool Kth7823Encoder::init()
{
  spi_init_type spi_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

  encoder_common::encoder_gpio_config_output(KTH7823_CS_PORT, KTH7823_CS_PIN);
  encoder_common::encoder_gpio_config_output(KTH7823_SCLK_PORT, KTH7823_SCLK_PIN);
  encoder_common::encoder_gpio_config_output(KTH7823_MOSI_PORT, KTH7823_MOSI_PIN);
  encoder_common::encoder_gpio_config_input(KTH7823_MISO_PORT, KTH7823_MISO_PIN);

  encoder_common::encoder_gpio_config_input(KTH7823_MGH_PORT, KTH7823_MGH_PIN);
  encoder_common::encoder_gpio_config_input(KTH7823_MGL_PORT, KTH7823_MGL_PIN);

  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);

  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(KTH7823_SPI, &spi_init_struct);
  spi_enable(KTH7823_SPI, TRUE);

  zero_angle_ = readRawAngle();
  return true;
}

uint16_t Kth7823Encoder::readRawAngle()
{
  uint16_t raw = 0U;
  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, false);
  raw = encoder_common::encoder_spi2_rw16(0x0000U);
  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);
  return raw;
}

void Kth7823Encoder::setZero(uint16_t zero_angle)
{
  zero_angle_ = zero_angle;
}

bool Kth7823Encoder::calibrate(const EncoderCalibrationConfig &config, EncoderCalibrationResult *result)
{
  if (result != nullptr)
  {
    result->offset_ok = false;
    result->direction_ok = false;
    result->noise_ok = false;
    result->walk_ok = false;
    result->offset_correction = 0;
    result->noise_rms = 0.0f;
    result->walk_peak = 0.0f;
  }

  if (!config.enable_offset_calibration && !config.enable_direction_calibration &&
      !config.enable_noise_calibration && !config.enable_walk_calibration)
  {
    return false;
  }

  if (result != nullptr)
  {
    result->offset_ok = true;
    result->direction_ok = true;
    result->noise_ok = true;
    result->walk_ok = true;
    result->offset_correction = config.offset_correction;
  }

  if (config.enable_direction_calibration)
  {
    uint16_t base = readRawAngle();
    uint16_t next = readRawAngle();
    if (result != nullptr)
    {
      result->direction_ok = normalize_angle(next, base) < config.sample_count;
    }
  }

  if (config.enable_noise_calibration && result != nullptr)
  {
    result->noise_rms = 0.0f;
    for (uint16_t i = 0U; i < config.sample_count; ++i)
    {
      const uint16_t raw = readRawAngle();
      const uint16_t diff = normalize_angle(raw, zero_angle_);
      result->noise_rms += static_cast<float>(diff * diff);
    }
    result->noise_rms = sqrt(result->noise_rms / static_cast<float>(config.sample_count));
    result->noise_ok = result->noise_rms <= config.noise_threshold;
  }

  if (config.enable_walk_calibration && result != nullptr)
  {
    result->walk_peak = 0.0f;
    uint16_t previous = readRawAngle();
    for (uint16_t i = 0U; i < config.sample_count; ++i)
    {
      const uint16_t current = readRawAngle();
      const uint16_t delta = normalize_angle(current, previous);
      if (delta > result->walk_peak)
      {
        result->walk_peak = static_cast<float>(delta);
      }
      previous = current;
    }
    result->walk_ok = result->walk_peak <= config.walk_threshold;
  }

  return true;
}
