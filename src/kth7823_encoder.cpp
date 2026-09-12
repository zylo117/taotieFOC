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

Kth7823Encoder::Kth7823Encoder()
  : zero_angle_(0U), last_raw_frame_(0U), last_tx_frame_(0U), read_count_(0U),
    all_ones_count_(0U), all_zeros_count_(0U)
{
}

bool Kth7823Encoder::init()
{
  spi_init_type spi_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

  encoder_common::encoder_gpio_config_output(KTH7823_CS_PORT, KTH7823_CS_PIN);

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = KTH7823_SCLK_PIN | KTH7823_MOSI_PIN;
  gpio_init(KTH7823_SCLK_PORT, &gpio_init_struct);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = KTH7823_MISO_PIN;
  gpio_init(KTH7823_MISO_PORT, &gpio_init_struct);

  encoder_common::encoder_gpio_config_input(KTH7823_MGH_PORT, KTH7823_MGH_PIN);
  encoder_common::encoder_gpio_config_input(KTH7823_MGL_PORT, KTH7823_MGL_PIN);

  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);

  spi_i2s_reset(KTH7823_SPI);
  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(KTH7823_SPI, &spi_init_struct);
  spi_enable(KTH7823_SPI, TRUE);

  uint32_t zero_sum = 0U;
  for (uint8_t sample = 0U; sample < 8U; ++sample)
  {
    zero_sum += readRawAngle();
  }
  zero_angle_ = static_cast<uint16_t>(zero_sum / 8U);
  return true;
}

uint16_t Kth7823Encoder::readRawAngle()
{
  uint16_t raw = 0U;
  last_tx_frame_ = 0x0300U;
  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, false);
  raw = encoder_common::encoder_spi2_rw16(last_tx_frame_);
  encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);
  last_raw_frame_ = raw;
  read_count_++;
  if (raw == 0xFFFFU)
  {
    all_ones_count_++;
  }
  else if (raw == 0x0000U)
  {
    all_zeros_count_++;
  }
  return raw;
}

uint16_t Kth7823Encoder::lastRawFrame() const
{
  return last_raw_frame_;
}

uint16_t Kth7823Encoder::lastTxFrame() const
{
  return last_tx_frame_;
}

uint32_t Kth7823Encoder::readCount() const
{
  return read_count_;
}

uint32_t Kth7823Encoder::allOnesCount() const
{
  return all_ones_count_;
}

uint32_t Kth7823Encoder::allZerosCount() const
{
  return all_zeros_count_;
}

uint8_t Kth7823Encoder::misoLevel() const
{
  return gpio_input_data_bit_read(KTH7823_MISO_PORT, KTH7823_MISO_PIN) != 0U ? 1U : 0U;
}

bool Kth7823Encoder::magneticFieldHigh() const
{
  return gpio_input_data_bit_read(KTH7823_MGH_PORT, KTH7823_MGH_PIN) != 0U;
}

bool Kth7823Encoder::magneticFieldLow() const
{
  return gpio_input_data_bit_read(KTH7823_MGL_PORT, KTH7823_MGL_PIN) != 0U;
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
