#ifndef ANGLE_ENCODER_H
#define ANGLE_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "at32f403a_407.h"

#define KTH7823_SPI              SPI2
#define KTH7823_SCLK_PORT        GPIOB
#define KTH7823_SCLK_PIN         GPIO_PINS_11
#define KTH7823_MISO_PORT        GPIOB
#define KTH7823_MISO_PIN         GPIO_PINS_2
#define KTH7823_MOSI_PORT        GPIOB
#define KTH7823_MOSI_PIN         GPIO_PINS_1
#define KTH7823_CS_PORT          GPIOB
#define KTH7823_CS_PIN           GPIO_PINS_10
#define KTH7823_MGH_PORT         GPIOB
#define KTH7823_MGH_PIN          GPIO_PINS_13
#define KTH7823_MGL_PORT         GPIOB
#define KTH7823_MGL_PIN          GPIO_PINS_12

struct EncoderCalibrationConfig
{
  bool enable_offset_calibration;
  bool enable_direction_calibration;
  bool enable_noise_calibration;
  bool enable_walk_calibration;
  uint16_t sample_count;
  float noise_threshold;
  float walk_threshold;
  int16_t offset_correction;
  bool invert_direction;
};

struct EncoderCalibrationResult
{
  bool offset_ok;
  bool direction_ok;
  bool noise_ok;
  bool walk_ok;
  int32_t offset_correction;
  float noise_rms;
  float walk_peak;
};

namespace encoder_common
{
void encoder_write_gpio(gpio_type *port, uint16_t pin, bool state);
void encoder_gpio_config_input(gpio_type *port, uint16_t pin);
void encoder_gpio_config_output(gpio_type *port, uint16_t pin);
uint16_t encoder_spi2_rw16(uint16_t tx_data);
}

class AngleEncoder
{
public:
  virtual ~AngleEncoder() = default;
  virtual bool init() = 0;
  virtual uint16_t readRawAngle() = 0;
  virtual void setZero(uint16_t zero_angle) = 0;
  virtual bool calibrate(const EncoderCalibrationConfig &config, EncoderCalibrationResult *result) = 0;
};

#endif
