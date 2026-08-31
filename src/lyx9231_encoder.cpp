#include "lyx9231_encoder.h"

Lyx9231Encoder::Lyx9231Encoder() : zero_angle_(0U)
{
}

bool Lyx9231Encoder::init()
{
  zero_angle_ = 0U;
  return true;
}

uint16_t Lyx9231Encoder::readRawAngle()
{
  return zero_angle_;
}

void Lyx9231Encoder::setZero(uint16_t zero_angle)
{
  zero_angle_ = zero_angle;
}

bool Lyx9231Encoder::calibrate(const EncoderCalibrationConfig &config, EncoderCalibrationResult *result)
{
  (void)config;
  if (result != nullptr)
  {
    result->offset_ok = true;
    result->direction_ok = true;
    result->noise_ok = true;
    result->walk_ok = true;
    result->offset_correction = 0;
    result->noise_rms = 0.0f;
    result->walk_peak = 0.0f;
  }
  return true;
}
