#ifndef LYX9231_ENCODER_H
#define LYX9231_ENCODER_H

#include "angle_encoder.h"

class Lyx9231Encoder : public AngleEncoder
{
public:
  Lyx9231Encoder();
  bool init() override;
  uint16_t readRawAngle() override;
  void setZero(uint16_t zero_angle) override;
  bool calibrate(const EncoderCalibrationConfig &config, EncoderCalibrationResult *result) override;

private:
  uint16_t zero_angle_;
};

#endif
