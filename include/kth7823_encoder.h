#ifndef KTH7823_ENCODER_H
#define KTH7823_ENCODER_H

#include "angle_encoder.h"

class Kth7823Encoder : public AngleEncoder
{
public:
  Kth7823Encoder();
  bool init() override;
  uint16_t readRawAngle() override;
  void setZero(uint16_t zero_angle) override;
  bool calibrate(const EncoderCalibrationConfig &config, EncoderCalibrationResult *result) override;

private:
  uint16_t zero_angle_;
};

#endif
