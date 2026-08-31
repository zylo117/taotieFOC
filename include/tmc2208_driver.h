#ifndef TMC2208_DRIVER_H
#define TMC2208_DRIVER_H

#include "stepper_driver.h"

class Tmc2208Driver : public StepperDriver
{
public:
  Tmc2208Driver();
  bool init() override;
  bool writeRegister(uint8_t reg, uint32_t value) override;
  void setEnable(bool enable) override;
  void setDirection(bool direction) override;
  void setStepState(bool state) override;
  void sendStepPulse(uint32_t width_us) override;

private:
  bool enabled_;
  bool direction_;
};

#endif
