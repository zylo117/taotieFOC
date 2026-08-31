#ifndef TMC5160_DRIVER_H
#define TMC5160_DRIVER_H

#include "stepper_driver.h"

class Tmc5160Driver : public StepperDriver
{
public:
  Tmc5160Driver();
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
