#ifndef TMC2208_DRIVER_H
#define TMC2208_DRIVER_H

#include "stepper_driver.h"

class Tmc2208Driver : public StepperDriver
{
public:
  Tmc2208Driver();
  bool init() override;
  bool configure(const StepperDriverConfig &config) override;
  const StepperDriverConfig &config() const override;
  bool writeRegister(uint8_t reg, uint32_t value) override;
  bool readRegister(uint8_t reg, uint32_t *value) override;
  void setEnable(bool enable) override;
  void setDirection(bool direction) override;
  void setStepState(bool state) override;
  void sendStepPulse(uint32_t width_us) override;
  bool setMicrosteps(uint16_t microsteps) override;
  bool setRunCurrent(float amps) override;
  bool setHoldCurrent(float amps, bool enabled) override;
  bool setSilentMode(bool enable) override;
  bool setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps) override;
  bool setSenseResistor(float ohm) override;
  bool setSingleWireUart(gpio_type *port, uint16_t pin, bool enable) override;

private:
  bool enabled_;
  bool direction_;
  StepperDriverConfig config_;
};

#endif
