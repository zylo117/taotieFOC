#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include "stepper_driver.h"

class Tmc2209Driver : public StepperDriver
{
public:
  Tmc2209Driver();
  bool init() override;
  bool configure(const StepperDriverConfig &config) override;
  const StepperDriverConfig &config() const override;
  bool writeRegister(uint8_t reg, uint32_t value) override;
  bool readRegister(uint8_t reg, uint32_t *value);
  bool configureKlipperCompatible(const StepperDriverConfig &config);
  bool uartWriteByte(uint8_t byte);
  bool uartReadByte(uint8_t *byte);
  void setEnable(bool enable) override;
  void setDirection(bool direction) override;
  void setStepState(bool state) override;
  void sendStepPulse(uint32_t width_us) override;
  bool setMicrosteps(uint16_t microsteps) override;
  bool setRunCurrent(float ma) override;
  bool setHoldCurrent(float ma, bool enabled) override;
  bool setSilentMode(bool enable) override;
  bool setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps) override;
  bool setSenseResistor(float ohm) override;
  bool setSingleWireUart(gpio_type *port, uint16_t pin, bool enable) override;

private:
  bool sendSingleWireByte(uint8_t byte, bool expect_reply);
  bool enabled_;
  bool direction_;
  bool single_wire_uart_;
  StepperDriverConfig config_;
  gpio_type *uart_port_;
  uint16_t uart_pin_;
};

#endif
