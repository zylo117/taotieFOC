#ifndef USB_CDC_PROTOCOL_H
#define USB_CDC_PROTOCOL_H

#include <stdint.h>

#include "closed_loop_controller.h"

class UsbCdcProtocolBridge
{
public:
  UsbCdcProtocolBridge();

  void init(ClosedLoopController *controller, void *udev);
  void poll();
  void sendTelemetry();

private:
  void handleFrame(const uint8_t *frame, uint16_t len);
  void sendFrame(uint8_t cmd, uint16_t reg, uint32_t value);
  static uint8_t crc8(const uint8_t *data, uint16_t len);

  ClosedLoopController *controller_;
  void *udev_;
};

#endif
