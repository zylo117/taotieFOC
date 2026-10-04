#ifndef USB_CDC_PROTOCOL_H
#define USB_CDC_PROTOCOL_H

#include <stdint.h>

#include "closed_loop_controller.h"

class UsbCdcProtocolBridge
{
public:
    UsbCdcProtocolBridge();

    void init(ClosedLoopController* controller, void* udev);
    void poll();
    void sendTelemetry();

private:
    void handleFrame(const uint8_t* frame, uint16_t len);
    bool sendFrame(uint8_t cmd, uint16_t reg, uint32_t value);
    void flushTxQueue();
    void serviceCalibrationTableDump();
    static uint8_t crc8(const uint8_t* data, uint16_t len);

    ClosedLoopController* controller_;
    void* udev_;
    volatile bool calibration_dump_active_;
    volatile uint8_t calibration_dump_type_;
    volatile uint16_t calibration_dump_pair_index_;
    volatile uint16_t calibration_dump_pair_count_;
    volatile uint16_t calibration_dump_pair_offset_;
    uint8_t rx_buffer_[128];
    uint16_t rx_buffer_len_;
    uint8_t tx_response_queue_[16][10];
    uint8_t tx_response_head_;
    uint8_t tx_response_tail_;
    uint8_t tx_response_count_;
    uint8_t tx_telemetry_frame_[10];
    uint16_t tx_telemetry_generation_;
    bool tx_telemetry_pending_;
};

#endif
