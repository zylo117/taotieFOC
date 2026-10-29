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

    // 接收侧异常计数（替代 poll() 里的 printf）：
    // rxOverflowCount = 单包超出 rx_buffer_ 容量的次数，
    // rxCrcFailCount  = CRC 不匹配被丢弃的帧数。
    // 两者持续增长说明上位机发帧有问题（波特率/帧格式/CRC），
    // 但不会再把这个 1KB 栈任务的调用链拖进栈溢出。
    uint32_t rxOverflowCount() const { return rx_overflow_count_; }
    uint32_t rxCrcFailCount() const { return rx_crc_fail_count_; }

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
    uint32_t rx_overflow_count_ = 0U;
    uint32_t rx_crc_fail_count_ = 0U;
};

#endif
