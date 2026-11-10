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

    // 把另一条通道（USART1/CH340）收到的字节灌进同一个帧解析器。
    // 必须在任务上下文调用（内部会走 handleFrame -> stopMotion / setEnable / printf）。
    void feedBytes(const uint8_t* data, uint16_t len);

    // 接收侧异常计数（替代 poll() 里的 printf）：
    // rxOverflowCount = 单包超出 rx_buffer_ 容量的次数，
    // rxCrcFailCount  = CRC 不匹配被丢弃的帧数。
    // 两者持续增长说明上位机发帧有问题（波特率/帧格式/CRC），
    // 但不会再把这个 1KB 栈任务的调用链拖进栈溢出。
    uint32_t rxOverflowCount() const { return rx_overflow_count_; }
    uint32_t rxCrcFailCount() const { return rx_crc_fail_count_; }
    // 响应队列（容量 16）满被丢弃的帧数。以前溢出是彻底静默的，主机只能靠
    // 1200ms 超时反推；现在挂到只读寄存器 0x16B，网页能直接看到。
    uint32_t txDropCount() const { return tx_drop_count_; }
    uint32_t writeDeniedCount() const { return write_denied_count_; }
    void recordWriteDenied() { write_denied_count_++; }

    // 单例指针：ParameterRouter 的只读寄存器（0x169/0x16A/0x16B/0x16C）要读这里的
    // 协议层计数，但 router 只拿得到 controller，不掌握 bridge 的生命周期。
    // bridge 在 main 里是静态对象，init() 时登记，永远有效。
    static UsbCdcProtocolBridge* instance() { return s_instance_; }
    static UsbCdcProtocolBridge* s_instance_;

private:
    void handleFrame(const uint8_t* frame, uint16_t len);
    // 把 rx_buffer_ 里已凑齐的帧逐个解析掉（poll 和 feedBytes 共用）
    void dispatchRxBuffer();
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
    uint32_t tx_drop_count_ = 0U;
    uint32_t write_denied_count_ = 0U;
};

#endif
