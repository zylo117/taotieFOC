#include "usb_cdc_protocol.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cdc_class.h"
#include "usbd_core.h"

namespace
{
    const uint8_t kFrameHeaderA = 0x55U;
    const uint8_t kFrameHeaderB = 0xAAU;
    const uint8_t kCmdRead = 0x01U;
    const uint8_t kCmdWrite = 0x02U;
    const uint16_t kMaxRxBytes = 64U;

    uint32_t read_u32_be(const uint8_t* buf)
    {
        return ((uint32_t)buf[0] << 24U) |
            ((uint32_t)buf[1] << 16U) |
            ((uint32_t)buf[2] << 8U) |
            (uint32_t)buf[3];
    }

    void write_u32_be(uint8_t* buf, uint32_t value)
    {
        buf[0] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
        buf[1] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
        buf[2] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
        buf[3] = static_cast<uint8_t>(value & 0xFFU);
    }
}

UsbCdcProtocolBridge::UsbCdcProtocolBridge()
        : controller_(nullptr), udev_(nullptr), calibration_dump_active_(false),
            calibration_dump_type_(0U), calibration_dump_pair_index_(0U),
            calibration_dump_pair_count_(0U), calibration_dump_pair_offset_(0U),
            rx_buffer_{0U}, rx_buffer_len_(0U), tx_response_queue_{},
            tx_response_head_(0U), tx_response_tail_(0U), tx_response_count_(0U),
            tx_telemetry_frame_{0U}, tx_telemetry_generation_(0U), tx_telemetry_pending_(false)
{
}

void UsbCdcProtocolBridge::init(ClosedLoopController* controller, void* udev)
{
    controller_ = controller;
    udev_ = udev;
}

void UsbCdcProtocolBridge::poll()
{
    if (udev_ == nullptr || controller_ == nullptr)
    {
        return;
    }

    uint8_t recv_buf[kMaxRxBytes];
    const uint16_t rx_len = usb_vcp_get_rxdata(udev_, recv_buf);
    if (rx_len > 0U)
    {
        printf("[USB_RX] packet_len=%u\r\n", static_cast<unsigned>(rx_len));
        if (rx_len > sizeof(rx_buffer_) - rx_buffer_len_)
        {
            printf("[USB_RX] BUFFER_OVERFLOW buffered=%u incoming=%u; dropping partial frame\r\n",
                   static_cast<unsigned>(rx_buffer_len_), static_cast<unsigned>(rx_len));
            rx_buffer_len_ = 0U;
        }
        memcpy(rx_buffer_ + rx_buffer_len_, recv_buf, rx_len);
        rx_buffer_len_ = static_cast<uint16_t>(rx_buffer_len_ + rx_len);
    }

    const uint16_t frame_len = 10U;
    while (rx_buffer_len_ >= 2U)
    {
        if (rx_buffer_[0] != kFrameHeaderA || rx_buffer_[1] != kFrameHeaderB)
        {
            memmove(rx_buffer_, rx_buffer_ + 1U, rx_buffer_len_ - 1U);
            rx_buffer_len_--;
            continue;
        }
        if (rx_buffer_len_ < frame_len)
        {
            break;
        }

        const uint8_t crc = rx_buffer_[9];
        const uint8_t expected_crc = crc8(rx_buffer_, 9U);
        if (crc != expected_crc)
        {
            printf("[USB_RX] CRC_FAIL cmd=0x%02X reg=0x%04X got=0x%02X expected=0x%02X\r\n",
                   rx_buffer_[2],
                   (static_cast<unsigned>(rx_buffer_[3]) << 8U) | static_cast<unsigned>(rx_buffer_[4]),
                   crc, expected_crc);
            memmove(rx_buffer_, rx_buffer_ + 1U, rx_buffer_len_ - 1U);
            rx_buffer_len_--;
            continue;
        }

        handleFrame(rx_buffer_, frame_len);
        rx_buffer_len_ = static_cast<uint16_t>(rx_buffer_len_ - frame_len);
        if (rx_buffer_len_ > 0U)
        {
            memmove(rx_buffer_, rx_buffer_ + frame_len, rx_buffer_len_);
        }
    }
    if (rx_len > 0U)
    {
        printf("[USB_RX] buffered_bytes=%u\r\n", static_cast<unsigned>(rx_buffer_len_));
    }
    serviceCalibrationTableDump();
    flushTxQueue();
}

void UsbCdcProtocolBridge::handleFrame(const uint8_t* frame, uint16_t len)
{
    if (frame == nullptr || len < 10U)
    {
        return;
    }

    const uint8_t cmd = frame[2];
    const uint16_t reg = (static_cast<uint16_t>(frame[3]) << 8U) | static_cast<uint16_t>(frame[4]);
    const uint32_t value = read_u32_be(frame + 5U);

    if (cmd == kCmdWrite && reg == TMC2209_EXT_PARAM_CALIBRATION_TABLE_DUMP)
    {
        const uint8_t dump_type = static_cast<uint8_t>(value & 0xFFU);
        const uint16_t dump_page = static_cast<uint16_t>((value >> 16U) & 0xFFFFU);
        calibration_dump_type_ = dump_type;
        calibration_dump_pair_index_ = 0U;
        calibration_dump_pair_offset_ = 0U;
        if (dump_type == 0U)
        {
            calibration_dump_pair_count_ = (dump_page == 0U) ? (CALIBRATION_TABLE_SIZE / 2U) : 128U;
            calibration_dump_pair_offset_ = (dump_page == 0U) ? 0U : (dump_page * 128U);
            calibration_dump_active_ = true;
        }
        else if (dump_type == 1U)
        {
            calibration_dump_pair_count_ = 128U;
            calibration_dump_pair_offset_ = dump_page * 128U;
            calibration_dump_active_ = true;
        }
        else if (dump_type == 2U)
        {
            calibration_dump_pair_count_ = 0U;
            calibration_dump_active_ = true;
        }
        else
        {
            calibration_dump_active_ = false;
        }
        sendFrame(0x82U, reg, value);
        return;
    }

        const bool is_calibration_lookup =
         reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED ||
         reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW;
        if (is_calibration_lookup && cmd == kCmdWrite)
        {
         printf("[CAL_LOOKUP] RX WRITE reg=0x%04X input=%lu\r\n",
             reg, static_cast<unsigned long>(value));
         const bool write_ok = controller_->writeParameter(reg, value);
         uint32_t result = 0U;
         const bool readback_ok = controller_->readParameter(reg, &result);
         printf("[CAL_LOOKUP] COMPUTE write_ok=%u readback_ok=%u result=0x%04lX (%lu)\r\n",
             write_ok ? 1U : 0U, readback_ok ? 1U : 0U,
             static_cast<unsigned long>(result & 0xFFFFU),
             static_cast<unsigned long>(result & 0xFFFFU));
         const bool ack_sent = sendFrame(0x82U, reg, value);
         printf("[CAL_LOOKUP] TX WRITE_ACK reg=0x%04X status=%s\r\n",
             reg, ack_sent ? "QUEUED" : "QUEUE_FULL");
         return;
        }
        if (is_calibration_lookup && cmd == kCmdRead)
        {
         uint32_t result = 0U;
         const bool read_ok = controller_->readParameter(reg, &result);
         printf("[CAL_LOOKUP] RX READ reg=0x%04X read_ok=%u result=0x%08lX (%lu)\r\n",
             reg, read_ok ? 1U : 0U,
             static_cast<unsigned long>(result),
             static_cast<unsigned long>(result));
         const bool response_sent = sendFrame(0x81U, reg, result);
         printf("[CAL_LOOKUP] TX READ_REPLY reg=0x%04X status=%s\r\n",
             reg, response_sent ? "QUEUED" : "QUEUE_FULL");
         return;
        }

    uint32_t reply_value = 0U;
    if (cmd == kCmdRead)
    {
        controller_->readParameter(reg, &reply_value);
        sendFrame(0x81U, reg, reply_value);
    }
    else if (cmd == kCmdWrite)
    {
        if (reg == TMC2209_EXT_PARAM_TARGET_POSITION)
        {
            controller_->setTargetStep(static_cast<int32_t>(value));
        }
        else if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
        {
            controller_->setTargetAngleDeg(static_cast<float>(value) / 1000.0f);
        }
        controller_->writeParameter(reg, value);
        sendFrame(0x82U, reg, value);
    }
}

bool UsbCdcProtocolBridge::sendFrame(uint8_t cmd, uint16_t reg, uint32_t value)
{
    if (udev_ == nullptr)
    {
        return false;
    }

    uint8_t frame[10];
    frame[0] = kFrameHeaderA;
    frame[1] = kFrameHeaderB;
    frame[2] = cmd;
    frame[3] = static_cast<uint8_t>((reg >> 8U) & 0xFFU);
    frame[4] = static_cast<uint8_t>(reg & 0xFFU);
    write_u32_be(frame + 5U, value);
    frame[9] = crc8(frame, 9U);

    bool queued = false;
    taskENTER_CRITICAL();
    if (cmd == 0x10U)
    {
        memcpy(tx_telemetry_frame_, frame, sizeof(frame));
        tx_telemetry_generation_++;
        tx_telemetry_pending_ = true;
        queued = true;
    }
    else if (tx_response_count_ < 16U)
    {
        memcpy(tx_response_queue_[tx_response_tail_], frame, sizeof(frame));
        tx_response_tail_ = static_cast<uint8_t>((tx_response_tail_ + 1U) % 16U);
        tx_response_count_++;
        queued = true;
    }
    taskEXIT_CRITICAL();

    if (!queued)
    {
        printf("[USB_TX] QUEUE_FULL cmd=0x%02X reg=0x%04X\r\n", cmd, reg);
    }
    return queued;
}

void UsbCdcProtocolBridge::flushTxQueue()
{
    if (udev_ == nullptr)
    {
        return;
    }

    uint8_t frame[10];
    bool is_response = false;
    bool has_frame = false;
    uint16_t telemetry_generation = 0U;

    taskENTER_CRITICAL();
    if (tx_response_count_ > 0U)
    {
        memcpy(frame, tx_response_queue_[tx_response_head_], sizeof(frame));
        is_response = true;
        has_frame = true;
    }
    else if (tx_telemetry_pending_)
    {
        memcpy(frame, tx_telemetry_frame_, sizeof(frame));
        telemetry_generation = tx_telemetry_generation_;
        has_frame = true;
    }
    taskEXIT_CRITICAL();

    if (!has_frame || usb_vcp_send_data(udev_, frame, sizeof(frame)) != SUCCESS)
    {
        return;
    }

    if (is_response)
    {
        taskENTER_CRITICAL();
        if (tx_response_count_ > 0U)
        {
            tx_response_head_ = static_cast<uint8_t>((tx_response_head_ + 1U) % 16U);
            tx_response_count_--;
        }
        taskEXIT_CRITICAL();
    }
    else
    {
        taskENTER_CRITICAL();
        if (tx_telemetry_pending_ && tx_telemetry_generation_ == telemetry_generation)
        {
            tx_telemetry_pending_ = false;
        }
        taskEXIT_CRITICAL();
    }

    if (frame[2] != 0x10U)
    {
        const uint16_t reg = (static_cast<uint16_t>(frame[3]) << 8U) | frame[4];
        printf("[USB_TX] START cmd=0x%02X reg=0x%04X\r\n", frame[2], reg);
    }
}

void UsbCdcProtocolBridge::sendTelemetry()
{
    if (udev_ == nullptr || controller_ == nullptr || calibration_dump_active_)
    {
        return;
    }

    // Telemetry uses the same extension-register addresses as normal reads.
    const uint16_t telemetry_regs[] = {
        TMC2209_EXT_PARAM_FAULT_STATUS,
        TMC2209_EXT_PARAM_ENCODER_FAULT,
        TMC2209_EXT_PARAM_MAGNETIC_FAULT,
        TMC2209_EXT_PARAM_OUTPUT_STOP,
        TMC2209_EXT_PARAM_AB_CURRENT_A,
        TMC2209_EXT_PARAM_AB_CURRENT_B,
        TMC2209_EXT_PARAM_POS_LOOP_HZ,
        TMC2209_EXT_PARAM_VEL_LOOP_HZ,
        TMC2209_EXT_PARAM_CUR_LOOP_HZ,
        TMC2209_EXT_PARAM_TARGET_POSITION,
        TMC2209_EXT_PARAM_ACTUAL_POSITION,
        TMC2209_EXT_PARAM_FOLLOW_ERROR,
        TMC2209_EXT_PARAM_ENCODER_RAW,
        TMC2209_EXT_PARAM_MAGNETIC_HIGH,
        TMC2209_EXT_PARAM_MAGNETIC_LOW,
        TMC2209_EXT_PARAM_ENCODER_ZERO,
        TMC2209_EXT_PARAM_START_RPM,
        TMC2209_EXT_PARAM_MAX_RPM,
        TMC2209_EXT_PARAM_ACCEL_RPM_S,
        TMC2209_EXT_PARAM_PULSE_COUNT,
        TMC2209_EXT_PARAM_MOTION_MODE,
        TMC2209_EXT_PARAM_SPEED_RPM,
        TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS, TMC2209_EXT_PARAM_MOTOR_ENABLE,
        TMC2209_EXT_PARAM_MOTOR_DISABLE, TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS,
        TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT,
        TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT
    };
    static uint16_t telemetry_index = 0U;

    static uint8_t telemetry_phase = 0U;
    uint16_t register_to_send = 0U;
    uint32_t value_to_send = 0U;
    bool should_send = false;
    switch (telemetry_phase)
    {
    case 0U:
        register_to_send = TMC2209_EXT_PARAM_POSITION_DEG;
        should_send = controller_->readParameter(register_to_send, &value_to_send);
        break;
    case 1U:
        register_to_send = TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG;
        should_send = controller_->readParameter(register_to_send, &value_to_send);
        break;
    case 2U:
        register_to_send = TMC2209_EXT_PARAM_MOTION_COMMAND;
        should_send = controller_->readParameter(register_to_send, &value_to_send);
        break;
    default:
        register_to_send = telemetry_regs[telemetry_index];
        telemetry_index = static_cast<uint16_t>((telemetry_index + 1U) %
                                                  (sizeof(telemetry_regs) / sizeof(telemetry_regs[0])));
        should_send = controller_->readParameter(register_to_send, &value_to_send);
        break;
    }
    telemetry_phase = static_cast<uint8_t>((telemetry_phase + 1U) % 4U);
    if (should_send)
    {
        sendFrame(0x10U, register_to_send, value_to_send);
    }
}

void UsbCdcProtocolBridge::serviceCalibrationTableDump()
{
    if (!calibration_dump_active_ || controller_ == nullptr || udev_ == nullptr)
    {
        return;
    }

    if (calibration_dump_type_ == 2U)
    {
        uint32_t checksum = 0U;
        if (controller_->readCalibrationTableChecksum(&checksum))
        {
            if (!sendFrame(0x85U, 0U, checksum)) return;
        }
        else
        {
            if (!sendFrame(0x86U, 0U, 0xFFFFFFFFU)) return;
        }
        calibration_dump_active_ = false;
        return;
    }

    const uint16_t absolute_pair_index = static_cast<uint16_t>(calibration_dump_pair_offset_ + calibration_dump_pair_index_);
    if (calibration_dump_pair_index_ >= calibration_dump_pair_count_)
    {
        if (!sendFrame(0x86U, calibration_dump_pair_count_, calibration_dump_type_)) return;
        calibration_dump_active_ = false;
        return;
    }

    uint32_t packed_values = 0U;
    if (!controller_->readCalibrationTablePair(calibration_dump_type_, absolute_pair_index,
                                               &packed_values))
    {
        if (!sendFrame(0x86U, absolute_pair_index, 0xFFFFFFFFU)) return;
        calibration_dump_active_ = false;
        return;
    }

    const uint8_t response_cmd = calibration_dump_type_ == 0U ? 0x83U : 0x84U;
    if (!sendFrame(response_cmd, absolute_pair_index, packed_values)) return;
    calibration_dump_pair_index_++;
}

uint8_t UsbCdcProtocolBridge::crc8(const uint8_t* data, uint16_t len)
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }
    return crc;
}
