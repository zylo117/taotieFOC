#include "usb_cdc_protocol.h"

#include <string.h>

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
            calibration_dump_type_(0U), calibration_dump_pair_index_(0U), calibration_dump_pair_count_(0U)
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
    // The upstream Web UI uses a fixed-length 10-byte protocol frame:
    // 0x55 0xAA, cmd, reg_hi, reg_lo, value[4], crc8
    // We accept a payload length from 10 bytes and ignore trailing bytes in the same packet.
    const uint16_t frame_len = 10U;
    for (uint16_t offset = 0U; offset + frame_len <= rx_len; offset += frame_len)
    {
        const uint8_t* frame = recv_buf + offset;
        if (frame[0] != kFrameHeaderA || frame[1] != kFrameHeaderB)
        {
            continue;
        }

        const uint8_t crc = frame[9];
        const uint8_t expected_crc = crc8(frame, 9U);
        if (crc != expected_crc)
        {
            continue;
        }

        handleFrame(frame, frame_len);
    }
    serviceCalibrationTableDump();
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
        calibration_dump_type_ = static_cast<uint8_t>(value);
        calibration_dump_pair_index_ = 0U;
        if (calibration_dump_type_ == 0U)
        {
            calibration_dump_pair_count_ = CALIBRATION_TABLE_SIZE / 2U;
            calibration_dump_active_ = true;
        }
        else if (calibration_dump_type_ == 1U)
        {
            calibration_dump_pair_count_ = 32768U;
            calibration_dump_active_ = true;
        }
        else if (calibration_dump_type_ == 2U)
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

    return usb_vcp_send_data(udev_, frame, sizeof(frame)) == SUCCESS;
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

    if (calibration_dump_pair_index_ >= calibration_dump_pair_count_)
    {
        if (!sendFrame(0x86U, calibration_dump_pair_count_, calibration_dump_type_)) return;
        calibration_dump_active_ = false;
        return;
    }

    uint32_t packed_values = 0U;
    if (!controller_->readCalibrationTablePair(calibration_dump_type_, calibration_dump_pair_index_,
                                               &packed_values))
    {
        if (!sendFrame(0x86U, calibration_dump_pair_index_, 0xFFFFFFFFU)) return;
        calibration_dump_active_ = false;
        return;
    }

    const uint8_t response_cmd = calibration_dump_type_ == 0U ? 0x83U : 0x84U;
    if (!sendFrame(response_cmd, calibration_dump_pair_index_, packed_values)) return;
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
