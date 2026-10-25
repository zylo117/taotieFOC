#include "closed_loop_protocol.h"

#include <stdio.h>

#include "at32f403a_407.h"

extern "C"
{
#include "flash.h"
}

namespace
{
    constexpr uint32_t kMotorConfigFlashAddress = 0x0801E800U;
    constexpr uint16_t kMotorConfigMagic0 = 0x544DU;
    constexpr uint16_t kMotorConfigMagic1 = 0x4332U;
    constexpr uint16_t kMotorConfigVersion = 1U;
    constexpr uint16_t kMotorConfigWordCount = 11U;
    /* 写入 SAVE_MOTOR_CONFIG 这个值 = 清掉 Flash 里那条电机参数记录（写全 0 → magic 不匹配 → 下次上电回落安全默认值） */
    constexpr uint32_t kMotorConfigEraseMagic = 0x5AA5U;

    uint16_t clamp_microsteps(uint16_t value)
    {
        static const uint16_t supported[] = {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U};
        if (value < 2U) return 32U;
        uint16_t nearest = supported[0];
        uint16_t delta = value > nearest ? value - nearest : nearest - value;
        for (uint8_t i = 1U; i < sizeof(supported) / sizeof(supported[0]); ++i)
        {
            const uint16_t candidate_delta = value > supported[i] ? value - supported[i] : supported[i] - value;
            if (candidate_delta < delta)
            {
                nearest = supported[i];
                delta = candidate_delta;
            }
        }
        return nearest;
    }

    /* 与驱动里的 kMaxCurrentA 保持一致：Rsense 110mΩ + 内阻 20mΩ，
     * CS=31 + VSENSE=0 时芯片实际能输出 1.768A——上限给到 2.0A 等于“允许烧”，不是“限幅”。
     * 要放开必须两个文件一起改，并确认电机额定电流和散热。 */
    constexpr float kMaxCurrentA = 1.2f;

    float clamp_current_a(float value)
    {
        if (value < 0.0f)
        {
            return 0.0f;
        }
        if (value > kMaxCurrentA)
        {
            return kMaxCurrentA;
        }
        return value;
    }

    /* TMC2209 电流换算：Irms = (CS+1)/32 * Vfs/(Rsense+20mΩ) / sqrt2
     * 采样电阻 110mΩ，芯片内部上下桥按 20mΩ 计入；Vfs 由 CHOPCONF.VSENSE 决定。 */
    constexpr float kSenseInternalOhm = 0.020f;
    constexpr float kVfsLow = 0.180f;      /* VSENSE = 1 */
    constexpr float kVfsHigh = 0.325f;     /* VSENSE = 0 */
    constexpr float kSqrt2 = 1.41421356f;
    constexpr uint32_t kIholdDelay = 8U;   /* 停转后约 0.5s 降到保持电流 */

    float senseTotalOhm(float rsense_ohm)
    {
        return (rsense_ohm > 0.0f ? rsense_ohm : 0.110f) + kSenseInternalOhm;
    }

    /* 和驱动 applyCurrentConfig 的选档规则保持一致：小电流优先 VSENSE = 1 */
    float selectVfs(float rsense_total, float run_current_a)
    {
        const float max_current_low_vfs = kVfsLow / (rsense_total * kSqrt2);
        return run_current_a <= max_current_low_vfs * 0.98f ? kVfsLow : kVfsHigh;
    }

    float ampsFromCs(uint32_t cs, float rsense_total, float vfs)
    {
        return (static_cast<float>(cs) + 1.0f) / 32.0f * vfs / (rsense_total * kSqrt2);
    }

    uint32_t csFromAmps(float amps, float rsense_total, float vfs)
    {
        if (amps <= 0.0f || rsense_total <= 0.0f || vfs <= 0.0f)
        {
            return 0U;
        }
        const float scale = amps * kSqrt2 * rsense_total * 32.0f / vfs;
        int32_t cs = static_cast<int32_t>(scale + 0.5f) - 1;
        if (cs < 0) cs = 0;
        if (cs > 31) cs = 31;
        return static_cast<uint32_t>(cs);
    }

    uint16_t mres_to_microsteps(uint32_t chopconf)
    {
        const uint8_t mres = static_cast<uint8_t>((chopconf >> 24U) & 0x0FU);
        return mres <= 8U ? static_cast<uint16_t>(256U >> mres) : 0U;
    }
}

Tmc2209ProtocolAdapter::Tmc2209ProtocolAdapter()
        : config_{32U, 256U, 1.8f, 0.8f, 0.2f, true, true, 0.110f, true, TMC2209_UART_GPIO, TMC2209_UART_PIN, TMC2209_UART_BAUDRATE},
            custom_parameters_{0U}, driver_(nullptr), motor_config_save_status_(0U)
{
}

bool Tmc2209ProtocolAdapter::isOfficialTmcRegister(uint8_t reg)
{
    switch (reg)
    {
    case TMC2209_REG_GCONF:
    case TMC2209_REG_GSTAT:
    case TMC2209_REG_IOIN:
    case TMC2209_REG_DRV_CONF:
    case TMC2209_REG_IHOLD_IRUN:
    case TMC2209_REG_TPOWERDOWN:
    case TMC2209_REG_TSTEP:
    case TMC2209_REG_TPWMTHRS:
    case TMC2209_REG_TCOOLTHRS:
    case TMC2209_REG_THIGH:
    case TMC2209_REG_CHOPCONF:
    case TMC2209_REG_COOLCONF:
    case TMC2209_REG_DCCTRL:
    case TMC2209_REG_DRV_STATUS:
    case TMC2209_REG_PWMCONF:
    case TMC2209_REG_PWMSCALE:
    case TMC2209_REG_ENCM_CTRL:
    case TMC2209_REG_MSCNT:
    case TMC2209_REG_MSCURACT:
        return true;
    default:
        return false;
    }
}

bool Tmc2209ProtocolAdapter::isCustomExtensionRegister(uint16_t id)
{
    switch (id)
    {
    case TMC2209_EXT_PARAM_MICROSTEPS:
    case TMC2209_EXT_PARAM_INTERPOLATION:
    case TMC2209_EXT_PARAM_RUN_CURRENT_A:
    case TMC2209_EXT_PARAM_HOLD_CURRENT_A:
    case TMC2209_EXT_PARAM_HOLD_ENABLED:
    case TMC2209_EXT_PARAM_SILENT_MODE:
    case TMC2209_EXT_PARAM_SENSE_RESISTOR:
    case TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE:
    case TMC2209_EXT_PARAM_FAULT_STATUS:
    case TMC2209_EXT_PARAM_ENCODER_FAULT:
    case TMC2209_EXT_PARAM_MAGNETIC_FAULT:
    case TMC2209_EXT_PARAM_OUTPUT_STOP:
    case TMC2209_EXT_PARAM_AB_CURRENT_A:
    case TMC2209_EXT_PARAM_AB_CURRENT_B:
    case TMC2209_EXT_PARAM_POS_LOOP_HZ:
    case TMC2209_EXT_PARAM_VEL_LOOP_HZ:
    case TMC2209_EXT_PARAM_CUR_LOOP_HZ:
    case TMC2209_EXT_PARAM_LAST_FAULT:
    case TMC2209_EXT_PARAM_TARGET_POSITION:
    case TMC2209_EXT_PARAM_ACTUAL_POSITION:
    case TMC2209_EXT_PARAM_FOLLOW_ERROR:
    case TMC2209_EXT_PARAM_ENCODER_RAW:
    case TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG:
    case TMC2209_EXT_PARAM_MAGNETIC_HIGH:
    case TMC2209_EXT_PARAM_MAGNETIC_LOW:
    case TMC2209_EXT_PARAM_ENCODER_ZERO:
    case TMC2209_EXT_PARAM_START_RPM:
    case TMC2209_EXT_PARAM_MAX_RPM:
    case TMC2209_EXT_PARAM_ACCEL_RPM_S:
    case TMC2209_EXT_PARAM_PULSE_COUNT:
    case TMC2209_EXT_PARAM_MOTION_MODE:
    case TMC2209_EXT_PARAM_MOTION_COMMAND:
    case TMC2209_EXT_PARAM_SPEED_RPM:
    case TMC2209_EXT_PARAM_POSITION_DEG:
    case TMC2209_EXT_PARAM_MOTOR_ENABLE:
    case TMC2209_EXT_PARAM_MOTOR_DISABLE:
    case TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS:
    case TMC2209_EXT_PARAM_TARGET_ANGLE_DEG:
    case TMC2209_EXT_PARAM_POSITION_KP:
    case TMC2209_EXT_PARAM_POSITION_KI:
    case TMC2209_EXT_PARAM_POSITION_KD:
    case TMC2209_EXT_PARAM_VELOCITY_KP:
    case TMC2209_EXT_PARAM_VELOCITY_KI:
    case TMC2209_EXT_PARAM_VELOCITY_KD:
    case TMC2209_EXT_PARAM_ANGLE_MAX_RPM:
    case TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S:
    case TMC2209_EXT_PARAM_PID_TUNE_CONTROL:
    case TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG:
    case TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT:
    case TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS:
    case TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM:
    case TMC2209_EXT_PARAM_CALIBRATE_ENCODER:
    case TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT:
    case TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT:
    case TMC2209_EXT_PARAM_CALIBRATION_TABLE_DUMP:
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED:
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_UPLOAD_COUNT:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_INDEX:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_SPEED_RPM:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_MDEG:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COMMIT:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_FIT:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_SPEED_RPM:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_QUERY_MDEG:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_POINT_COUNT:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_SLOPE:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_INTERCEPT:
    case TMC2209_EXT_PARAM_LOAD_ANGLE_FIT_EXPONENT:
    case TMC2209_EXT_PARAM_MOTOR_STEP_ANGLE_MDEG:
    case TMC2209_EXT_PARAM_SAVE_MOTOR_CONFIG:
        return true;
    default:
        return false;
    }
}

uint32_t Tmc2209ProtocolAdapter::buildRequestWord(uint8_t reg, uint32_t value)
{
    // Klipper/TMC UART-style requests are conceptually addressed by register number,
    // with the value packed into the low 16 bits of the word and the register in the
    // upper byte. This keeps the host-facing API consistent with how the MCU requests
    // parameter writes and reads while leaving the real driver backend unaware of the
    // protocol format.
    return (static_cast<uint32_t>(reg) << 16U) | (value & 0xFFFFUL);
}

uint32_t Tmc2209ProtocolAdapter::decodeResponseWord(uint8_t reg, uint32_t value)
{
    (void)reg;
    return value & 0xFFFFUL;
}

bool Tmc2209ProtocolAdapter::applyOfficialRegister(uint8_t reg, uint32_t value)
{
    if (driver_ == nullptr)
    {
        return false;
    }

    switch (reg)
    {
    case TMC2209_REG_IHOLD_IRUN:
    {
        /* 上位机给的是 IHOLD_IRUN 原始值：bit4:0 = IHOLD，bit12:8 = IRUN，bit19:16 = IHOLDDELAY。
         * 先按芯片当前的 VSENSE 把 CS 档位反算成安培，再交给驱动去写
         * （驱动内部会重新选 VSENSE 并落到最接近的 CS 档位）。 */
        uint32_t chopconf = 0U;
        if (!driver_->readRegister(TMC2209_REG_CHOPCONF, &chopconf))
        {
            chopconf = 1UL << 17U;    /* 读不到就按 VSENSE = 1 估 */
        }
        const float rsense_total = senseTotalOhm(config_.sense_resistor_ohm);
        const float vfs = (chopconf & (1UL << 17U)) != 0U ? kVfsLow : kVfsHigh;

        const uint32_t ihold_cs = value & 0x1FU;
        const uint32_t irun_cs = (value >> 8U) & 0x1FU;

        config_.run_current_a = clamp_current_a(ampsFromCs(irun_cs, rsense_total, vfs));
        config_.hold_current_a = clamp_current_a(ampsFromCs(ihold_cs, rsense_total, vfs));
        config_.enable_hold_current = ihold_cs != 0U;    /* 0x10 没有使能位，IHOLD = 0 即最小保持 */
        driver_->setRunCurrent(config_.run_current_a);
        driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
        return true;
    }

    case TMC2209_REG_CHOPCONF:
        config_.microsteps = clamp_microsteps(static_cast<uint16_t>(value & 0xFFU));
        driver_->setMicrosteps(config_.microsteps);
        return true;

    case TMC2209_REG_DRV_CONF:
        config_.silent_mode = (value & 0x01U) != 0U;
        driver_->setSilentMode(config_.silent_mode);
        return true;

    case TMC2209_REG_GCONF:
        config_.use_single_wire_uart = (value & 0x04U) != 0U;
        config_.uart_baudrate = TMC2209_UART_BAUDRATE;
        driver_->setSingleWireUart(config_.uart_port, config_.uart_pin, config_.use_single_wire_uart);
        return true;

    case TMC2209_REG_GSTAT:
        return true;

    default:
        return true;
    }
}

bool Tmc2209ProtocolAdapter::readOfficialRegister(uint8_t reg, uint32_t* value) const
{
    if (value == nullptr)
    {
        return false;
    }

    uint32_t raw_value = 0U;

    switch (reg)
    {
    case TMC2209_REG_IHOLD_IRUN:
    {
        /* 安培 → CS 档位，格式与芯片一致：bit4:0 IHOLD，bit12:8 IRUN，bit19:16 IHOLDDELAY。
         * 芯片没存 IHOLD 使能位，enable_hold_current = false 时按 IHOLD = 0（最小档）上报。 */
        const float rsense_total = senseTotalOhm(config_.sense_resistor_ohm);
        const float vfs = selectVfs(rsense_total, config_.run_current_a);
        const uint32_t ihold = config_.enable_hold_current
            ? csFromAmps(config_.hold_current_a, rsense_total, vfs)
            : 0U;
        const uint32_t irun = csFromAmps(config_.run_current_a, rsense_total, vfs);
        raw_value = ihold | (irun << 8U) | (kIholdDelay << 16U);
        break;
    }

    case TMC2209_REG_CHOPCONF:
        raw_value = static_cast<uint32_t>(config_.microsteps);
        break;

    case TMC2209_REG_DRV_CONF:
        raw_value = config_.silent_mode ? 1U : 0U;
        break;

    case TMC2209_REG_GCONF:
        raw_value = config_.use_single_wire_uart ? 0x04U : 0U;
        break;

    default:
        raw_value = 0U;
        break;
    }

    *value = decodeResponseWord(reg, raw_value);
    return true;
}

bool Tmc2209ProtocolAdapter::init()
{
    if (driver_ != nullptr)
    {
        driver_->init();
        const uint8_t probe_registers[] = {TMC2209_REG_GCONF, TMC2209_REG_CHOPCONF, TMC2209_REG_IHOLD_IRUN};
        bool uart_probe_ok = true;
        for (uint8_t reg : probe_registers)
        {
            uint32_t value = 0U;
            if (driver_->readRegister(reg, &value))
            {
                printf("[TMC_UART] PA4 boot read reg=0x%02X value=0x%08lX(%lu) OK\r\n",
                       reg, static_cast<unsigned long>(value), value);
            }
            else
            {
                uart_probe_ok = false;
                printf("[TMC_UART] PA4 boot read reg=0x%02X FAILED; driver may not support TMC UART\r\n", reg);
            }
        }
        printf("[TMC_UART] PA4 boot probe %s\r\n", uart_probe_ok ? "OK" : "FAILED");
        loadMotorSettings();
        syncToDriver();
    }
    return true;
}

bool Tmc2209ProtocolAdapter::configure(const ClosedLoopDriverProtocolConfig& config)
{
    config_ = config;
    if (config_.sense_resistor_ohm <= 0.0f)
    {
        config_.sense_resistor_ohm = 0.110f;
    }
    if (config_.uart_baudrate == 0U)
    {
        config_.uart_baudrate = TMC2209_UART_BAUDRATE;
    }
    if (config_.microsteps == 0U)
    {
        config_.microsteps = 32U;
    }
    config_.microsteps = clamp_microsteps(config_.microsteps);
    config_.interpolation_microsteps = config_.interpolation_microsteps >= 256U ? 256U : 0U;
    if (config_.step_angle_deg < 0.5f || config_.step_angle_deg > 3.0f)
    {
        config_.step_angle_deg = 1.8f;
    }
    if (config_.uart_port == nullptr)
    {
        config_.uart_port = TMC2209_UART_GPIO;
    }
    if (config_.uart_pin == 0U)
    {
        config_.uart_pin = TMC2209_UART_PIN;
    }
    return true;
}

const ClosedLoopDriverProtocolConfig& Tmc2209ProtocolAdapter::config() const
{
    return config_;
}

bool Tmc2209ProtocolAdapter::writeRegister(uint16_t reg, uint32_t value)
{
    if (isCustomExtensionRegister(reg))
    {
        return setCustomParameter(reg, value);
    }

    if (reg > 0xFFU || !isOfficialTmcRegister(static_cast<uint8_t>(reg)))
    {
        return false;
    }

    return driver_ != nullptr && driver_->writeRegister(static_cast<uint8_t>(reg), value);
}

bool Tmc2209ProtocolAdapter::readRegister(uint16_t reg, uint32_t* value)
{
    if (isCustomExtensionRegister(reg))
    {
        return getCustomParameter(reg, value);
    }

    if (reg > 0xFFU || !isOfficialTmcRegister(static_cast<uint8_t>(reg)))
    {
        return false;
    }

    return driver_ != nullptr && driver_->readRegister(static_cast<uint8_t>(reg), value);
}

bool Tmc2209ProtocolAdapter::setCustomParameter(uint16_t id, uint32_t value)
{
    if (!isCustomExtensionRegister(id))
    {
        return false;
    }
    const uint16_t index = static_cast<uint16_t>(id - 0x100U);
    if (index >= 74U)
    {
        return false;
    }

    bool applied = true;
    switch (id)
    {
    case TMC2209_EXT_PARAM_MICROSTEPS:
        config_.microsteps = clamp_microsteps(static_cast<uint16_t>(value));
        if (driver_ != nullptr) applied = driver_->setMicrosteps(config_.microsteps);
        break;
    case TMC2209_EXT_PARAM_INTERPOLATION:
        config_.interpolation_microsteps = value >= 256U ? 256U : 0U;
        if (driver_ != nullptr) applied = driver_->setInterpolation(config_.microsteps, config_.interpolation_microsteps);
        break;
    case TMC2209_EXT_PARAM_RUN_CURRENT_A:
        config_.run_current_a = clamp_current_a(static_cast<float>(value) / 1000.0f);
        if (driver_ != nullptr) applied = driver_->setRunCurrent(config_.run_current_a);
        break;
    case TMC2209_EXT_PARAM_HOLD_CURRENT_A:
        config_.hold_current_a = clamp_current_a(static_cast<float>(value) / 1000.0f);
        if (driver_ != nullptr) applied = driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
        break;
    case TMC2209_EXT_PARAM_HOLD_ENABLED:
        config_.enable_hold_current = value != 0U;
        if (driver_ != nullptr) applied = driver_->setHoldCurrent(config_.hold_current_a, config_.enable_hold_current);
        break;
    case TMC2209_EXT_PARAM_SILENT_MODE:
        config_.silent_mode = value != 0U;
        if (driver_ != nullptr) applied = driver_->setSilentMode(config_.silent_mode);
        break;
    case TMC2209_EXT_PARAM_SENSE_RESISTOR:
    {
        /* 采样电阻是电流换算的唯一标尺，写错等于把电流放大/缩小几十倍：
         * 传 0 会让所有电流配置直接失效，传 1000（1Ω）会把 CS 顶到 31 去凑目标值。
         * 只接受 50~500mΩ（本板 110mΩ），其余整条丢弃、保持原值。 */
        const float ohm = static_cast<float>(value) / 1000.0f;
        if (ohm < 0.05f || ohm > 0.5f)
        {
            printf("[TMC_SAFE] sense resistor %lumOhm out of range 50~500, ignored\r\n",
                   static_cast<unsigned long>(value));
            return false;
        }
        config_.sense_resistor_ohm = ohm;
        if (driver_ != nullptr) applied = driver_->setSenseResistor(config_.sense_resistor_ohm);
        break;
    }
    case TMC2209_EXT_PARAM_MOTOR_STEP_ANGLE_MDEG:
        config_.step_angle_deg = static_cast<float>(value) / 1000.0f;
        if (config_.step_angle_deg < 0.5f || config_.step_angle_deg > 3.0f) return false;
        if (driver_ != nullptr)
        {
            StepperDriverConfig driver_config = driver_->config();
            driver_config.step_angle_deg = config_.step_angle_deg;
            applied = driver_->configure(driver_config);
        }
        break;
    case TMC2209_EXT_PARAM_SAVE_MOTOR_CONFIG:
        /* 写入 kMotorConfigEraseMagic = 清掉已保存的电机参数（开机不再加载旧值）；
         * 写其它非 0 值 = 保存当前参数。 */
        if (value == kMotorConfigEraseMagic)
        {
            uint16_t blank[kMotorConfigWordCount] = {};
            const bool erased = flash_write(kMotorConfigFlashAddress, blank, kMotorConfigWordCount) == SUCCESS;
            if (erased)
            {
                /* 只擦 Flash 只对“下次上电”生效：本次运行里的 config_ 还是旧值，
                 * 上位机回读也会拿到旧值，看起来就像擦除没起作用。
                 * 所以 RAM 里的 config_ 一并回到内置默认，再尽力同步一次芯片。
                 * 这里必须和 loadMotorSettings() 拒绝现场时用的默认值保持一致。 */
                config_.microsteps = 32U;
                config_.interpolation_microsteps = 256U;
                config_.step_angle_deg = 1.8f;
                config_.run_current_a = 0.8f;
                config_.hold_current_a = 0.2f;
                config_.enable_hold_current = true;
                config_.silent_mode = true;
                config_.sense_resistor_ohm = 0.110f;
                /* UART 已损坏时 configure() 会返回 false，但 config_ 已经是对的：
                 * 上位机回读、下一次保存、以及上电默认值都靠 config_，不靠这次写成功。 */
                syncToDriver();
            }
            motor_config_save_status_ = erased ? 1U : 0U;    /* 与保存一致：1 = 成功，上位机回读判据 */
            printf("[MOTOR_CFG] motor settings %s; defaults now 32/256/1.8deg/800mA/200mA/silent\r\n",
                   erased ? "erased" : "erase FAILED");
            return erased;
        }
        motor_config_save_status_ = value != 0U && saveMotorSettings() ? 1U : 0U;
        return value != 0U && motor_config_save_status_ != 0U;
    case TMC2209_EXT_PARAM_START_RPM:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_MAX_RPM:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_ACCEL_RPM_S:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_PULSE_COUNT:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_MOTION_MODE:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_MOTION_COMMAND:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED:
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW:
    case TMC2209_EXT_PARAM_CALIBRATION_TABLE_DUMP:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS:
        custom_parameters_[index] = value;
        break;
    case TMC2209_EXT_PARAM_MOTOR_ENABLE:
    case TMC2209_EXT_PARAM_MOTOR_DISABLE:
    case TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS:
    case TMC2209_EXT_PARAM_TARGET_ANGLE_DEG:
        custom_parameters_[index] = value;
        break;
    default:
        break;
    }
    custom_parameters_[index] = value;
    return applied;
}

bool Tmc2209ProtocolAdapter::getCustomParameter(uint16_t id, uint32_t* value) const
{
    if (!isCustomExtensionRegister(id) || value == nullptr)
    {
        return false;
    }
    const uint16_t index = static_cast<uint16_t>(id - 0x100U);
    if (index >= 74U)
    {
        return false;
    }
    switch (id)
    {
    case TMC2209_EXT_PARAM_MICROSTEPS:
    case TMC2209_EXT_PARAM_INTERPOLATION:
    {
        uint32_t chopconf = 0U;
        if (driver_ == nullptr || !driver_->readRegister(TMC2209_REG_CHOPCONF, &chopconf))
        {
            /* 读不到就上报本地已知值，绝不把 0xFFFFFFFF 这类无效值交给上位机：
             * 上位机会拿它继续算（例如 getMotorStepsPerRevolution(微步)），天文数字的步数就是能烧驱动器的输入。 */
            printf("[TMC_UART] CHOPCONF read failed for 0x%04X, reporting local config\r\n", id);
            *value = id == TMC2209_EXT_PARAM_MICROSTEPS
                ? static_cast<uint32_t>(config_.microsteps)
                : static_cast<uint32_t>(config_.interpolation_microsteps);
            return true;
        }
        *value = id == TMC2209_EXT_PARAM_MICROSTEPS
            ? mres_to_microsteps(chopconf)
            : ((chopconf & (1UL << 28U)) != 0U ? 256U : 0U);
        /* MRES 读出 9~15（非法档位）时 mres_to_microsteps 返回 0。此时绝不能返回 0 让上层
         * 继续算“每圈步数”，也不能让 usb 层兑成 0xFFFFFFFF，直接报本地配置。 */
        if (id == TMC2209_EXT_PARAM_MICROSTEPS && *value == 0U)
        {
            printf("[TMC_UART] CHOPCONF MRES=%lu invalid, reporting local microsteps=%u\r\n",
                   static_cast<unsigned long>((chopconf >> 24U) & 0x0FU), config_.microsteps);
            *value = static_cast<uint32_t>(config_.microsteps);
        }
        return true;
    }
    case TMC2209_EXT_PARAM_RUN_CURRENT_A:
    case TMC2209_EXT_PARAM_HOLD_CURRENT_A:
    {
        uint32_t chopconf = 0U;
        uint32_t ihold_irun = 0U;
        if (driver_ == nullptr || !driver_->readRegister(TMC2209_REG_CHOPCONF, &chopconf) ||
            !driver_->readRegister(TMC2209_REG_IHOLD_IRUN, &ihold_irun))
        {
            printf("[TMC_UART] current register read failed for 0x%04X, reporting local config\r\n", id);
            const float fallback_amps = id == TMC2209_EXT_PARAM_RUN_CURRENT_A
                ? config_.run_current_a
                : (config_.enable_hold_current ? config_.hold_current_a : 0.0f);
            *value = static_cast<uint32_t>(fallback_amps * 1000.0f + 0.5f);
            return true;
        }
        /* 这块模块回读 IHOLD_IRUN 恒为 0，拿它反算会永远报 ~30mA，
         * 所以只有读到非零值才用寄存器反算，否则回落到本地配置值（写进去的那个）。 */
        if (ihold_irun == 0U)
        {
            const float amps = id == TMC2209_EXT_PARAM_RUN_CURRENT_A
                ? config_.run_current_a
                : (config_.enable_hold_current ? config_.hold_current_a : 0.0f);
            *value = static_cast<uint32_t>(amps * 1000.0f + 0.5f);
            return true;
        }

        const uint32_t cs = id == TMC2209_EXT_PARAM_RUN_CURRENT_A
            ? ((ihold_irun >> 8U) & 0x1FU)
            : (ihold_irun & 0x1FU);
        const float vfs = (chopconf & (1UL << 17U)) != 0U ? 0.180f : 0.325f;
        const float rsense_total = config_.sense_resistor_ohm + 0.020f;
        const float current_ma = (static_cast<float>(cs + 1U) / 32.0f) * vfs /
                                 (rsense_total * 1.41421356f) * 1000.0f;
        *value = static_cast<uint32_t>(current_ma + 0.5f);
        return true;
    }
    case TMC2209_EXT_PARAM_HOLD_ENABLED:
        *value = config_.enable_hold_current ? 1U : 0U;
        break;
    case TMC2209_EXT_PARAM_SILENT_MODE:
    {
        uint32_t gconf = 0U;
        if (driver_ == nullptr || !driver_->readRegister(TMC2209_REG_GCONF, &gconf))
        {
            printf("[TMC_UART] GCONF read failed for silent mode, reporting local config\r\n");
            *value = config_.silent_mode ? 1U : 0U;
            return true;
        }
        *value = (gconf & (1UL << 2U)) == 0U ? 1U : 0U;
        return true;
    }
    case TMC2209_EXT_PARAM_SENSE_RESISTOR:
        *value = static_cast<uint32_t>(config_.sense_resistor_ohm * 1000.0f);
        break;
    case TMC2209_EXT_PARAM_MOTOR_STEP_ANGLE_MDEG:
        *value = static_cast<uint32_t>(config_.step_angle_deg * 1000.0f);
        break;
    case TMC2209_EXT_PARAM_SAVE_MOTOR_CONFIG:
        *value = motor_config_save_status_;
        break;
    case TMC2209_EXT_PARAM_START_RPM:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_MAX_RPM:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_ACCEL_RPM_S:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_PULSE_COUNT:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_MOTION_MODE:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_MOTION_COMMAND:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED:
    case TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW:
    case TMC2209_EXT_PARAM_CALIBRATION_TABLE_DUMP:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS:
        *value = custom_parameters_[index];
        break;
    case TMC2209_EXT_PARAM_MOTOR_ENABLE:
    case TMC2209_EXT_PARAM_MOTOR_DISABLE:
    case TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS:
    case TMC2209_EXT_PARAM_TARGET_ANGLE_DEG:
        *value = custom_parameters_[index];
        break;
    default:
        *value = custom_parameters_[index];
        break;
    }
    return true;
}

void Tmc2209ProtocolAdapter::attachDriver(StepperDriver* driver)
{
    driver_ = driver;
}

void Tmc2209ProtocolAdapter::syncToDriver()
{
    if (driver_ == nullptr)
    {
        return;
    }

    StepperDriverConfig driver_config = driver_->config();
    driver_config.microsteps = config_.microsteps;
    driver_config.interpolation_microsteps = config_.interpolation_microsteps;
    driver_config.step_angle_deg = config_.step_angle_deg;
    driver_config.run_current_a = config_.run_current_a;
    driver_config.hold_current_a = config_.hold_current_a;
    driver_config.enable_hold_current = config_.enable_hold_current;
    driver_config.silent_mode = config_.silent_mode;
    driver_config.sense_resistor_ohm = config_.sense_resistor_ohm;
    driver_config.uart_baudrate = config_.uart_baudrate;
    driver_config.use_single_wire_uart = config_.use_single_wire_uart;
    driver_config.uart_port = config_.uart_port;
    driver_config.uart_pin = config_.uart_pin;
    driver_->configure(driver_config);
}

bool Tmc2209ProtocolAdapter::saveMotorSettings()
{
    uint16_t words[kMotorConfigWordCount] = {};
    words[0] = kMotorConfigMagic0;
    words[1] = kMotorConfigMagic1;
    words[2] = kMotorConfigVersion;
    words[3] = config_.microsteps;
    words[4] = config_.interpolation_microsteps;
    words[5] = static_cast<uint16_t>(config_.step_angle_deg * 1000.0f + 0.5f);
    words[6] = static_cast<uint16_t>(config_.run_current_a * 1000.0f + 0.5f);
    words[7] = static_cast<uint16_t>(config_.hold_current_a * 1000.0f + 0.5f);
    words[8] = static_cast<uint16_t>((config_.enable_hold_current ? 1U : 0U) |
                                    (config_.silent_mode ? 2U : 0U));
    words[9] = static_cast<uint16_t>(config_.sense_resistor_ohm * 1000.0f + 0.5f);
    uint16_t checksum = 0U;
    for (uint16_t i = 0U; i < kMotorConfigWordCount - 1U; ++i) checksum += words[i];
    words[10] = checksum;
    return flash_write(kMotorConfigFlashAddress, words, kMotorConfigWordCount) == SUCCESS;
}

bool Tmc2209ProtocolAdapter::loadMotorSettings()
{
    uint16_t words[kMotorConfigWordCount];
    flash_read(kMotorConfigFlashAddress, words, kMotorConfigWordCount);
    uint16_t checksum = 0U;
    for (uint16_t i = 0U; i < kMotorConfigWordCount - 1U; ++i) checksum += words[i];
    if (words[0] != kMotorConfigMagic0 || words[1] != kMotorConfigMagic1 ||
        words[2] != kMotorConfigVersion || words[10] != checksum ||
        words[3] < 2U || words[3] > 256U ||
        (words[4] != 0U && words[4] != 256U) ||
        words[5] < 500U || words[5] > 3000U ||
        /* 50mA 以下几乎肯定是脏数据：旧版本把 CS 档位误当成「10mA 为单位」，
         * CS=1 会算出 0.010A 并被 saveMotorSettings 写进 Flash，这里拦住并回落默认值。
         * 1200mA 以上同样拒绝：那是超过安全上限的值，宁可不加载也不能上电就跑大电流。 */
        words[6] < 50U || words[6] > 1200U ||
        words[7] > 1200U || words[9] == 0U)
    {
        /* 把拒绝现场的原始字打出来：上电第一句话就能判断是“没存过”还是“存了脏值” */
        printf("[MOTOR_CFG] no valid record at 0x%08lX (magic=%04X/%04X ver=%u msteps=%u interp=%u angle=%u run=%umA hold=%umA flags=%u rsense=%umOhm dash=%u/%u); using built-in defaults\r\n",
               static_cast<unsigned long>(kMotorConfigFlashAddress),
               words[0], words[1], words[2], words[3], words[4], words[5],
               words[6], words[7], words[8], words[9], words[10], checksum);
        motor_config_save_status_ = 0U;
        return false;
    }

    /* 双重保险：即使 Flash 记录通过了上面的范围校验，加载时再过一道电流夹紧，
     * 保证 config_ 里不可能存在会上电就烧驱动的电流值。 */
    config_.microsteps = words[3];
    config_.interpolation_microsteps = words[4];
    config_.step_angle_deg = static_cast<float>(words[5]) / 1000.0f;
    config_.run_current_a = clamp_current_a(static_cast<float>(words[6]) / 1000.0f);
    config_.hold_current_a = clamp_current_a(static_cast<float>(words[7]) / 1000.0f);
    config_.enable_hold_current = (words[8] & 1U) != 0U;
    config_.silent_mode = (words[8] & 2U) != 0U;
    config_.sense_resistor_ohm = static_cast<float>(words[9]) / 1000.0f;
    printf("[MOTOR_CFG] loaded: msteps=%u interp=%u angle=%.2f run=%.0fmA hold=%.0fmA silent=%u\r\n",
           config_.microsteps, config_.interpolation_microsteps, config_.step_angle_deg,
           config_.run_current_a * 1000.0f, config_.hold_current_a * 1000.0f,
           config_.silent_mode ? 1U : 0U);
    motor_config_save_status_ = 1U;
    return true;
}
