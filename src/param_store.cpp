#include "param_store.h"

#include "flash.h"

#include <cstddef>
#include <cstring>

// 参数扇区：0x0801E000（2KB），夹在滞后角数据 0x0801D000 与
// 校准表元数据 0x0801F000 之间，不与任何现有数据重叠。
static const uint32_t kParamFlashBase = 0x0801E000U;
static const uint32_t kParamMagic = 0x54414F54U;   // 'TAOT'
static const uint16_t kParamVersionV1 = 1U;
static const uint16_t kParamVersionV2 = 2U;
static const uint16_t kParamVersionV3 = 3U;
static const uint16_t kParamVersionV4 = 4U;
static const uint16_t kParamVersionV5 = 5U;
static const uint16_t kParamVersionV6 = 6U;
static const uint16_t kParamVersion = 7U;

// v6 布局（v7 追加 encoder_invert 之前的最后形态）。**只为迁移读旧记录用**，
// 校验和必须按该版本自己的结构大小算（原因同 V5/V4）。
struct PersistedMotorParamsV6
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t step_pulse_ns;
    uint16_t angle_max_rpm;
    uint16_t angle_accel_rpm_s;
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint32_t angle_position_kp;
    uint32_t angle_position_ki;
    uint32_t angle_position_kd;
    uint32_t angle_velocity_kp;
    uint32_t angle_velocity_ki;
    uint32_t angle_velocity_kd;
    uint16_t en_mode;
    uint16_t lag_accel_coef_mdeg_per_rpm_s;
    uint16_t lag_deadband_mdeg;
    uint16_t motion_source;
    uint16_t resonance_lo_rpm;
    uint16_t resonance_hi_rpm;
    uint16_t stall_quiet_ms;
    uint16_t stall_move_lsb;
};

// v5 布局（v6 追加字段之前的最后形态）。**只为迁移读旧记录用**，
// 原因同 V4：校验和必须按该版本自己的结构大小算。
struct PersistedMotorParamsV5
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t step_pulse_ns;
    uint16_t angle_max_rpm;
    uint16_t angle_accel_rpm_s;
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint32_t angle_position_kp;
    uint32_t angle_position_ki;
    uint32_t angle_position_kd;
    uint32_t angle_velocity_kp;
    uint32_t angle_velocity_ki;
    uint32_t angle_velocity_kd;
    uint16_t en_mode;
    uint16_t lag_accel_coef_mdeg_per_rpm_s;
    uint16_t lag_deadband_mdeg;
    uint16_t motion_source;
    uint16_t resonance_lo_rpm;
    uint16_t resonance_hi_rpm;
};

// v4 布局（v5 追加字段之前的最后形态）。**只为迁移读旧记录用**。
// 【为什么必须单独定义】校验和按“该版本自己的结构大小”算：v5 在末尾追加了 6 字节，
// 拿 sizeof(v5) 校验一条 v4 记录会多读 6 个 0xFFFF，校验必失败，表现为
// “升级固件后运动源/共振带之外的所有参数莫名回到编译期默认值”。
struct PersistedMotorParamsV4
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t step_pulse_ns;
    uint16_t angle_max_rpm;
    uint16_t angle_accel_rpm_s;
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint32_t angle_position_kp;
    uint32_t angle_position_ki;
    uint32_t angle_position_kd;
    uint32_t angle_velocity_kp;
    uint32_t angle_velocity_ki;
    uint32_t angle_velocity_kd;
    uint16_t en_mode;
    uint16_t lag_accel_coef_mdeg_per_rpm_s;
    uint16_t lag_deadband_mdeg;
};

// v1 布局：只是为了还能读出旧记录做迁移用。**永远不要拿它写 Flash**。
// v2 只是在末尾追加了 6 个角度模式增益，所以 v1 的字段偏移完全不变。
struct PersistedMotorParamsV1
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t step_pulse_ns;
    uint16_t angle_max_rpm;
    uint16_t angle_accel_rpm_s;
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint16_t reserved[7];
};

// v3 布局（v2 与它同尺寸：v2 的最后 2 字节是 reserved[1]，v3 把它用作 en_mode）。
// 【为什么必须单独定义】校验和是按“该版本自己的结构大小”算的。v4 在末尾追加了
// 4 字节，如果拿 sizeof(v4) 去校验一条 v3 记录，会多读 4 个 0xFFFF —— 校验必然
// 失败，表现为“升级固件后参数莫名其妙丢了、回到编译期默认值”。
struct PersistedMotorParamsV3
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t step_pulse_ns;
    uint16_t angle_max_rpm;
    uint16_t angle_accel_rpm_s;
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint32_t angle_position_kp;
    uint32_t angle_position_ki;
    uint32_t angle_position_kd;
    uint32_t angle_velocity_kp;
    uint32_t angle_velocity_ki;
    uint32_t angle_velocity_kd;
    uint16_t en_mode;
};

namespace
{
    // 从 checksum 字段之后开始累加，避免把自己算进去。
    // 必须按“该版本自己的结构大小”累加：v2 多出的字段在旧记录里是擦除后的 0xFFFF，
    // 用 v2 的字节数去校验一条 v1 记录必然算出不匹配的校验和（表现是“参数莫名丢失”）。
    uint16_t checksumOf(const uint8_t* bytes, size_t total, size_t skip)
    {
        uint16_t sum = 0U;
        for (size_t i = skip; i < total; ++i)
        {
            sum = static_cast<uint16_t>(sum + bytes[i]);
        }
        return sum;
    }

    uint16_t computeChecksum(const PersistedMotorParams& p)
    {
        return checksumOf(reinterpret_cast<const uint8_t*>(&p),
                          sizeof(PersistedMotorParams),
                          offsetof(PersistedMotorParams, checksum) + sizeof(p.checksum));
    }

    uint16_t computeChecksumV1(const PersistedMotorParamsV1& p)
    {
        return checksumOf(reinterpret_cast<const uint8_t*>(&p),
                          sizeof(PersistedMotorParamsV1),
                          offsetof(PersistedMotorParamsV1, checksum) + sizeof(p.checksum));
    }

    // 角度模式（串级）的推荐起步值，与 closed_loop_controller.h 里的 NSDMI 一致。
    // 迁移时**不能**照抄规划运动那组：同一个数字在两条路径里含义完全不同。
    void fillAngleDefaults(PersistedMotorParams* out)
    {
        out->angle_position_kp = 20000U;   // 20
        out->angle_position_ki = 0U;
        out->angle_position_kd = 0U;
        out->angle_velocity_kp = 200U;     // 0.2
        out->angle_velocity_ki = 0U;
        out->angle_velocity_kd = 0U;
    }

    // v4 的滞后角模型默认值。
    // 【实测后把加减速项默认关掉（0）】它在正向确实能把末端过冲从 0.94° 压到 0.5°
    // 以下，但在反向会把"欠程"放大（实测反向 -0.9° -> -1.6°），说明正反向的真实
    // 滞后角并不对称（重力/预载），一个纯线性的 c·a 项没有好处。留着寄存器，
    // 需要时在网页上打开并单独标定。
    // 死区 400 毫度是"末端过冲 vs 跟踪刚度"的折中：150 时 1 圈末端 +0.65°，
    // 400 时 +0.45°；再大就会让末端精度掉到 1° 以上。
    constexpr uint16_t kDefaultLagAccelCoef = 0U;
    constexpr uint16_t kDefaultLagDeadband = 400U;

    void fillLagModelDefaults(PersistedMotorParams* out)
    {
        out->lag_accel_coef_mdeg_per_rpm_s = kDefaultLagAccelCoef;
        out->lag_deadband_mdeg = kDefaultLagDeadband;
    }

    // v3 的 PID 起手值（与实际增益一致，×1000 存储）。
    // 【为什么迁移要直接重置】v3 把速度环的口径从“×0.01 的隐藏倍率”改成真实增益：
    // 旧记录里的 30 在新口径下是 30（设计值 0.3），大 100 倍；位置环的输入也从
    // “度×0.01”改成了“圈”，旧数值同样不再对应任何东西。把旧值按比例换算只会
    // 把“用户当时没调好的数”一起搬过来，还会让人以为它有意义。
    // 所以：显式重置为安全默认值，并打印一行提示 —— 反正接下来就要重新整定。
    void fillPidDefaultsV3(PersistedMotorParams* out)
    {
        out->position_kp = 60000U;   // 60
        out->position_ki = 10000U;   // 10
        out->position_kd = 0U;       // 0
        out->velocity_kp = 300U;     // 0.3
        out->velocity_ki = 0U;       // 0
        out->velocity_kd = 0U;       // 0
    }

    // v5 默认：运动源=外部脉冲跟随（用户的需求形态：PLC 脉冲直连），
    // 共振带 300~600 RPM（实测剧烈抖动区，用户可在网页调，写 hi=0 关闭）。
    void fillMotionDefaults(PersistedMotorParams* out)
    {
        out->motion_source = 1U;
        out->resonance_lo_rpm = 300U;
        out->resonance_hi_rpm = 600U;
    }

    // v6 默认：堵转阈值 = 编译期口径（200ms / 100LSB），与引入可调之前的行为一致。
    void fillStallDefaults(PersistedMotorParams* out)
    {
        out->stall_quiet_ms = 200U;
        out->stall_move_lsb = 100U;
    }
}

bool param_store_load(PersistedMotorParams* out)
{
    if (out == nullptr)
    {
        return false;
    }

    // 直接读 Flash 映射地址（与 angle_encoder 的 m_nvmFlashMeta 同样做法），
    // 避免额外拷贝一整个扇区。
    const PersistedMotorParams* flash_params =
        reinterpret_cast<const PersistedMotorParams*>(kParamFlashBase);
    const PersistedMotorParamsV1* flash_params_v1 =
        reinterpret_cast<const PersistedMotorParamsV1*>(kParamFlashBase);

    if (flash_params->magic != kParamMagic)
    {
        return false;
    }

    // v7 新增字段先给默认值：所有比 v7 老的迁移分支 memcpy 的都是更短的结构，
    // 不显式清零就会把调用方的残留字节当成极性。0=正装是安全默认
    // （校准一跑，探针会自动改判）。
    out->encoder_invert = 0U;

    if (flash_params->version == kParamVersion)
    {
        if (flash_params->checksum != computeChecksum(*flash_params))
        {
            return false;
        }
        std::memcpy(out, flash_params, sizeof(PersistedMotorParams));
        return true;
    }

    // v2 / v3 -> v5：前面所有字段偏移一致，按 V3 的长度校验并整段搬过来，
    // 再补上 v4/v5 新增的字段（旧记录里没有，用默认值）。
    if (flash_params->version == kParamVersionV3 || flash_params->version == kParamVersionV2)
    {
        const PersistedMotorParamsV3* v3 =
            reinterpret_cast<const PersistedMotorParamsV3*>(kParamFlashBase);
        const uint16_t v3_checksum = checksumOf(
            reinterpret_cast<const uint8_t*>(v3), sizeof(PersistedMotorParamsV3),
            offsetof(PersistedMotorParamsV3, checksum) + sizeof(v3->checksum));
        if (flash_params->checksum != v3_checksum)
        {
            return false;
        }
        std::memcpy(out, v3, sizeof(PersistedMotorParamsV3));
        out->version = kParamVersion;
        fillLagModelDefaults(out);
        fillMotionDefaults(out);
        fillStallDefaults(out);
        printf("[PARAM] migrated v%u -> v7: kept PID/en_mode, lag-model + motion + stall defaults\r\n",
               static_cast<unsigned>(flash_params->version));
        return true;
    }

    // v4 -> v5：偏移一致（v5 只在末尾追加 3 个 uint16），按 V4 长度校验后整段搬，
    // 再填运动源/共振带默认值。
    if (flash_params->version == kParamVersionV4)
    {
        const PersistedMotorParamsV4* v4 =
            reinterpret_cast<const PersistedMotorParamsV4*>(kParamFlashBase);
        const uint16_t v4_checksum = checksumOf(
            reinterpret_cast<const uint8_t*>(v4), sizeof(PersistedMotorParamsV4),
            offsetof(PersistedMotorParamsV4, checksum) + sizeof(v4->checksum));
        if (flash_params->checksum != v4_checksum)
        {
            return false;
        }
        std::memcpy(out, v4, sizeof(PersistedMotorParamsV4));
        out->version = kParamVersion;
        fillMotionDefaults(out);
        fillStallDefaults(out);
        printf("[PARAM] migrated v4 -> v7: kept all, motion_source=%u resonance=[%u,%u]\r\n",
               static_cast<unsigned>(out->motion_source),
               static_cast<unsigned>(out->resonance_lo_rpm),
               static_cast<unsigned>(out->resonance_hi_rpm));
        return true;
    }

    // v5 -> v6：偏移完全一致（v6 只在末尾追加 2 个 uint16 堵转阈值），
    // 按 V5 长度校验后整段搬，只补堵转阈值默认值。
    if (flash_params->version == kParamVersionV5)
    {
        const PersistedMotorParamsV5* v5 =
            reinterpret_cast<const PersistedMotorParamsV5*>(kParamFlashBase);
        const uint16_t v5_checksum = checksumOf(
            reinterpret_cast<const uint8_t*>(v5), sizeof(PersistedMotorParamsV5),
            offsetof(PersistedMotorParamsV5, checksum) + sizeof(v5->checksum));
        if (flash_params->checksum != v5_checksum)
        {
            return false;
        }
        std::memcpy(out, v5, sizeof(PersistedMotorParamsV5));
        out->version = kParamVersion;
        fillStallDefaults(out);
        printf("[PARAM] migrated v5 -> v7: kept all, stall defaults %u ms / %u LSB\r\n",
               static_cast<unsigned>(out->stall_quiet_ms),
               static_cast<unsigned>(out->stall_move_lsb));
        return true;
    }

    // v6 -> v7：偏移完全一致（v7 只在末尾追加 1 个 uint16 极性标志），
    // 按 V6 长度校验后整段搬；encoder_invert 保持 0（magic 检查后统一预置），
    // 下次校准的极性探针会重新鉴别并把它写成实际值。
    if (flash_params->version == kParamVersionV6)
    {
        const PersistedMotorParamsV6* v6 =
            reinterpret_cast<const PersistedMotorParamsV6*>(kParamFlashBase);
        const uint16_t v6_checksum = checksumOf(
            reinterpret_cast<const uint8_t*>(v6), sizeof(PersistedMotorParamsV6),
            offsetof(PersistedMotorParamsV6, checksum) + sizeof(v6->checksum));
        if (flash_params->checksum != v6_checksum)
        {
            return false;
        }
        std::memcpy(out, v6, sizeof(PersistedMotorParamsV6));
        out->version = kParamVersion;
        printf("[PARAM] migrated v6 -> v7: kept all, encoder_invert=0 (recalibrate to re-detect)\r\n");
        return true;
    }

    // v1 -> v5
    if (flash_params->version == kParamVersionV1)
    {
        if (flash_params_v1->checksum != computeChecksumV1(*flash_params_v1))
        {
            return false;
        }

        out->version = kParamVersion;
        out->microsteps = flash_params_v1->microsteps;
        out->run_current_ma = flash_params_v1->run_current_ma;
        out->hold_current_ma = flash_params_v1->hold_current_ma;
        out->step_pulse_ns = flash_params_v1->step_pulse_ns;
        out->angle_max_rpm = 0U;
        out->angle_accel_rpm_s = 0U;
        out->en_mode = 0U;
        fillPidDefaultsV3(out);
        fillAngleDefaults(out);
        fillLagModelDefaults(out);
        fillMotionDefaults(out);
        fillStallDefaults(out);
        printf("[PARAM] migrated v1 -> v7: PID reset to new scale, defaults for lag/motion/stall\r\n");
        return true;
    }

    return false;
}

bool param_store_save(const PersistedMotorParams* in)
{
    if (in == nullptr)
    {
        return false;
    }

    PersistedMotorParams staging = *in;
    staging.magic = kParamMagic;
    staging.version = kParamVersion;
    staging.checksum = computeChecksum(staging);

    const uint16_t* words = reinterpret_cast<const uint16_t*>(&staging);
    const uint16_t word_count = static_cast<uint16_t>(sizeof(PersistedMotorParams) / 2U);
    if (flash_write(kParamFlashBase, const_cast<uint16_t*>(words), word_count) != SUCCESS)
    {
        return false;
    }

    // 回读校验：Flash 写入失败（比如没擦除）时不会报错，必须读回来重算校验
    const PersistedMotorParams* verify =
        reinterpret_cast<const PersistedMotorParams*>(kParamFlashBase);
    return verify->magic == kParamMagic && verify->checksum == computeChecksum(*verify);
}
