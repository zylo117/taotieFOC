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
static const uint16_t kParamVersion = 3U;

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

    if (flash_params->version == kParamVersion)
    {
        if (flash_params->checksum != computeChecksum(*flash_params))
        {
            return false;
        }
        std::memcpy(out, flash_params, sizeof(PersistedMotorParams));
        return true;
    }

    // v1 / v2 -> v3：字段偏移完全一致，先原样搬过来，再把 PID 重置成新口径的默认值。
    if (flash_params->version == kParamVersionV2 || flash_params->version == kParamVersionV1)
    {
        if (flash_params->version == kParamVersionV1 &&
            flash_params_v1->checksum != computeChecksumV1(*flash_params_v1))
        {
            return false;
        }
        if (flash_params->version == kParamVersionV2 &&
            flash_params->checksum != computeChecksum(*flash_params))
        {
            return false;
        }

        out->version = kParamVersion;
        out->microsteps = flash_params->microsteps;
        out->run_current_ma = flash_params->run_current_ma;
        out->hold_current_ma = flash_params->hold_current_ma;
        out->step_pulse_ns = flash_params->step_pulse_ns;
        out->angle_max_rpm = 0U;
        out->angle_accel_rpm_s = 0U;
        fillPidDefaultsV3(out);
        if (flash_params->version == kParamVersionV1)
        {
            // v1 没有角度字段：它们本来就不再用，统一写 0
            fillAngleDefaults(out);
            printf("[PARAM] migrated v1 -> v3: PID reset to new scale (tune again), "
                   "angle fields unused\r\n");
        }
        else
        {
            out->angle_position_kp = 0U;
            out->angle_position_ki = 0U;
            out->angle_position_kd = 0U;
            out->angle_velocity_kp = 0U;
            out->angle_velocity_ki = 0U;
            out->angle_velocity_kd = 0U;
            printf("[PARAM] migrated v2 -> v3: velocity-loop gain scale changed "
                   "(old values were 100x), PID reset to safe defaults -- re-tune, then save\r\n");
        }
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
