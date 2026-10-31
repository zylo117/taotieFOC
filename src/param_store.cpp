#include "param_store.h"

#include "flash.h"

#include <cstddef>
#include <cstring>

// 参数扇区：0x0801E000（2KB），夹在负载角数据 0x0801D000 与
// 校准表元数据 0x0801F000 之间，不与任何现有数据重叠。
static const uint32_t kParamFlashBase = 0x0801E000U;
static const uint32_t kParamMagic = 0x54414F54U;   // 'TAOT'
static const uint16_t kParamVersion = 1U;

// 从 checksum 字段之后开始累加，避免把自己算进去。
static uint16_t computeChecksum(const PersistedMotorParams& p)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
    const size_t skip = offsetof(PersistedMotorParams, checksum) + sizeof(p.checksum);
    uint16_t sum = 0U;
    for (size_t i = skip; i < sizeof(PersistedMotorParams); ++i)
    {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return sum;
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

    if (flash_params->magic != kParamMagic || flash_params->version != kParamVersion)
    {
        return false;
    }

    const uint16_t expected = computeChecksum(*flash_params);
    if (flash_params->checksum != expected)
    {
        return false;
    }

    std::memcpy(out, flash_params, sizeof(PersistedMotorParams));
    return true;
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
