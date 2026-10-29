#include "dfu_bootloader.h"

#include "at32f403a_407.h"

namespace dfu_bootloader
{
    // AT32F403A 的 System Memory（片内 DFU bootloader）常见起始地址。
    // 不同手册/封装给的是 0x1FFFC000（16KB）或 0x1FFFE000（8KB），这里都试一遍。
    // 三个候选指向不同的起始地址，只有真正装了 bootloader 的那个首字（初始 SP）
    // 才会落在 SRAM 范围内，其余读到的是 0xFFFFFFFF。
    const uint32_t kCandidateBases[kCandidateBaseCount] = {
        0x1FFFC000U,
        0x1FFFE000U,
        0x1FFFF000U
    };

    // AT32F403A 的 SRAM 上限。初始 SP 必须落在这个区间里，才认为该地址上是
    // 一个合法的向量表（而不是空白 Flash）。
    constexpr uint32_t kSramBase = 0x20000000U;
    constexpr uint32_t kSramEnd = 0x20020000U;  // 128KB，留足余量

    uint32_t probeInitialStackPointer(uint32_t base)
    {
        if (base < kSramBase || base >= 0x20000000U + 0x1000000U)
        {
            // 只允许读 System Memory 区，避免误读其他外设空间产生总线错误
            if (base < 0x1FF00000U || base >= 0x20000000U)
            {
                return 0U;
            }
        }
        const uint32_t sp = *reinterpret_cast<volatile const uint32_t*>(base);
        if (sp < kSramBase || sp >= kSramEnd)
        {
            return 0U;
        }
        return sp;
    }

    uint32_t findValidBase()
    {
        for (uint32_t i = 0U; i < kCandidateBaseCount; ++i)
        {
            if (probeInitialStackPointer(kCandidateBases[i]) != 0U)
            {
                return kCandidateBases[i];
            }
        }
        return 0U;
    }

    uint32_t enter()
    {
        const uint32_t base = findValidBase();
        if (base == 0U)
        {
            return 0U;
        }

        const uint32_t sp = *reinterpret_cast<volatile const uint32_t*>(base);
        const uint32_t pc = *reinterpret_cast<volatile const uint32_t*>(base + 4U);

        // 关 SysTick，否则跳过去之后还会按应用的节拍打断 bootloader
        SysTick->CTRL = 0U;
        SysTick->LOAD = 0U;
        SysTick->VAL = 0U;

        // 关掉所有 NVIC 中断并清挂起位
        for (uint32_t i = 0U; i < 8U; ++i)
        {
            NVIC->ICER[i] = 0xFFFFFFFFU;
            NVIC->ICPR[i] = 0xFFFFFFFFU;
        }

        // USB 外设必须关掉：bootloader 要自己重新初始化 USB，两边同时挂总线会枚举失败
        crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, FALSE);

        __disable_irq();

        __set_MSP(sp);
        reinterpret_cast<void (*)(void)>(pc)();

        // 正常不会走到这里（bootloader 不会返回）
        for (;;)
        {
        }
    }
}
