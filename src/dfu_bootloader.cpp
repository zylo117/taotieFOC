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

    // BPR（备份电源域寄存器）里存一个 magic。它不受系统复位影响，
    // 而且复位后最早就能读，正好用来当“下一次启动要进 bootloader”的信号。
    constexpr uint32_t kBprBootMagic = 0x44465531UL;  // "DFU1"

    volatile uint32_t g_dfu_step = 0U;
    volatile uint32_t g_dfu_probe_sp[kCandidateBaseCount] = {0U, 0U, 0U};

    void probeAll()
    {
        for (uint32_t i = 0U; i < kCandidateBaseCount; ++i)
        {
            g_dfu_probe_sp[i] = probeInitialStackPointer(kCandidateBases[i]);
        }
    }

    void requestAfterReset()
    {
        // 【必须先使能 PWC 时钟】BPR 属于备份电源域，PWC 时钟没开时
        // pwc_battery_powered_domain_access(TRUE) 里那个等待位永远不生效，
        // 函数会卡在里面 —— 现象就是 usb_task 不动了（ADC 中断还在跑、isr 继续涨），
        // 后面的 NVIC_SystemReset() 根本没机会执行。
        crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
        pwc_battery_powered_domain_access(TRUE);
        BPR->dt1 = kBprBootMagic;
        pwc_battery_powered_domain_access(FALSE);
    }

    bool consumeResetRequest()
    {
        // main() 最开头调，此时 CRM 还是复位默认状态，同样要先开 PWC 时钟
        crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
        pwc_battery_powered_domain_access(TRUE);
        const bool requested = (BPR->dt1 == kBprBootMagic);
        if (requested)
        {
            // 只生效一次，否则刷完机重启又会被拉回 bootloader
            BPR->dt1 = 0U;
        }
        pwc_battery_powered_domain_access(FALSE);
        return requested;
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

        // 【必须把时钟树恢复成复位默认（HSI）】
        // 应用把 PLL 配到了 250MHz。bootloader 是按“刚从复位启动”的前提写的，
        // 它会用 HSI/HEXT 重新算 PLL 和 USB 48MHz；如果任由应用留下的 PLL 继续跑，
        // bootloader 的时钟初始化会基于错误输入计算，USB 枚举失败（现象：跳过去后
        // 既看不到 DFU 设备，也没有任何串口输出）。
        crm_reset();

        __set_MSP(sp);
        // 【关键：切回 MSP + 特权级线程模式】
        // FreeRTOS 的任务跑在“线程模式 + PSP”。只 __set_MSP 是不够的，
        // CPU 仍然拿 PSP 去取栈，一跳进 bootloader 就会用到无效栈 → HardFault。
        // CONTROL=0 同时清掉 SPSEL（用 MSP）和 nPRIV（特权级）。
        __set_CONTROL(0U);
        __ISB();

        reinterpret_cast<void (*)(void)>(pc)();

        // 正常不会走到这里（bootloader 不会返回）
        for (;;)
        {
        }
    }
}
