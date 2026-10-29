#ifndef DFU_BOOTLOADER_H
#define DFU_BOOTLOADER_H

#include <cstdint>

// 从应用直接跳进 AT32 片内 System Memory 里的 USB DFU bootloader。
//
// 为什么要软件跳：AT32F403A 进 DFU 只能靠 BOOT0 引脚，每次刷机都要手动拨开关。
// System Memory 里常驻一个 USB DFU 固件，只要把栈指针和复位向量交过去就能进，
// 不需要任何外部引脚动作。
//
// 安全措施：
//   1. 跳转前先校验候选地址处的初始 SP 是否落在 SRAM 范围内，不合法就当没这回事，
//      直接复位回应用而不是跳进野地址；
//   2. 不同封装/批次的 System Memory 起始地址不完全一致（0x1FFFC000 / 0x1FFFE000），
//      所以按候选表逐个试，第一个 SP 合法的就跳。
namespace dfu_bootloader
{
    // 候选的 System Memory 基址（越大越保守，先试常见的）
    constexpr uint32_t kCandidateBaseCount = 3U;
    extern const uint32_t kCandidateBases[kCandidateBaseCount];

    // 各候选基址处读到的初始 SP（0 表示不可用），调试时可以从串口打出来
    uint32_t probeInitialStackPointer(uint32_t base);

    // 找到可用的 bootloader 基址并跳过去。找到返回基址，一个都没找到返回 0（未跳转）。
    // 调用本函数前不要关中断 —— 它自己做。
    uint32_t enter();

    // 只探测不跳转，给自检打印用
    uint32_t findValidBase();
}

#endif
