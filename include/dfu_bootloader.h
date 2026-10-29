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

    // 【复位后跳转】USB 命令不应该直接跳 bootloader。
    // 应用跑在 250MHz PLL + FreeRTOS 的 PSP 线程模式，bootloader 是按“刚从复位启动”
    // （HSI、MSP、无中断）写的，现场改时钟+换栈再跳实测起不来（跳过去后无任何输出）。
    // 所以改成 AT32 推荐的两步：先写 BPR 标志，再系统复位；
    // main() 最开头 consumeResetRequest() 命中后才真跳——那时环境与 BOOT0 一致。
    void requestAfterReset();
    bool consumeResetRequest();

    // ===== 诊断（先只标记不复位）=====
    // 用状态码把“命令到底有没有进 DFU 分支”变成可观测的：
    //   0 = 从没收到 DFU 命令
    //   1 = handleFrame 的 DFU 分支命中了（收到命令）
    //   2 = 三个候选地址的初始 SP 已读完
    // telemetry 任务每秒把 g_dfu_step 和三个 SP 打成一行 [DFU]。
    // 这一步刻意不调 NVIC_SystemReset()：只要能看到 step=2，就说明分支和探测都正常，
    // 后续失败就只剩“地址不对”或“跳转环境不对”两种原因。
    extern volatile uint32_t g_dfu_step;
    extern volatile uint32_t g_dfu_probe_sp[kCandidateBaseCount];
    void probeAll();
}

#endif
