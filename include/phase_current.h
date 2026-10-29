#ifndef PHASE_CURRENT_H
#define PHASE_CURRENT_H

#include <cstdint>

class PhaseCurrentMonitor
{
public:
    static constexpr uint32_t kDmaNotifyFirstHalf = 1U;
    static constexpr uint32_t kDmaNotifySecondHalf = 2U;

    PhaseCurrentMonitor();

    bool init();
    void handleDmaNotification(uint32_t notification_value);

    float phaseCurrentA() const;
    float phaseCurrentB() const;
    bool isInitialized() const;

    // ===== 自检用（不参与控制）=====
    // isrCount  ：DMA 半满/全满中断次数。不涨说明 ADC 根本没触发，或 DMA 没搬数据。
    // lastRawA/B：最近一次算平均值用的 ADC 原始码。中点约 2048，0 说明 DMA 里全是 0。
    // 这三个量能把“ADC 没转” / “DMA 没搬” / “信号真是 0A” 三者分开。
    uint32_t isrCount() const { return isr_count_; }
    uint16_t lastRawA() const { return last_raw_a_; }
    uint16_t lastRawB() const { return last_raw_b_; }

private:
    // 【每半区的采样对数 = 8】
    // TMR4 每触发一次转换 2 个通道（A/B），对应 2 个半字。
    // 旧值 1 意味着 buffer 只有 4 个半字 = 2 对，半满和全满两个中断
    // 每个控制周期都各触发一次 → 20.83kHz x 2 = 41.7kHz 中断。
    // 而且 main.cpp 用 eSetValueWithOverwrite 通知，任务每次只处理 value
    // 指向的那一半，另一半数据会被下一次覆盖，实际丢掉 50% 样本。
    // 改成 8 后：buffer = 32 半字，中断降到 20.83k/4 = 5.2kHz，
    // 且每次一口气处理 8 对，求和平均更充分。
    static constexpr uint8_t kCurrentAdcPairsPerHalf = 8U;
    static constexpr uint16_t kCurrentAdcDmaBufferLength = kCurrentAdcPairsPerHalf * 4U;
    static constexpr float kCurrentAdcVddaVolts = 3.3f;
    static constexpr float kCurrentAdcReferenceVolts = 1.65f;
    static constexpr float kIna240Gain = 50.0f;
    static constexpr float kCurrentShuntOhms = 0.010f;

    volatile uint16_t dma_buffer_[kCurrentAdcDmaBufferLength];
    float phase_a_current_a_;
    float phase_b_current_a_;
    bool initialized_;
    volatile uint32_t isr_count_ = 0U;
    volatile uint16_t last_raw_a_ = 0U;
    volatile uint16_t last_raw_b_ = 0U;

    static float rawToAmps(uint16_t raw);
};

#endif
