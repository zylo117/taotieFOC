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

private:
    static constexpr uint8_t kCurrentAdcPairsPerHalf = 1U;
    static constexpr uint16_t kCurrentAdcDmaBufferLength = kCurrentAdcPairsPerHalf * 4U;
    static constexpr float kCurrentAdcVddaVolts = 3.3f;
    static constexpr float kCurrentAdcReferenceVolts = 1.65f;
    static constexpr float kIna240Gain = 50.0f;
    static constexpr float kCurrentShuntOhms = 0.010f;

    volatile uint16_t dma_buffer_[kCurrentAdcDmaBufferLength];
    float phase_a_current_a_;
    float phase_b_current_a_;
    bool initialized_;

    static float rawToAmps(uint16_t raw);
};

#endif
