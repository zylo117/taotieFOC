#include "phase_current.h"

#include "at32f403a_407_board.h"

PhaseCurrentMonitor::PhaseCurrentMonitor()
    : dma_buffer_{0U},
      phase_a_current_a_(0.0f),
      phase_b_current_a_(0.0f),
      initialized_(false)
{
}

float PhaseCurrentMonitor::rawToAmps(uint16_t raw)
{
    const float output_voltage = static_cast<float>(raw) * kCurrentAdcVddaVolts / 4095.0f;
    return (output_voltage - kCurrentAdcReferenceVolts) / (kIna240Gain * kCurrentShuntOhms);
}

bool PhaseCurrentMonitor::init()
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init(GPIOA, &gpio_init_struct);

    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_6);

    adc_base_config_type adc_base_struct;
    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = FALSE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = 2U;
    adc_base_config(ADC1, &adc_base_struct);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1U, ADC_SAMPLETIME_55_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, 2U, ADC_SAMPLETIME_55_5);
    adc_dma_mode_enable(ADC1, TRUE);

    dma_init_type dma_init_struct;
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.buffer_size = kCurrentAdcDmaBufferLength;
    dma_init_struct.peripheral_base_addr = reinterpret_cast<uint32_t>(&ADC1->odt);
    dma_init_struct.memory_base_addr = reinterpret_cast<uint32_t>(const_cast<uint16_t*>(dma_buffer_));
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    dma_flag_clear(DMA1_FDT1_FLAG | DMA1_HDT1_FLAG | DMA1_DTERR1_FLAG);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT | DMA_FDT_INT, TRUE);
    nvic_irq_enable(DMA1_Channel1_IRQn, 2U, 0U);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

    adc_enable(ADC1, TRUE);
    constexpr uint32_t kCalibrationTimeoutIterations = 1000000U;
    adc_calibration_init(ADC1);
    uint32_t timeout = kCalibrationTimeoutIterations;
    while (adc_calibration_init_status_get(ADC1) == SET && timeout > 0U)
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        return false;
    }

    adc_calibration_start(ADC1);
    timeout = kCalibrationTimeoutIterations;
    while (adc_calibration_status_get(ADC1) == SET && timeout > 0U)
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        return false;
    }

    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR4CH4, TRUE);
    initialized_ = true;
    return true;
}

void PhaseCurrentMonitor::handleDmaNotification(uint32_t notification_value)
{
    if (!initialized_)
    {
        return;
    }

    const uint16_t buffer_offset = notification_value == kDmaNotifyFirstHalf
        ? 0U
        : (kCurrentAdcDmaBufferLength / 2U);

    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    for (uint8_t sample = 0U; sample < kCurrentAdcPairsPerHalf; ++sample)
    {
        const uint16_t sample_offset = static_cast<uint16_t>(buffer_offset + sample * 2U);
        sum_a += dma_buffer_[sample_offset];
        sum_b += dma_buffer_[sample_offset + 1U];
    }

    const uint16_t raw_a = static_cast<uint16_t>(sum_a / kCurrentAdcPairsPerHalf);
    const uint16_t raw_b = static_cast<uint16_t>(sum_b / kCurrentAdcPairsPerHalf);
    phase_a_current_a_ = rawToAmps(raw_a);
    phase_b_current_a_ = rawToAmps(raw_b);
}

float PhaseCurrentMonitor::phaseCurrentA() const
{
    return phase_a_current_a_;
}

float PhaseCurrentMonitor::phaseCurrentB() const
{
    return phase_b_current_a_;
}

bool PhaseCurrentMonitor::isInitialized() const
{
    return initialized_;
}
