/**
 * TMR1 CH1 PA8 LED模拟STEP步进脉冲输出
 * 模式0：SWITCH(Toggle‑on‑match) + DMA + DMA‑FDT中断重放序列，不会卡死
 * 模式1：旧版PWM CPU干预模拟测试【对比用，低速，有抖动】
 * 硬件：AT32F403A‑407；PA8(TMR1_CH1)接LED
 * 功能：循环输出5个模拟步进脉冲，LED肉眼可见闪烁，跑通边沿输出流程
 * 后续迁移：把外设从TMR1切换到TMR2‑CH3 PB10作为真实STEP输出
*/
#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

// ======== 测试模式选择 0=DMA‑SWITCH+中断重放；1=旧CPU‑PWM模拟测试 ========
#define TEST_MODE_PWM_SIM      0

namespace
{
    constexpr uint16_t kLedPin = GPIO_PINS_8;   // PA8 TMR1_CH1 LED模拟STEP

    // ====================== 共用硬件参数 ======================
    constexpr uint32_t f_tmr_clk = 125000000UL; // TMR1 APB2时钟125MHz
    constexpr uint32_t k_psc = 1249UL;          // PSC=1249 → tick = 10us
    constexpr uint32_t tick_us = 10U;

#if (TEST_MODE_PWM_SIM == 0)
    // ---------------- DMA‑SWITCH(Toggle)模式参数 ----------------
    constexpr uint16_t fixed_pulse_width_tick = 100U; //固定脉宽100tick = 1ms，LED肉眼看得清脉冲
    constexpr uint16_t seg_window_tick = 2000U;       //每一段时间窗口固定2000tick(20ms)
    //四段：段0：1脉冲；段1：2脉冲；段2：3脉冲；段3：4脉冲
    constexpr uint8_t seg_pulse_cnt[] = {1U, 2U, 3U, 4U};
    constexpr uint8_t segment_num = sizeof(seg_pulse_cnt)/sizeof(seg_pulse_cnt[0]);

    //计算最大边沿总数：每段N个脉冲=2*N边沿
    constexpr uint32_t max_edge_total = 2U * (1U+2U+3U+4U); //20个边沿
    uint16_t edge_buf[max_edge_total];
    uint32_t edge_actual_total = 0; //实际生成边沿数量

    /**
     * @brief 预生成【逐级加密脉冲边沿数组】
     * 规则：每一段固定窗口seg_window_tick=2000tick
     * seg0(0~2000):1个脉冲
     * seg1(2000~4000):2个脉冲
     * seg2(4000~6000):3个脉冲
     * seg3(6000~8000):4个脉冲
     * 每段内部脉冲均匀分布；脉宽固定fixed_pulse_width_tick；
     * 输出edge_buf存放绝对tick；带start_offset消除上电毛刺
    */
    void build_edge_sequence()
{
    const uint16_t start_offset = 10U;
    uint32_t current_tick = start_offset;
    edge_actual_total = 0;

    //四阶段，每个阶段2000tick，累计脉冲数：1，3，6，10
    //每个阶段的【本阶段新增脉冲数量】
    const uint8_t add_pulse_per_phase[] = {1,3,5,8};
    const uint8_t phase_num = sizeof(add_pulse_per_phase)/sizeof(add_pulse_per_phase[0]);
    const uint32_t phase_len_tick = 2000U;
    uint32_t phase_start_tick = start_offset;

    for(uint8_t ph=0; ph < phase_num; ph++)
    {
        uint8_t new_pulse = add_pulse_per_phase[ph];
        uint32_t ph_end_tick = phase_start_tick + phase_len_tick;

        // 在本phase_start_tick ~ ph_end_tick之间均匀插入 new_pulse个完整脉冲
        // 可用时长 = phase_len_tick；每个脉冲占用：脉宽 + 静默间隔
        uint32_t avail = ph_end_tick - phase_start_tick;
        uint32_t gap_total = avail - new_pulse * fixed_pulse_width_tick;
        uint32_t gap_per_pulse = gap_total / new_pulse;

        for(uint8_t p=0; p < new_pulse; p++)
        {
            //上升沿
            uint32_t rise = current_tick;
            uint32_t fall = rise + fixed_pulse_width_tick;

            edge_buf[edge_actual_total++] = static_cast<uint16_t>(rise);
            edge_buf[edge_actual_total++] = static_cast<uint16_t>(fall);

            //下一个脉冲的起始：fall + gap_per_pulse；连续，没有强制跳到phase_start_tick！！
            current_tick = fall + gap_per_pulse;
        }
        phase_start_tick = ph_end_tick;
    }
}

    void dma_tmr1_ch1_config()
{
    dma_init_type dma_conf;
    dma_default_para_init(&dma_conf);

    dma_conf.peripheral_base_addr  = reinterpret_cast<uint32_t>(&TMR1->c1dt);
    dma_conf.memory_base_addr      = reinterpret_cast<uint32_t>(edge_buf);
    dma_conf.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
    // 修复：edge_total → edge_actual_total
    dma_conf.buffer_size           = static_cast<uint16_t>(edge_actual_total);

    dma_conf.peripheral_inc_enable  = FALSE;
    dma_conf.memory_inc_enable      = TRUE;

    dma_conf.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_conf.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD;

    dma_conf.loop_mode_enable       = FALSE;
    dma_conf.priority               = DMA_PRIORITY_HIGH;

    dma_init(DMA1_CHANNEL2, &dma_conf);

    dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR1_CH1);
    dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, TRUE);
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
}

    void timer_toggle_dma_config()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR1, 65535UL, k_psc);
        tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);

        // SWITCH = Toggle‑on‑match，CNT==CCR1翻转OC输出
        output_config.oc_mode = TMR_OUTPUT_CONTROL_SWITCH;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output_config.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output_config.oc_output_state = TRUE;
        output_config.occ_output_state = FALSE;

        tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, edge_buf[0]);

        //定时器侧打开CH1匹配DMA请求输出
        tmr_dma_request_enable(TMR1, TMR_C1_DMA_REQUEST, TRUE);

        tmr_counter_value_set(TMR1, 0U);
        tmr_output_enable(TMR1, TRUE);  //TMR1高级定时器MOE主输出使能，必须打开
        tmr_counter_enable(TMR1, TRUE);
    }

    /**
     * @brief DMA1 Channel2 全传输完成中断：一整组边沿播放完毕，重新复位定时器+DMA，循环重放序列
    */
    extern "C" void DMA1_Channel2_IRQHandler(void)
{
    if(dma_flag_get(DMA1_FDT2_FLAG) != RESET)
    {
        tmr_counter_enable(TMR1, FALSE);

        tmr_counter_value_set(TMR1, 0U);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, edge_buf[0]);

        //修复 edge_total → edge_actual_total
        dma_data_number_set(DMA1_CHANNEL2, static_cast<uint16_t>(edge_actual_total));

        dma_channel_enable(DMA1_CHANNEL2, TRUE);

        tmr_counter_enable(TMR1, TRUE);

        dma_flag_clear(DMA1_FDT2_FLAG);
    }
}

    /**
     * @brief DMA1 Channel2 中断NVIC配置 AT32库版本
    */
    void dma1_ch2_nvic_config()
    {
        // main中已经执行 nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
        // 参数：IRQ号，抢占优先级=2，子优先级=0（GROUP4子优先级无效）
        nvic_irq_enable(DMA1_Channel2_IRQn, 2U, 0U);
    }

#elif (TEST_MODE_PWM_SIM == 1)
    // ----------------旧CPU‑PWM模拟测试模式（对比用）----------------
    constexpr uint32_t kDefaultArr = 9999UL;
    constexpr uint32_t kDefaultCcr = 2000UL;

    void set_timer_pulse(uint32_t arr, uint32_t ccr)
    {
        tmr_counter_enable(TMR1, FALSE);
        tmr_period_value_set(TMR1, arr);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, ccr);
        tmr_counter_value_set(TMR1, 0U);
        tmr_counter_enable(TMR1, TRUE);
    }

    void timer_pwm_sim_config()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR1, kDefaultArr, k_psc);
        tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);

        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.oc_output_state = TRUE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;

        tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, kDefaultCcr);
        tmr_output_enable(TMR1, TRUE);
        tmr_counter_enable(TMR1, TRUE);
    }
#endif

    // ====================== 共用GPIO配置 ======================
    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
#if (TEST_MODE_PWM_SIM ==0)
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
#endif

        gpio_init_struct.gpio_pins = kLedPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOA, &gpio_init_struct);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    gpio_configuration();

#if (TEST_MODE_PWM_SIM == 0)
    // ========= DMA‑SWITCH + DMA‑FDT中断重放【模拟STEP脉冲，LED闪烁】 =========
    build_edge_sequence();
    dma1_ch2_nvic_config();
    dma_tmr1_ch1_config();
    timer_toggle_dma_config();

    while (1)
    {
        //脉冲全部硬件+DMA中断自主循环播放；主循环空闲
    }

#elif (TEST_MODE_PWM_SIM == 1)
    // =========旧CPU干预PWM模拟测试模式【对比，低速】 =========
    timer_pwm_sim_config();
    uint32_t pulse_periods[] = {9999UL, 4999UL, 1999UL, 999UL};
    uint32_t pulse_widths[]  = {2000UL, 1500UL, 600UL, 200UL};

    while (1)
    {
        for (uint32_t i = 0U; i < sizeof(pulse_periods)/sizeof(pulse_periods[0]); ++i)
        {
            set_timer_pulse(pulse_periods[i], pulse_widths[i]);
            delay_ms(500U);
        }
    }
#endif
}
