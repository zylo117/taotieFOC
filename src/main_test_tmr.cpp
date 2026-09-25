/**
 * TMR2 CH3 PB10 STEP / CH4 PB11 DIR PWM‑A ACTIVE_LOW + FLEX‑DMA加载ARR序列
 * 新增宏 PWM_SEQ_ONE_SHOT_MODE
 *  0 = 原始模式：DMA循环，无限重复输出arr_seq序列（原有逻辑完全保留）
 *  1 = 单次序列模式：完整跑完一遍arr_seq全部脉冲，硬件自动停机，PA8拉低，不再输出
 * tick =10us PSC=1249；如需768kHz修改PSC=0，ARR≈161
 * 重要：ONE_SHOT模式下，全部预定脉冲完整跑完之后才执行停机，不会中途切断脉冲
*/
#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ENABLE_UART_DEBUG   1

//===================== 模式切换宏 =====================
#define PWM_SEQ_ONE_SHOT_MODE     1
// 0：原始无限循环模式； 1：跑完整一遍数组后自动停止输出
//=====================================================

namespace
{
    constexpr uint16_t kStepPin = GPIO_PINS_10;
    constexpr uint16_t kDirPin = GPIO_PINS_11;
    constexpr uint16_t kEnPin = GPIO_PINS_3;

    // at32的定时器的外部时钟频率fTMRxCLK（我简称fT）是cpu频率除以2（APB总线），250Mhz就是125Mhz
    // PSC决定了定时器计数频率，越大计数越快，fC = fT/(PSC+1)
    // ARR指计数到多少就重置，指数到多少归零（周期），事件触发间隔T = (PSC+1)*(ARR+1) / fT
    // CCR指数到多少触发通道动作（翻转 / 高低电平）
    // 这里一定要搞明白，在SWITCH模式下，翻转状态是会继承的
    // 也就是你下一次计数重置并不会重置翻转状态，而是继承
    // 也就是SWITCH模式下CCR设置多少，占空比都是50%
    // 而在PWM模式下，是会重置的，不会继承！！！

    constexpr uint32_t F_APB = 125 * 1e6; // cpu主频的一半，125Mhz

    // PSC=12 → tick = (12+1)/125M = 104ns
    // PSC=0 → tick = (0+1)/125M = 8ns
    constexpr uint32_t target_tick_time = 8; // 8ns
    constexpr uint32_t k_psc = ceil(target_tick_time * (F_APB / 1.0e9)) - 1;  // 8ns @ 125Mhz

     // 固定脉宽6.82ms，脉宽tick数*psc对应的tick时间
     // 脉宽tick数/ARR都不可以大于计数器最大范围，比如16位就是2^16，32位就2^32
     // ARR需要大于脉宽tick数
     // 脉宽tick数到达ARR位置就会重置电平

     // CCR
     // 脉宽或负脉宽（取决于你，先触发就脉宽，先等待后触发就是负脉宽）
     // 脉宽tick 12，96ns
    constexpr uint32_t target_pulse_width = 200; // 8ns
    constexpr uint32_t fixed_pulse_width_tick = ceil(target_pulse_width / target_tick_time);  // 四舍五入往上取整（math.ceil） 

        /*
        模式	极性	        CNT<CCR	    CNT≥CCR	    CCR处边沿	ARR(溢出归零)边沿
        PWM‑A	ACTIVE_HIGH	    HIGH	    LOW	        下降沿	    上升沿
        PWM‑A	ACTIVE_LOW	    LOW	        HIGH	    上升沿	    下降沿  // 就要这个，步进脉冲
        PWM‑B	ACTIVE_HIGH	    LOW	        HIGH	    上升沿	    下降沿  // 就要这个，步进脉冲
        PWM‑B	ACTIVE_LOW	    HIGH	    LOW	        下降沿	    上升沿
        */
     // 也就是说你用PWM-A + ACTIVE_LOW或者PWM‑B + ACTIVE_HIGH就可以模拟步进脉冲
     // 从0计数到脉宽tick就是脉冲前，然后发生一个脉宽tick数长度的脉冲（上升沿）
     // 但是会持续到ARR数，也就是说你要结束这个脉冲，只需要把ARR设置到脉宽稍微大一点（至少1）就行
     // 但是有个问题，步进脉冲识别的是上升沿，也就是第一tick必须是低电平，也就是脉冲不可以一上来就是高电平
     // 步进的方向可以一上来就高，脉冲不行，切记
     // 也就是说：把ARR设置成你要触发的时间的位置的tick数+脉宽即可

    // 波形周期序列数组ARR
    // 每个周期代表这次脉冲持续多久才拉低（结束），也就是ARR越小，脉冲越密集
    // 也就是说可以通过这个来调整步进电机的转速/加速度
    // 200步/圈 * 64细分 * 2圈 = 25600个STEP脉冲。
    constexpr uint32_t pulse_count = 1UL * 200UL * 64UL;
    // 125 MHz / (9765 + 1) = about 12800 STEP/s = 60 rpm at 64 microsteps.
    constexpr uint32_t step_period_tick = 9765UL;

    // 所有周期都小于65535，因此用半字数组一次性装入25600个DMA周期值。
    uint32_t arr_seq[pulse_count];

    void fill_arr_sequence()
    {
        for (uint32_t i = 0U; i < pulse_count; ++i)
        {
            arr_seq[i] = step_period_tick;
        }
    }

    void pwm_overflow_dma_config()
    {
        fill_arr_sequence();

        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);
        dma_conf.peripheral_base_addr  = reinterpret_cast<uint32_t>(&TMR2->pr);
        dma_conf.memory_base_addr      = reinterpret_cast<uint32_t>(arr_seq);
        dma_conf.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_conf.buffer_size           = static_cast<uint16_t>(pulse_count);
        dma_conf.peripheral_inc_enable  = FALSE;
        dma_conf.memory_inc_enable      = TRUE;
        dma_conf.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_conf.memory_data_width      = DMA_MEMORY_DATA_WIDTH_WORD;

        // 一次DMA装入全部25600个脉冲；0模式循环，1模式完成后停止。
    #if (PWM_SEQ_ONE_SHOT_MODE == 1U)
        dma_conf.loop_mode_enable       = FALSE;
    #else
        dma_conf.loop_mode_enable       = TRUE;
    #endif
        dma_conf.priority               = DMA_PRIORITY_HIGH;

        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);

    #if (PWM_SEQ_ONE_SHOT_MODE == 1U)
        dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, TRUE);
        nvic_irq_enable(DMA1_Channel2_IRQn, 2U, 0U);
    #else
        dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, FALSE);
    #endif

        dma_channel_enable(DMA1_CHANNEL2, TRUE);

    }

    void timer_pwm_dma_config()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR2, arr_seq[0], k_psc);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);  //ARR预装载，保证周期不会中途撕裂波形，高频必须打开

        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        output_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, fixed_pulse_width_tick);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

        // CH4：同样使用 PWM-A + ACTIVE_LOW，通过软件定时修改 CCR4 实现 DIR 翻转。
        tmr_output_config_type dir_config;
        tmr_output_default_para_init(&dir_config);
        dir_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        dir_config.oc_idle_state = FALSE;
        dir_config.occ_idle_state = FALSE;
        dir_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        dir_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &dir_config);
        // DIR 与 STEP 共用 TMR2 的 ARR/计数周期；这里只修改 CCR4
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 0U);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);

        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);

        tmr_counter_value_set(TMR2, 0U);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);
    }

    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE); // 非常重要，重映射io必须有这句话
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

        // TMR2 CH3/CH4 are exposed on PB10/PB11 only after this remap is enabled.
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = kEnPin;
        gpio_init(GPIOA, &gpio_init_struct);
        gpio_bits_reset(GPIOA, kEnPin);

        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = LED5_PIN;
        gpio_init(GPIOA, &gpio_init_struct);
        gpio_bits_set(GPIOA, LED5_PIN);

        gpio_init_struct.gpio_pins = kStepPin | kDirPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOB, &gpio_init_struct);
    }

#if ENABLE_UART_DEBUG
    void uart1_debug_init()
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_pins = GPIO_PINS_9;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOA, &gpio_init_struct);

        usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
        usart_parity_selection_config(USART1, USART_PARITY_NONE);
        usart_transmitter_enable(USART1, TRUE);
        usart_enable(USART1, TRUE);
    }
    void uart_send_str(const char *str)
    {
        while(*str)
        {
            while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
            usart_data_transmit(USART1, static_cast<uint16_t>(*str++));
        }
    }
    void uart_print_num(uint32_t val)
    {
        char buf[16];
        sprintf(buf, "%lu\r\n", (unsigned long)val);
        uart_send_str(buf);
    }
#endif
}

#if (PWM_SEQ_ONE_SHOT_MODE == 1U)
/**
 * DMA1 Channel2 ISR：一次DMA完成全部25600个脉冲后停机。
 */
extern "C" void DMA1_Channel2_IRQHandler(void)
{
    if(dma_flag_get(DMA1_FDT2_FLAG) != RESET)
    {
        dma_flag_clear(DMA1_FDT2_FLAG);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);

        tmr_counter_enable(TMR2, FALSE);
        tmr_force_output_set(TMR2, TMR_SELECT_CHANNEL_3, TMR_FORCE_OUTPUT_LOW);
        tmr_output_enable(TMR2, FALSE);

#if ENABLE_UART_DEBUG
        uart_send_str("\r\n==== PWM SEQ ONE-SHOT FINISHED! TMR STOPPED ====\r\n");
#endif
    }
}
#endif

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    tmr_32_bit_function_enable(TMR2, TRUE);  // 32位定时器特殊函数

    gpio_configuration();
#if ENABLE_UART_DEBUG
    uart1_debug_init();
#if (PWM_SEQ_ONE_SHOT_MODE ==1U)
    uart_send_str("PWM DMA ONE‑SHOT MODE: run seq once then stop\r\n");
#else
    uart_send_str("PWM DMA LOOP MODE: infinite repeat seq(original)\r\n");
#endif
#endif

    pwm_overflow_dma_config();
    timer_pwm_dma_config();

#if ENABLE_UART_DEBUG
    uart_send_str("PR init = ");
    uart_print_num(tmr_period_value_get(TMR2));
#endif

#if (PWM_SEQ_ONE_SHOT_MODE == 0U)
    bool dir_high = true;
#endif
    while (1)
    {
#if (PWM_SEQ_ONE_SHOT_MODE == 0U)
        // STEP 的 ARR/脉冲时序由硬件 DMA+TMR 自主运行；
        // DIR 与 STEP 共用同一个 TMR2 周期，只在约 1 秒时修改一次 CCR4。
        delay_ms(1000U);
        // tmr_channel_value_set(
        //     TMR2,
        //     TMR_SELECT_CHANNEL_4,
        //     dir_high ? 0U : 0xFFFFFFFFUL);
        // dir_high = !dir_high;
#else
        // ONE-SHOT：全部轨迹由DMA完成中断续装，跑完后自动停机。
#endif

        // ONE‑SHOT：序列跑完DMA‑FDT中断自动停机；
        // LOOP原始模式：无限循环输出；
    }
}
