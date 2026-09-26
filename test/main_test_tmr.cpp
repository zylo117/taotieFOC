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
// 0：原始无限循环模式； 1：跑完整一遍数组后自动停止输出; 2：软件分段给脉冲
//=====================================================

namespace
{
    void uart_send_str(const char *str);

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
    constexpr uint32_t target_tick_time = 8; // ns
    constexpr uint32_t target_pulse_width = 200; // ns, 需要保证足够宽的高电平，避免丢步
    constexpr uint32_t dir_to_step_setup_time = 20; // ns, DIR to STEP 最小提前时间
    constexpr uint32_t dir_to_step_hold_time = 20; // ns, DIR to STEP 最小保持时间

    // PSC=12 → tick = (12+1)/125M = 104ns
    // PSC=0 → tick = (0+1)/125M = 8ns
    constexpr uint32_t k_psc = ceil(target_tick_time * (F_APB / 1.0e9)) - 1;  // 8ns @ 125Mhz

     // 脉宽tick数/ARR都不可以大于计数器最大范围，比如16位就是2^16，32位就2^32
     // ARR需要大于脉宽tick数
     // 脉宽tick数到达ARR位置就会重置电平

     // CCR
     // 脉宽或负脉宽（取决于你，先触发就脉宽，先等待后触发就是负脉宽）
     // 脉宽tick 25, 200ns @ 125Mhz
    // 负脉宽tick数，每个周期等多久就开始触发上升沿
    constexpr uint32_t k_step_pulse_ticks = ceil(target_pulse_width / (float) target_tick_time);
    constexpr uint32_t k_dir_guard_ticks = ceil((dir_to_step_setup_time > dir_to_step_hold_time ? dir_to_step_setup_time : dir_to_step_hold_time) / (float) target_tick_time);
    constexpr uint32_t guard_tick = (k_dir_guard_ticks > 1U) ? k_dir_guard_ticks : 1U;

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
    // 200步/圈 * 64细分 = 12800个STEP脉冲；要求模式1执行：正转一圈 -> 反转一圈。
    constexpr uint32_t pulse_count = 1UL * 200UL * 64UL;
    constexpr uint32_t dir_toogle_index = pulse_count;
    // 125 MHz / (9765 + 1) = about 12800 STEP/s = 60 rpm at 64 microsteps.
    constexpr uint32_t step_period_tick = 9765UL;  // 匀速的话，每一周期（一个周期有且只有一步，每一周期就是每一步）就有那么多个tick

    // TMR2 是 32 位计数器，因此 ARR 序列必须是 32 位。
    // 这里复用同一份周期表，正转和反转都只需要切换 DIR 输出 + 重新装载同一块 RAM，
    // 这样既能完成“正转一圈 -> 反转一圈”，又不会把 RAM 翻倍消耗掉。

    // 人为规定第一个和最后arr周期是用来提前和延后换向的，
    // 如果是同向，这两个的arr周期为0，否则arr为guard_tick
    constexpr uint32_t seq_count = pulse_count + 2;  // 乘2是因为脉冲必须先高后低，高是一个ARR周期，低也是一个ARR周期
    uint32_t arr_seq_cycle[seq_count];  

    bool current_direction = true;

    void generate_step_sequence(uint32_t *seq, uint32_t seq_count) {
        for (uint32_t i = 1; i < seq_count - 1; i+=1){  //头尾一个是用来换向的，不是脉冲用的
            // 这里就贪方便匀速，实际测试要改成各种匀加速，S加速
            seq[i] = step_period_tick;
        }
    }

    void add_dir_to_step_sequence(uint32_t *seq, uint32_t seq_count, bool direction)
    {
        // 如果开局和上一次方向相同则不必加额外换向等待，否则等一个guard_tick
        // 但是末端一定要加，避免这一局最后一步脉冲结束不到guard_tick就进入下一局开局换向
        if (direction == current_direction) {
            seq[0] = 0;
        } else {
            seq[0] = guard_tick;
            current_direction = not current_direction;
        }
        seq[seq_count - 1U] = guard_tick;
    }

    void run_mode1_turn_cycle_test()
    {
        generate_step_sequence(arr_seq_cycle, seq_count);
        add_dir_to_step_sequence(arr_seq_cycle, seq_count, not current_direction);
    }


    void stop_timer_dma_for_reload()
    {
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
    }

    void dma_reload_arr_sequence(uint32_t *seq, uint32_t seq_count)
    {
        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);
        dma_conf.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_conf.buffer_size           = static_cast<uint16_t>(seq_count);
        dma_conf.peripheral_inc_enable  = FALSE;
        dma_conf.memory_inc_enable      = TRUE;
        dma_conf.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_conf.memory_data_width      = DMA_MEMORY_DATA_WIDTH_WORD;
        dma_conf.loop_mode_enable      = FALSE;
        dma_conf.priority              = DMA_PRIORITY_HIGH;

        dma_conf.peripheral_base_addr  = reinterpret_cast<uint32_t>(&TMR2->pr);
        dma_conf.memory_base_addr      = reinterpret_cast<uint32_t>(seq);
        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);

        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
    }

    void pwm_overflow_dma_config()
    {
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 0U);
        dma_reload_arr_sequence(arr_seq_cycle, seq_count);

    #if (PWM_SEQ_ONE_SHOT_MODE == 1U)
        // dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, TRUE);
        // nvic_irq_enable(DMA1_Channel2_IRQn, 2U, 0U);
    #else
        dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, FALSE);
    #endif
    }

    void timer_pwm_dma_config(bool direction)
    {
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);

        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR2, 1, k_psc);  // 1是垃圾值，只为了初始，后期会被dma的arr取代
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);  //ARR预装载，保证周期不会中途撕裂波形，高频必须打开

        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        output_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
        // 高电平宽度按真正被要求的脉宽保持，随后拉低，避免过窄导致被忽略。
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, k_step_pulse_ticks);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

        // CH4：DIR 使用同一个 ARR 计时基准；前后加保护时间，保证 DIR 在 STEP 产生前后都稳定。
        tmr_output_config_type dir_config;
        tmr_output_default_para_init(&dir_config);
        dir_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        dir_config.oc_idle_state = FALSE;
        dir_config.occ_idle_state = FALSE;
        dir_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        dir_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &dir_config);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, direction? 0U: 0xFFFFFFFF);
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

    void start_step_sequence(uint32_t *seq, uint32_t seq_count, bool direction)
    {
        stop_timer_dma_for_reload();
        dma_reload_arr_sequence(seq, seq_count);
        timer_pwm_dma_config(direction);
    }
}

// #if (PWM_SEQ_ONE_SHOT_MODE >= 1U)
// /**
//  * DMA1 Channel2 ISR：在 one-shot 模式下执行正转一圈 -> 反转一圈 -> 停止。
//  */
// extern "C" void DMA1_Channel2_IRQHandler(void)
// {
//     if(dma_flag_get(DMA1_FDT2_FLAG) != RESET)
//     {
//         dma_flag_clear(DMA1_FDT2_FLAG);
//         dma_channel_enable(DMA1_CHANNEL2, FALSE);

//         tmr_counter_enable(TMR2, FALSE);
//         tmr_force_output_set(TMR2, TMR_SELECT_CHANNEL_3, TMR_FORCE_OUTPUT_LOW);
//         tmr_output_enable(TMR2, FALSE);

// #if ENABLE_UART_DEBUG
//         uart_send_str("\r\n==== PWM SEQ ONE-SHOT FINISHED! TMR STOPPED ====\r\n");
// #endif
//     }
// }
// #endif

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

#if ENABLE_UART_DEBUG
        uart_send_str("fuck1\n");
#endif
    run_mode1_turn_cycle_test();
    pwm_overflow_dma_config();
    timer_pwm_dma_config(current_direction);

#if ENABLE_UART_DEBUG
        uart_send_str("fuck2\n");
#endif
#if ENABLE_UART_DEBUG
    uart_send_str("PR init = ");
    uart_print_num(tmr_period_value_get(TMR2));
#endif

delay_ms(1500);
    

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
// #if ENABLE_UART_DEBUG
//         uart_send_str("fuck\n");
// #endif

        // 设定, 20khz/4pulse/12v/1A
        uint32_t iter_delay_time_us = 10;  // us
        delay_us(iter_delay_time_us);
        uint32_t pulse_count = 1UL;
        // 匀速的话，每一周期（一个周期有且只有一步，每一周期就是每一步）就有那么多个tick
        uint32_t step_ticks = k_step_pulse_ticks * 2;  // 最少脉宽两倍，留足高电平脉宽之余的低电平脉宽

        uint32_t seq_count = pulse_count + 2;  // 乘2是因为脉冲必须先高后低，高是一个ARR周期，低也是一个ARR周期

        uint32_t max_iter_time_ns = (2 * guard_tick + step_ticks * pulse_count) * target_tick_time;

        // 因为硬件TMR定时器+DMA工作是异步的，耗时必须短于软件定时器迭代时间，否则就会输出延迟
        if (iter_delay_time_us * 1000 < max_iter_time_ns) {
#if ENABLE_UART_DEBUG
        uart_send_str("shitfuck, iter delay time too short, lower your iter rate.\n");
#endif
        }

        bool next_dir = true;

        uint32_t my_arr_seq_cycle[seq_count];
        // generate_step_sequence(my_arr_seq_cycle, seq_count);
        
        for (uint32_t i = 1; i < seq_count - 1; i++){  //头尾一个是用来换向的，不是脉冲用的
            // 这里就贪方便匀速，实际测试要改成各种匀加速，S加速
            my_arr_seq_cycle[i] = step_ticks;
        }
        add_dir_to_step_sequence(my_arr_seq_cycle, seq_count, next_dir);

        start_step_sequence(my_arr_seq_cycle, seq_count, 0);

// #if ENABLE_UART_DEBUG
//         uart_send_str("shit\n");
// #endif
        
#endif

        // ONE‑SHOT：序列跑完DMA‑FDT中断自动停机；
        // LOOP原始模式：无限循环输出；
    }
}
