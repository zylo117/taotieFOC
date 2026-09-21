/**
 * TMR1 CH1 PA8 LED验证版
 * 功能：PWM+溢出DMA 固定脉宽+可变间隔，肉眼可见逐级加速闪烁
 * 串口：115200 打印周期值
*/
#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include <stdio.h>
#include <string.h>

#define TEST_MODE_PWM_SIM      0
#define ENABLE_UART_DEBUG   1

namespace
{
    constexpr uint16_t kLedPin = GPIO_PINS_8;

    constexpr uint32_t k_psc = 1249UL;          // PSC=1249 → tick = 10us
    constexpr uint16_t fixed_pulse_width_tick = 200U; // 固定脉宽 2ms

    const uint16_t arr_seq[] =
    {
        50000U,   // 500ms 慢闪
        // 20000U,   // 200ms 慢闪
        10000U   // 100ms 慢闪
    };
    constexpr uint32_t pulse_count = sizeof(arr_seq)/sizeof(arr_seq[0]);

    void pwm_overflow_dma_config()
    {
        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);

        dma_conf.peripheral_base_addr  = reinterpret_cast<uint32_t>(&TMR1->pr);
        dma_conf.memory_base_addr      = reinterpret_cast<uint32_t>(arr_seq);
        dma_conf.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_conf.buffer_size           = static_cast<uint16_t>(pulse_count);

        dma_conf.peripheral_inc_enable  = FALSE;
        dma_conf.memory_inc_enable      = TRUE;

        dma_conf.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dma_conf.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD;

        dma_conf.loop_mode_enable       = TRUE;
        dma_conf.priority               = DMA_PRIORITY_HIGH;

        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR1_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
    }

    void timer_pwm_dma_config()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR1, arr_seq[0], k_psc);
        tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);

        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output_config.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output_config.oc_output_state = TRUE;
        output_config.occ_output_state = FALSE;

        tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, fixed_pulse_width_tick);

        tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);

        tmr_counter_value_set(TMR1, 0U);
        tmr_output_enable(TMR1, TRUE);
        tmr_counter_enable(TMR1, TRUE);
    }

    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

        gpio_init_struct.gpio_pins = kLedPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOA, &gpio_init_struct);
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

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    gpio_configuration();

#if ENABLE_UART_DEBUG
    uart1_debug_init();
    uart_send_str("Start test\r\n");
#endif

    pwm_overflow_dma_config();
    timer_pwm_dma_config();

    uart_send_str("PR init = ");
    uart_print_num(tmr_period_value_get(TMR1));

    uint32_t last_pr = 0;
    uint32_t tick_cnt = 0;

    while (1)
    {
        delay_ms(2);
        tick_cnt++;

        uint32_t now_pr = tmr_period_value_get(TMR1);
        if(now_pr != last_pr)
        {
            last_pr = now_pr;
            uart_send_str("PR = ");
            uart_print_num(now_pr);
        }

        if(tick_cnt >= 500)
        {
            tick_cnt = 0;
            uart_send_str("heartbeat\r\n");
        }
    }
}
