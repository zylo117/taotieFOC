/**
 * Timer-driven proof test.
 *
 * TMR2 CH3 -> PB10 (STEP)
 * TMR2 CH4 -> PB11 (DIR)
 * PA3 / EN = low active
 * LED5 blinks as heartbeat
 */

#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

namespace
{
    constexpr uint16_t kLedPin = GPIO_PINS_8;

    // at32的定时器的外部时钟频率fTMRxCLK（我简称fT）是cpu频率除以2，250Mhz就是125Mhz
    // PSC决定了定时器计数频率，越大计数越快，fC = fT/(PSC+1)
    // ARR指计数到多少就重置，指数到多少归零（周期），事件触发间隔T = (PSC+1)*(ARR+1) / fT
    // CCR指数到多少触发通道动作（翻转 / 高低电平）
    // 这里一定要搞明白，再SWITCH模式下，翻转状态是会继承的
    // 也就是你下一次计数重置并不会重置翻转状态，而是继承
    // 也就是SWITCH模式下CCR设置多少，占空比都是50%
    constexpr uint32_t PSC = 2249UL;
    constexpr uint32_t ARR = 9999UL;
    constexpr uint32_t CCR = 3333UL;


    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

        gpio_init_struct.gpio_pins = kLedPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOA, &gpio_init_struct);
    }



    void timer_configuration()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR1, ARR, PSC);
        tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);

        output_config.oc_mode = TMR_OUTPUT_CONTROL_SWITCH;
        output_config.oc_idle_state = FALSE;
        output_config.oc_output_state = TRUE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, CCR);

        tmr_output_enable(TMR1, TRUE);
        tmr_counter_enable(TMR1, TRUE);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    gpio_configuration();
    timer_configuration();

    while (1)
    {
        // LED toggling logic is handled by the timer interrupt
    }
}
