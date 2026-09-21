/**
 * Timer pulse simulation for PA8 / TMR1_CH1.
 *
 * 这个版本不再用 SWITCH 模式，因为 SWITCH 会天然固定 50% 占空比。
 * 我们用 PWM 模式来模拟“固定脉宽 + 可变间隔”的步进脉冲，适合验证定时器流程。
 * 频率不用跑到 768 kHz，目标是让 LED 的变化看得见，方便确认定时器输出流程真能跑。
 */

#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

namespace
{
    constexpr uint16_t kLedPin = GPIO_PINS_8;

    // 说明：
    // TMR1 的输入时钟大约是 CPU/2。这里我们用一个低速、看得见的模拟脉冲设置，
    // 方便确认输出流程正确，后面你可以直接把 ARR / CCR 改成更高的频率。
    // 参考公式：f_out ≈ fTMR / ((PSC + 1) * (ARR + 1))
    // 这里 fTMR ≈ 125 MHz，PSC = 1249，ARR = 99999 => 大约 1 Hz
    // 也就是 1 秒一个周期，亮/灭变化很容易看见。
    constexpr uint32_t kPsc = 1249UL;
    constexpr uint32_t kDefaultArr = 99999UL;
    constexpr uint32_t kDefaultCcr = 20000UL;

    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

        gpio_init_struct.gpio_pins = kLedPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOA, &gpio_init_struct);
    }

    void set_timer_pulse(uint32_t arr, uint32_t ccr)
    {
        // 先停定时器，再写 ARR/CCR，避免更新到一半被打断。
        tmr_counter_enable(TMR1, FALSE);
        tmr_period_value_set(TMR1, arr);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, ccr);
        tmr_counter_value_set(TMR1, 0U);
        tmr_counter_enable(TMR1, TRUE);
    }

    void timer_configuration()
    {
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        // 这里使用 PWM_MODE_A，输出是“高电平持续 CCR，之后低电平到 ARR”，
        // 这才是“固定脉宽 + 可变间隔”的正确思路，不再受 SWITCH 的 50% 固定占空比限制。
        tmr_base_init(TMR1, kDefaultArr, kPsc);
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
}

int main(void)
{
    system_clock_config();
    at32_board_init();

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    gpio_configuration();
    timer_configuration();

    uint32_t pulse_periods[] = {99999UL, 49999UL, 19999UL, 9999UL};
    uint32_t pulse_widths[] = {20000UL, 15000UL, 6000UL, 2000UL};

    while (1)
    {
        // 模拟一组“脉冲序列”：周期变短、脉宽变窄，方便观察 LED 的变化。
        // 你如果需要更高频率，直接把 ARR / CCR 改小即可；
        // 如果需要更长时间间隔，直接把 ARR 改大即可。
        for (uint32_t i = 0U; i < sizeof(pulse_periods) / sizeof(pulse_periods[0]); ++i)
        {
            set_timer_pulse(pulse_periods[i], pulse_widths[i]);
            delay_ms(500U);
        }
    }
}
