#include "stepper_driver.h"
#include "at32f403a_407_board.h"

namespace stepper_common
{
    namespace
    {
        bool motion_timer_running = false;
        uint32_t motion_timer_frequency = 0U;
        uint32_t motion_timer_pulse_width = 0U;
    }

    void stepper_write_gpio_high(gpio_type* port, uint16_t pin) { port->scr = pin; }
    void stepper_write_gpio_low(gpio_type* port, uint16_t pin) { port->clr = pin; }

    void stepper_write_gpio(gpio_type* port, uint16_t pin, bool state)
    {
        if (state)
        {
            port->scr = pin;
        }
        else
        {
            port->clr = pin;
        }
    }

    void stepper_delay_us(uint32_t microseconds)
    {
        delay_us(microseconds);
    }

    void stepper_delay_ns(uint64_t nanoseconds)
    {
        delay_ns(nanoseconds);
    }

    // k_psc: 每tick时间， k_step_pulse_ticks: 脉宽tick数， direction: true为正
    void stepper_init_motion_timer(uint32_t k_psc, uint32_t k_step_pulse_ticks, bool direction)
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

    // void stepper_start_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    // {
    //     if (step_hz == 0U)
    //     {
    //         stepper_stop_motion_timer();
    //         return;
    //     }
    //
    //     crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    //     gpio_pin_remap_config(TMR2_MUX_10, TRUE);
    //
    //     gpio_init_type gpio_init_struct;
    //     gpio_default_para_init(&gpio_init_struct);
    //     gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    //     gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    //     gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    //     gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    //     gpio_init_struct.gpio_pins = STEP_OUT_PIN;
    //     gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);
    //
    //     // TMR2_MUX_10 also selects PB11 as TMR2_CH4. Re-assert PB11 as a normal
    //     // GPIO so DIR remains a static logic output while PB10 carries STEP.
    //     gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    //     gpio_init_struct.gpio_pins = DIR_OUT_PIN;
    //     gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);
    //
    //     const uint32_t timer_tick_hz = system_core_clock;
    //     uint32_t period = timer_tick_hz / step_hz;
    //     if (period < 2U)
    //     {
    //         period = 2U;
    //     }
    //     --period;
    //
    //     uint32_t pulse_ticks = (timer_tick_hz / 1000000U) * pulse_width_us;
    //     if (pulse_ticks < 1U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     if (pulse_ticks > period)
    //     {
    //         pulse_ticks = period / 2U;
    //     }
    //     if (pulse_ticks == 0U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //
    //     tmr_counter_enable(TMR2, FALSE);
    //     tmr_base_init(TMR2, period, 0U);
    //     tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    //
    //     tmr_output_config_type output_config;
    //     tmr_output_default_para_init(&output_config);
    //     output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    //     output_config.oc_output_state = TRUE;
    //     output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    //     output_config.oc_idle_state = FALSE;
    //     tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
    //     tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
    //     tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
    //     tmr_output_enable(TMR2, TRUE);
    //     tmr_counter_enable(TMR2, TRUE);
    //
    //     motion_timer_frequency = step_hz;
    //     motion_timer_pulse_width = pulse_width_us;
    //     motion_timer_running = true;
    // }
    //
    // void stepper_update_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    // {
    //     if (!motion_timer_running)
    //     {
    //         stepper_start_motion_timer(step_hz, pulse_width_us);
    //         return;
    //     }
    //     if (step_hz == 0U)
    //     {
    //         stepper_stop_motion_timer();
    //         return;
    //     }
    //     if (step_hz == motion_timer_frequency && pulse_width_us == motion_timer_pulse_width)
    //     {
    //         return;
    //     }
    //
    //     const uint32_t timer_tick_hz = system_core_clock;
    //     uint32_t period = timer_tick_hz / step_hz;
    //     if (period < 2U)
    //     {
    //         period = 2U;
    //     }
    //     --period;
    //     uint32_t pulse_ticks = (timer_tick_hz / 1000000U) * pulse_width_us;
    //     if (pulse_ticks < 1U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     if (pulse_ticks > period)
    //     {
    //         pulse_ticks = period / 2U;
    //     }
    //     if (pulse_ticks == 0U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     tmr_period_value_set(TMR2, period);
    //     tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
    //     motion_timer_frequency = step_hz;
    //     motion_timer_pulse_width = pulse_width_us;
    // }
    //
    // void stepper_stop_motion_timer(void)
    // {
    //     tmr_counter_enable(TMR2, FALSE);
    //     tmr_output_enable(TMR2, FALSE);
    //     tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, FALSE);
    //     gpio_pin_remap_config(TMR2_MUX_10, FALSE);
    //
    //     gpio_init_type gpio_init_struct;
    //     gpio_default_para_init(&gpio_init_struct);
    //     gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    //     gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    //     gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    //     gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    //     gpio_init_struct.gpio_pins = STEP_OUT_PIN | DIR_OUT_PIN;
    //     gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);
    //     stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, false);
    //     motion_timer_running = false;
    //     motion_timer_frequency = 0U;
    //     motion_timer_pulse_width = 0U;
    // }

    void stepper_init_step_gpio(void)
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE); // 非常重要，重映射io必须有这句话
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
        gpio_default_para_init(&gpio_init_struct);

        // TMR2 CH3/CH4 are exposed on PB10/PB11 only after this remap is enabled.
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
#ifdef USE_SOFT_PULSE
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
#else
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
#endif
        gpio_init_struct.gpio_pins = STEP_OUT_PIN | DIR_OUT_PIN;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

        gpio_init_struct.gpio_pins = EN_OUT_PIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init(EN_OUTPUT_PORT, &gpio_init_struct);
        stepper_init_motion_timer(k_psc, k_step_pulse_ticks, true);
        stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, true);
        stepper_write_gpio(DIR_OUTPUT_PORT, DIR_OUT_PIN, false);
        stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, false);
    }

    void stepper_init_uart_gpio(gpio_type* port, uint16_t pin)
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = pin;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(port, &gpio_init_struct);
        stepper_write_gpio(port, pin, true);
    }
}
