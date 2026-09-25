#include "stepper_driver.h"
#include "at32f403a_407_board.h"
#include "core_cm4.h"

#define PWM_SEQ_ONE_SHOT_MODE 1U

namespace stepper_common
{
    namespace
    {
        bool motion_timer_running = false;
        bool g_hardware_timer_ready = false;
        bool g_dma_window_active = false;
        bool g_dma_compare_active = false;
        StepDirCaptureEvent g_step_dir_ring[kStepDirCaptureRingDepth];
        volatile uint32_t g_ring_head = 0U;
        volatile uint32_t g_ring_tail = 0U;
        volatile uint32_t g_ring_count = 0U;
        constexpr uint32_t kMaxDmaBurstRows = (kStepDirCaptureRingDepth * 2U) + 1U;
        uint32_t g_dma_window_arr[kMaxHardwareWindowPulses];
        uint32_t g_dma_capture_arr[kMaxDmaBurstRows];
        uint32_t g_dma_capture_compare[kMaxDmaBurstRows * 2U];
        static_assert((kMaxDmaBurstRows * 2U) <= kMaxHardwareWindowPulses,
                  "DMA burst buffer is too small for a capture batch");
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

    static uint32_t stepper_ns_to_ticks(uint64_t nanoseconds)
    {
        uint64_t ticks = (nanoseconds * stepper_timer_clock_hz() + 999999999ULL) / 1000000000ULL;
        if (ticks < 1ULL)
        {
            ticks = 1ULL;
        }
        return ticks > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(ticks);
    }

    static uint32_t stepper_ns_to_prescaled_ticks(uint64_t nanoseconds, uint16_t prescaler)
    {
        const uint64_t divisor = static_cast<uint64_t>(prescaler) + 1ULL;
        uint64_t ticks = (nanoseconds * stepper_timer_clock_hz() +
                          (1000000000ULL * divisor) - 1ULL) /
                         (1000000000ULL * divisor);
        if (ticks < 1ULL)
        {
            ticks = 1ULL;
        }
        return ticks > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(ticks);
    }

    static uint16_t stepper_timer_prescaler_for_ticks(uint64_t raw_ticks)
    {
        uint64_t divisor = 1ULL;
        while (raw_ticks > (0xFFFFFFFFULL * divisor) && divisor < 0x10000ULL)
        {
            ++divisor;
        }
        return static_cast<uint16_t>(divisor - 1ULL);
    }

    static uint32_t stepper_timer_period_ticks(uint32_t step_hz, uint16_t prescaler)
    {
        if (step_hz == 0U)
        {
            return 0xFFFFFFFFU;
        }

        const uint64_t timer_tick_hz = stepper_timer_clock_hz();
        const uint64_t timer_divisor = static_cast<uint64_t>(prescaler) + 1ULL;
        uint64_t period_ticks = (timer_tick_hz + (static_cast<uint64_t>(step_hz) * timer_divisor) - 1ULL) /
                                (static_cast<uint64_t>(step_hz) * timer_divisor);
        if (period_ticks < 2ULL)
        {
            period_ticks = 2ULL;
        }
        return period_ticks > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(period_ticks - 1ULL);
    }

    uint32_t stepper_timer_clock_hz(void)
    {
        return system_core_clock / 2U;
    }

    static void stepper_configure_hardware_timer2(void);
    static void stepper_resume_capture_timer(void);

    bool stepper_dma_window_is_busy(void)
    {
        if (!g_dma_window_active)
        {
            return false;
        }
        if (dma_flag_get(DMA1_FDT2_FLAG) == RESET ||
            (g_dma_compare_active && dma_flag_get(DMA1_FDT1_FLAG) == RESET))
        {
            return true;
        }

        dma_flag_clear(DMA1_FDT2_FLAG);
        dma_flag_clear(DMA1_FDT1_FLAG);
        dma_channel_enable(DMA1_CHANNEL1, FALSE);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        tmr_counter_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_3,
                                       TMR_OUTPUT_CONTROL_FORCE_LOW);
        g_dma_window_active = false;
                        g_dma_compare_active = false;
        stepper_resume_capture_timer();
        return false;
    }

    bool stepper_plan_dma_capture_sequence(const StepPulseScheduleEntry* pulses,
                                           uint32_t pulse_count,
                                           uint64_t pulse_width_ns)
    {
        if (pulses == nullptr || pulse_count == 0U || pulse_count > kStepDirCaptureRingDepth ||
            stepper_dma_window_is_busy())
        {
            return false;
        }

        stepper_configure_hardware_timer2();
        const uint64_t timer_hz = stepper_timer_clock_hz();
        const uint32_t raw_pulse_ticks = stepper_ns_to_ticks(
            (pulse_width_ns < kStepPulseMinimumNs) ? kStepPulseMinimumNs : pulse_width_ns);
        const uint16_t prescaler = stepper_timer_prescaler_for_ticks(raw_pulse_ticks);
        const uint32_t divisor = static_cast<uint32_t>(prescaler) + 1U;
        uint32_t pulse_ticks = raw_pulse_ticks / divisor;
        if (pulse_ticks == 0U)
        {
            pulse_ticks = 1U;
        }

        const uint32_t guard_ticks = stepper_ns_to_prescaled_ticks(kDirectionSetupHoldMinimumNs, prescaler);
        uint32_t row_count = 0U;
        bool current_direction = pulses[0].direction;
        const uint32_t initial_compare = guard_ticks;
        const uint32_t initial_arr = initial_compare + pulse_ticks;
        const bool initial_direction = pulses[0].direction;

        const auto append_row = [&](uint32_t arr_value, uint32_t compare_value, bool direction)
        {
            g_dma_capture_arr[row_count] = arr_value;
            g_dma_capture_compare[row_count * 2U] = compare_value;
            g_dma_capture_compare[(row_count * 2U) + 1U] = direction ? 0xFFFFFFFFU : 0U;
            ++row_count;
        };

        for (uint32_t i = 1U; i < pulse_count; ++i)
        {
            const uint32_t delta_cycles = pulses[i].timestamp_cycles - pulses[i - 1U].timestamp_cycles;
            uint64_t interval_ticks =
                (static_cast<uint64_t>(delta_cycles) * timer_hz + (system_core_clock - 1U)) /
                system_core_clock;
            if (interval_ticks < static_cast<uint64_t>(pulse_ticks) + 2ULL)
            {
                interval_ticks = static_cast<uint64_t>(pulse_ticks) + 2ULL;
            }

            if (pulses[i].direction != current_direction)
            {
                const uint32_t guard_arr = (guard_ticks > 1U) ? (guard_ticks - 1U) : 1U;
                append_row(guard_arr, guard_arr + 1U, current_direction);
                current_direction = pulses[i].direction;

                const uint64_t minimum_interval = static_cast<uint64_t>(guard_ticks) * 2ULL +
                                                  pulse_ticks + 2ULL;
                if (interval_ticks < minimum_interval)
                {
                    interval_ticks = minimum_interval;
                }
                uint64_t arr_value = interval_ticks - guard_ticks - 1ULL;
                if (arr_value <= pulse_ticks)
                {
                    arr_value = static_cast<uint64_t>(pulse_ticks) + 1ULL;
                }
                append_row(static_cast<uint32_t>(arr_value),
                           static_cast<uint32_t>(arr_value - pulse_ticks), current_direction);
            }
            else
            {
                const uint64_t arr_value = interval_ticks - 1ULL;
                append_row(static_cast<uint32_t>(arr_value),
                           static_cast<uint32_t>(arr_value - pulse_ticks), current_direction);
            }
        }

        const uint32_t terminal_arr = (guard_ticks > 1U) ? (guard_ticks - 1U) : 1U;
        append_row(terminal_arr, terminal_arr + 1U, current_direction);
        append_row(terminal_arr, terminal_arr + 1U, current_direction);

        if (row_count < 2U || row_count > kMaxDmaBurstRows)
        {
            return false;
        }

        tmr_counter_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        tmr_32_bit_function_enable(TMR2, TRUE);
        tmr_base_init(TMR2, initial_arr, prescaler);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);

        tmr_output_config_type step_config;
        tmr_output_default_para_init(&step_config);
        step_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        step_config.oc_idle_state = FALSE;
        step_config.occ_idle_state = FALSE;
        step_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        step_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &step_config);
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, FALSE);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, initial_arr - pulse_ticks);
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

        tmr_output_config_type direction_config;
        tmr_output_default_para_init(&direction_config);
        direction_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        direction_config.oc_idle_state = FALSE;
        direction_config.occ_idle_state = FALSE;
        direction_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        direction_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &direction_config);
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_4, FALSE);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4,
                              initial_direction ? 0xFFFFFFFFU : 0U);
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);

        // Prime cycle 2 before starting; DMA writes at overflow take effect one update later.
        tmr_period_value_set(TMR2, g_dma_capture_arr[0]);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, g_dma_capture_compare[0]);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, g_dma_capture_compare[1]);

        tmr_dma_control_config(TMR2, TMR_DMA_TRANSFER_2BYTES, TMR_C3DT_ADDRESS);

        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);
        dma_conf.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        const uint32_t dma_row_count = row_count - 1U;
        dma_conf.buffer_size = static_cast<uint16_t>(dma_row_count);
        dma_conf.peripheral_inc_enable = FALSE;
        dma_conf.memory_inc_enable = TRUE;
        dma_conf.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_conf.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
#if (PWM_SEQ_ONE_SHOT_MODE == 1U)
        dma_conf.loop_mode_enable = FALSE;
#else
        dma_conf.loop_mode_enable = TRUE;
#endif
        dma_conf.priority = DMA_PRIORITY_HIGH;
        dma_conf.peripheral_base_addr = reinterpret_cast<uint32_t>(&TMR2->pr);
        dma_conf.memory_base_addr = reinterpret_cast<uint32_t>(&g_dma_capture_arr[1]);

        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);

        dma_conf.buffer_size = static_cast<uint16_t>(dma_row_count * 2U);
        dma_conf.peripheral_base_addr = reinterpret_cast<uint32_t>(&TMR2->dmadt);
        dma_conf.memory_base_addr = reinterpret_cast<uint32_t>(&g_dma_capture_compare[2]);
        dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL1, &dma_conf);

        dma_flag_clear(DMA1_FDT2_FLAG);
        dma_flag_clear(DMA1_FDT1_FLAG);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);
        dma_channel_enable(DMA1_CHANNEL1, TRUE);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
        g_dma_window_active = true;
        g_dma_compare_active = true;

        tmr_counter_value_set(TMR2, 0U);
        tmr_one_cycle_mode_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);
        return true;
    }

    static void stepper_configure_hardware_timer2(void)
    {
        if (g_hardware_timer_ready)
        {
            return;
        }

        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pins = STEP_OUT_PIN | DIR_OUT_PIN;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

        tmr_output_config_type ch3_cfg;
        tmr_output_default_para_init(&ch3_cfg);
        ch3_cfg.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        ch3_cfg.oc_idle_state = FALSE;
        ch3_cfg.occ_idle_state = FALSE;
        ch3_cfg.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        ch3_cfg.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &ch3_cfg);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, 1U);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

        tmr_output_config_type ch4_cfg;
        tmr_output_default_para_init(&ch4_cfg);
        ch4_cfg.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
        ch4_cfg.oc_idle_state = FALSE;
        ch4_cfg.occ_idle_state = FALSE;
        ch4_cfg.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        ch4_cfg.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &ch4_cfg);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 0U);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);

        tmr_base_init(TMR2, 1U, 0U);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        tmr_output_enable(TMR2, TRUE);
        g_hardware_timer_ready = true;
    }

    static void stepper_resume_capture_timer(void)
    {
        tmr_counter_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        tmr_one_cycle_mode_enable(TMR2, FALSE);
        tmr_32_bit_function_enable(TMR2, TRUE);
        tmr_base_init(TMR2, 0xFFFFFFFFU, 0U);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_3,
                                       TMR_OUTPUT_CONTROL_FORCE_LOW);
        tmr_counter_value_set(TMR2, 0U);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);
    }

    static void stepper_config_direction_output(bool direction)
    {
        stepper_configure_hardware_timer2();
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_4,
            direction ? TMR_OUTPUT_CONTROL_FORCE_HIGH : TMR_OUTPUT_CONTROL_FORCE_LOW);
    }

    static void stepper_config_step_output(uint32_t compare_ticks)
    {
        stepper_configure_hardware_timer2();
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);
        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        output_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, compare_ticks);
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
    }

    void stepper_set_direction(bool direction)
    {
        stepper_config_direction_output(direction ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
    }

    void stepper_set_step_state(bool state)
    {
        if (state)
        {
            stepper_send_step_pulse(kDefaultStepPulseWidthNs);
            return;
        }

        tmr_counter_enable(TMR2, FALSE);
        dma_channel_enable(DMA1_CHANNEL1, FALSE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_FORCE_LOW);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        dma_flag_clear(DMA1_FDT1_FLAG);
        dma_flag_clear(DMA1_FDT2_FLAG);
        g_dma_window_active = false;
        g_dma_compare_active = false;
        stepper_resume_capture_timer();
    }

    void stepper_send_step_pulse(uint64_t width_ns)
    {
        stepper_configure_hardware_timer2();

        const uint32_t pulse_ticks = stepper_ns_to_ticks(
            (width_ns < kStepPulseMinimumNs) ? kStepPulseMinimumNs : width_ns);
        // PWM-A + ACTIVE_LOW: rising edge is CCR, falling edge is ARR overflow.
        // Keep one timer tick low before the rising edge, then hold STEP high
        // for exactly pulse_ticks before the one-shot period ends.
        const uint32_t compare_ticks = 1U;
        const uint32_t arr_value = compare_ticks + pulse_ticks;
        const uint16_t timer_prescaler = stepper_timer_prescaler_for_ticks(arr_value);
        tmr_counter_enable(TMR2, FALSE);
        tmr_counter_value_set(TMR2, 0U);
        tmr_base_init(TMR2, arr_value, timer_prescaler);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        stepper_config_step_output(compare_ticks);
        tmr_one_cycle_mode_enable(TMR2, TRUE);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);
    }

    bool stepper_plan_dma_window(uint32_t pulse_count,
                                bool direction,
                                uint32_t step_hz,
                                uint64_t pulse_width_ns)
    {
        if (pulse_count == 0U || pulse_count > kMaxHardwareWindowPulses || step_hz == 0U)
        {
            return false;
        }
        if (g_dma_window_active && stepper_dma_window_is_busy())
        {
            return false;
        }

        stepper_configure_hardware_timer2();
        stepper_set_direction(direction);

        const uint32_t raw_pulse_ticks = stepper_ns_to_ticks(
            (pulse_width_ns < kStepPulseMinimumNs) ? kStepPulseMinimumNs : pulse_width_ns);
        const uint16_t timer_prescaler = stepper_timer_prescaler_for_ticks(raw_pulse_ticks);
        uint32_t requested_period_ticks = stepper_timer_period_ticks(step_hz, timer_prescaler);
        uint32_t pulse_ticks = raw_pulse_ticks / (static_cast<uint32_t>(timer_prescaler) + 1U);
        if (pulse_ticks < 1U)
        {
            pulse_ticks = 1U;
        }
        if (pulse_ticks >= requested_period_ticks)
        {
            pulse_ticks = requested_period_ticks - 1U;
        }
        const uint32_t guard_ticks = stepper_ns_to_prescaled_ticks(
            kDirectionSetupHoldMinimumNs, timer_prescaler);
        const uint32_t minimum_period_ticks = pulse_ticks + guard_ticks;
        if (requested_period_ticks < minimum_period_ticks)
        {
            requested_period_ticks = minimum_period_ticks;
        }
        const uint32_t compare_ticks = requested_period_ticks - pulse_ticks;
        for (uint32_t i = 0U; i < pulse_count; ++i)
        {
            g_dma_window_arr[i] = requested_period_ticks;
        }

        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);
        dma_conf.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_conf.buffer_size = static_cast<uint16_t>(pulse_count);
        dma_conf.peripheral_inc_enable = FALSE;
        dma_conf.memory_inc_enable = TRUE;
        dma_conf.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_conf.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
        dma_conf.loop_mode_enable = FALSE;
        dma_conf.priority = DMA_PRIORITY_HIGH;
        dma_conf.peripheral_base_addr = reinterpret_cast<uint32_t>(&TMR2->pr);
        dma_conf.memory_base_addr = reinterpret_cast<uint32_t>(g_dma_window_arr);

        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);

        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
        g_dma_window_active = true;
        g_dma_compare_active = false;

        tmr_counter_enable(TMR2, FALSE);
        tmr_counter_value_set(TMR2, 0U);
        tmr_base_init(TMR2, requested_period_ticks, timer_prescaler);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        stepper_config_step_output(compare_ticks);
        tmr_one_cycle_mode_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);

        return true;
    }

    void stepper_init_capture_ring_buffer(void)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        g_ring_head = 0U;
        g_ring_tail = 0U;
        g_ring_count = 0U;
        for (uint32_t i = 0U; i < kStepDirCaptureRingDepth; ++i)
        {
            g_step_dir_ring[i].tick = 0U;
            g_step_dir_ring[i].step = false;
            g_step_dir_ring[i].dir = false;
        }
    }

    bool stepper_push_capture_event(uint32_t tick, bool step_state, bool dir_state)
    {
        if (g_ring_count >= kStepDirCaptureRingDepth)
        {
            return false;
        }

        g_step_dir_ring[g_ring_head].tick = tick;
        g_step_dir_ring[g_ring_head].step = step_state;
        g_step_dir_ring[g_ring_head].dir = dir_state;
        g_ring_head = (g_ring_head + 1U) % kStepDirCaptureRingDepth;
        g_ring_count++;
        return true;
    }

    bool stepper_pop_capture_event(StepDirCaptureEvent* output)
    {
        if (output == nullptr || g_ring_count == 0U)
        {
            return false;
        }

        *output = g_step_dir_ring[g_ring_tail];
        g_ring_tail = (g_ring_tail + 1U) % kStepDirCaptureRingDepth;
        g_ring_count--;
        return true;
    }

    void stepper_reset_capture_ring_buffer(void)
    {
        stepper_init_capture_ring_buffer();
    }

    void stepper_queue_simulated_pulse_sequence(uint32_t pulse_count, bool direction, uint32_t start_hz, uint32_t max_hz, uint32_t accel_hz_s)
    {
        (void)start_hz;
        (void)max_hz;
        (void)accel_hz_s;
        for (uint32_t i = 0U; i < pulse_count; ++i)
        {
            stepper_push_capture_event((uint32_t)(i * 1000U), true, direction);
            stepper_push_capture_event((uint32_t)(i * 1000U + 20U), false, direction);
        }
    }

    void stepper_init_motion_timer(void)
    {
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, FALSE);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        motion_timer_running = false;
        stepper_set_step_state(false);
    }

    void stepper_start_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    {
        if (step_hz == 0U)
        {
            stepper_stop_motion_timer();
            return;
        }

        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        // TMR2 CH3/CH4 are exposed on PB10/PB11 only after this remap is enabled.
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_pins = STEP_OUT_PIN;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pins = DIR_OUT_PIN;
        gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);

        (void)step_hz;
        (void)pulse_width_us;
        stepper_set_step_state(false);
        motion_timer_running = true;
    }

    void stepper_update_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    {
        if (!motion_timer_running)
        {
            stepper_start_motion_timer(step_hz, pulse_width_us);
            return;
        }
        if (step_hz == 0U)
        {
            stepper_stop_motion_timer();
            return;
        }
        (void)step_hz;
        (void)pulse_width_us;
    }

    void stepper_stop_motion_timer(void)
    {
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, FALSE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, FALSE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, FALSE);
        tmr_one_cycle_mode_enable(TMR2, FALSE);

        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_pins = STEP_OUT_PIN;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);
        gpio_init_struct.gpio_pins = DIR_OUT_PIN;
        gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);
        stepper_set_step_state(false);
        motion_timer_running = false;
    }

    void stepper_init_tmr2_capture_and_oc(void)
    {
        // TMR2 CH3/CH4 are routed to PB10/PB11 only after the remap is applied.
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        // Configure TMR2 CH1 = DIR capture on PA15, CH2 = STEP capture on PB3.
        tmr_input_config_type ic_init;
        tmr_input_default_para_init(&ic_init);
        ic_init.input_channel_select = TMR_SELECT_CHANNEL_1;
        ic_init.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
        ic_init.input_polarity_select = TMR_INPUT_RISING_EDGE;
        tmr_input_channel_init(TMR2, &ic_init, TMR_CHANNEL_INPUT_DIV_1);

        tmr_input_default_para_init(&ic_init);
        ic_init.input_channel_select = TMR_SELECT_CHANNEL_2;
        ic_init.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
        ic_init.input_polarity_select = TMR_INPUT_RISING_EDGE;
        tmr_input_channel_init(TMR2, &ic_init, TMR_CHANNEL_INPUT_DIV_1);

        // Configure TMR2 CH3/CH4 as the hard output compare path for STEP/DIR generation.
        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);
        output_config.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
        output_config.oc_idle_state = FALSE;
        output_config.oc_output_state = TRUE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &output_config);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);

        tmr_base_init(TMR2, 0xFFFFU, 0U);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_interrupt_enable(TMR2, TMR_C1_INT | TMR_C2_INT, TRUE);
        nvic_irq_enable(TMR2_GLOBAL_IRQn, 0U, 0U);
    }

    void stepper_init_step_gpio(void)
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

        gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_default_para_init(&gpio_init_struct);

        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_pins = GPIO_PINS_15;
        gpio_init(GPIOA, &gpio_init_struct);

        gpio_init_struct.gpio_pins = GPIO_PINS_3;
        gpio_init(GPIOB, &gpio_init_struct);

        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = EN_OUT_PIN;
        gpio_init(EN_OUTPUT_PORT, &gpio_init_struct);

        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pins = STEP_OUT_PIN;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

        gpio_init_struct.gpio_pins = DIR_OUT_PIN;
        gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);

        stepper_init_capture_ring_buffer();
        stepper_init_tmr2_capture_and_oc();
        stepper_init_motion_timer();
        stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, true);
        stepper_set_direction(false);
        stepper_set_step_state(false);
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

extern "C" void TMR2_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR2, TMR_C1_FLAG) != RESET)
    {
        tmr_flag_clear(TMR2, TMR_C1_FLAG);
        const uint32_t capture_tick = DWT->CYCCNT;
        const bool dir_state = gpio_input_data_bit_read(GPIOA, GPIO_PINS_15) != 0U;
        stepper_common::stepper_push_capture_event(capture_tick, false, dir_state);
    }

    if (tmr_interrupt_flag_get(TMR2, TMR_C2_FLAG) != RESET)
    {
        tmr_flag_clear(TMR2, TMR_C2_FLAG);
        const uint32_t capture_tick = DWT->CYCCNT;
        const bool dir_state = gpio_input_data_bit_read(GPIOA, GPIO_PINS_15) != 0U;
        stepper_common::stepper_push_capture_event(capture_tick, true, dir_state);
    }
}
