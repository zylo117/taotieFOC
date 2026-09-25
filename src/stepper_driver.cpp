#include "stepper_driver.h"
#include "at32f403a_407_board.h"

namespace stepper_common
{
    namespace
    {
        bool motion_timer_running = false;
        bool g_hardware_timer_ready = false;
        constexpr uint64_t default_step_pulse_width_ns = 2000ULL;
        constexpr uint32_t kMaxHardwareWindowPulses = 12800U;

        StepDirCaptureEvent g_step_dir_ring[kStepDirCaptureRingDepth];
        volatile uint32_t g_ring_head = 0U;
        volatile uint32_t g_ring_tail = 0U;
        volatile uint32_t g_ring_count = 0U;
        uint32_t g_dma_window_arr[kMaxHardwareWindowPulses];
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
        uint64_t ticks = (nanoseconds * static_cast<uint64_t>(system_core_clock) + 999999999ULL) / 1000000000ULL;
        if (ticks < 1ULL)
        {
            ticks = 1ULL;
        }
        return ticks > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(ticks);
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

    static void stepper_config_direction_output(bool direction)
    {
        stepper_configure_hardware_timer2();
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_4,
            direction ? TMR_OUTPUT_CONTROL_FORCE_HIGH : TMR_OUTPUT_CONTROL_FORCE_LOW);
    }

    static void stepper_config_step_output(uint32_t pulse_ticks)
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
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
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
            stepper_send_step_pulse(default_step_pulse_width_ns);
            return;
        }

        tmr_counter_enable(TMR2, FALSE);
        tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_FORCE_LOW);
    }

    void stepper_send_step_pulse(uint64_t width_ns)
    {
        stepper_configure_hardware_timer2();

        uint32_t pulse_ticks = stepper_ns_to_ticks(width_ns < 100ULL ? 100ULL : width_ns);
        if (pulse_ticks < 2U)
        {
            pulse_ticks = 2U;
        }

        uint32_t arr_value = pulse_ticks + 1U;
        tmr_counter_enable(TMR2, FALSE);
        tmr_counter_value_set(TMR2, 0U);
        tmr_base_init(TMR2, arr_value, 0U);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        stepper_config_step_output(pulse_ticks);
        tmr_one_cycle_mode_enable(TMR2, TRUE);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);
    }

    bool stepper_plan_dma_window(uint32_t pulse_count,
                                bool direction,
                                uint32_t step_period_tick,
                                uint32_t pulse_width_tick,
                                uint32_t guard_ticks)
    {
        if (pulse_count == 0U || pulse_count > kMaxHardwareWindowPulses)
        {
            return false;
        }

        stepper_configure_hardware_timer2();
        stepper_set_direction(direction);

        uint32_t safe_guard_ticks = (guard_ticks > 1U) ? guard_ticks - 1U : 1U;
        for (uint32_t i = 0U; i < pulse_count; ++i)
        {
            g_dma_window_arr[i] = step_period_tick;
        }

        if (pulse_count > 1U)
        {
            g_dma_window_arr[pulse_count - 1U] = safe_guard_ticks;
        }
        else
        {
            g_dma_window_arr[0U] = safe_guard_ticks;
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
        dma_channel_enable(DMA1_CHANNEL2, TRUE);

        const uint32_t pulse_ticks = (pulse_width_tick > 0U) ? pulse_width_tick : 25U;
        tmr_counter_enable(TMR2, FALSE);
        tmr_counter_value_set(TMR2, 0U);
        tmr_base_init(TMR2, step_period_tick, 0U);
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
        tmr_period_buffer_enable(TMR2, TRUE);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
        tmr_output_enable(TMR2, TRUE);
        tmr_counter_enable(TMR2, TRUE);

        return true;
    }

    void stepper_init_capture_ring_buffer(void)
    {
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
        const uint32_t capture_tick = tmr_channel_value_get(TMR2, TMR_SELECT_CHANNEL_1);
        const bool dir_state = gpio_input_data_bit_read(GPIOA, GPIO_PINS_15) != 0U;
        stepper_common::stepper_push_capture_event(capture_tick, false, dir_state);
    }

    if (tmr_interrupt_flag_get(TMR2, TMR_C2_FLAG) != RESET)
    {
        tmr_flag_clear(TMR2, TMR_C2_FLAG);
        const uint32_t capture_tick = tmr_channel_value_get(TMR2, TMR_SELECT_CHANNEL_2);
        const bool dir_state = gpio_input_data_bit_read(GPIOA, GPIO_PINS_15) != 0U;
        stepper_common::stepper_push_capture_event(capture_tick, true, dir_state);
    }
}
