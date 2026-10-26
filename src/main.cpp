/**
  ******************************************************************************
  * @file     main.cpp
  * @brief    main program
  ******************************************************************************
  */

#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_core.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include "usbd_int.h"

#include "closed_loop_controller.h"
#include "usb_cdc_protocol.h"
#include "tmc2209_driver.h"
#include "kth7823_encoder.h"
#include "phase_current.h"
#include "pulse_capture.h"

TaskHandle_t led5_handler;
TaskHandle_t control_handler;
TaskHandle_t telemetry_handler;
TaskHandle_t usb_handler;
TaskHandle_t current_adc_handler;

static ClosedLoopController g_controller;
static Tmc2209Driver g_driver;
static Tmc2209ProtocolAdapter g_protocol;
static Kth7823Encoder g_encoder;
static UsbCdcProtocolBridge g_usb_bridge;
static PhaseCurrentMonitor g_phase_current;
static usbd_core_type g_usb_core;

void led5_task_function(void* pvParameters);
void control_task_function(void* pvParameters);
void telemetry_task_function(void* pvParameters);
void usb_task_function(void* pvParameters);
void current_adc_task_function(void* pvParameters);

// 磁编码器采样时基：TMR5 溢出。
// 注意：TMR3 已被 pulse_capture_init() 占用成自由运行捕获（ARR=0xFFFF、不能有周期中断），
// 所以这个周期性时基不能再放 TMR3。
static void encoder_timer_init(void)
{
    constexpr uint32_t k_encoder_sample_us = 2U;
    // 250mhz下每ns 4 ticks，1us就是4000 ticks
    constexpr uint32_t k_tick_per_us = 4000U;  // 每us的tick数，clock那里设置了分频为2，导致apb的定时器频率等于主频（at32特殊，看文档17页）
    constexpr uint32_t k_arr_value = (k_encoder_sample_us * k_tick_per_us) - 1U;

    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR5, static_cast<uint16_t>(k_arr_value), 0U);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR5, TMR_CLOCK_DIV2);
    tmr_period_buffer_enable(TMR5, TRUE);
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR5_GLOBAL_IRQn, 2U, 0U);
    tmr_counter_value_set(TMR5, 0U);
    tmr_counter_enable(TMR5, TRUE);
}

static void control_timer_init(void)
{
    crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR4, 1000U - 1U, system_core_clock / 20000000U - 1U);
    tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR4, TMR_CLOCK_DIV2);

    tmr_output_config_type output_config;
    tmr_output_default_para_init(&output_config);
    output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    output_config.oc_output_state = FALSE;
    output_config.occ_output_state = FALSE;
    output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    output_config.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_4, &output_config);
    tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, 0);
    tmr_channel_enable(TMR4, TMR_SELECT_CHANNEL_4, TRUE);

    tmr_interrupt_enable(TMR4, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR4_GLOBAL_IRQn, 1U, 0U);
    tmr_counter_enable(TMR4, TRUE);
}

static void usb_device_init(void)
{
    /* Use the calibrated 8 MHz HICK as the USB clock source. This keeps USB at
       48 MHz.
     虽然官方说the maximum frequency of APB1/APB2 clock is 120 MHz，实测主频250分频到125Mhz也能跑
    */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

    nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 0, 0);
    usbd_core_init(&g_usb_core, USB, &cdc_class_handler, &cdc_desc_handler, 0);
    usbd_connect(&g_usb_core);
}

extern "C" void USBFS_L_CAN1_RX0_IRQHandler(void)
{
    usbd_irq_handler(&g_usb_core);
}

extern "C" void TMR4_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR4, TMR_OVF_FLAG) != RESET)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        tmr_flag_clear(TMR4, TMR_OVF_FLAG);
        if (control_handler != NULL)
        {
            vTaskNotifyGiveFromISR(control_handler, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

extern "C" void TMR5_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR5, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR5, TMR_OVF_FLAG);
        g_encoder.updateFilteredSample();
    }
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (dma_interrupt_flag_get(DMA1_HDT1_FLAG) != RESET)
    {
        dma_flag_clear(DMA1_HDT1_FLAG);
        if (current_adc_handler != NULL)
        {
            xTaskNotifyFromISR(current_adc_handler, PhaseCurrentMonitor::kDmaNotifyFirstHalf,
                               eSetValueWithOverwrite, &higher_priority_task_woken);
        }
    }
    if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET)
    {
        dma_flag_clear(DMA1_FDT1_FLAG);
        if (current_adc_handler != NULL)
        {
            xTaskNotifyFromISR(current_adc_handler, PhaseCurrentMonitor::kDmaNotifySecondHalf,
                               eSetValueWithOverwrite, &higher_priority_task_woken);
        }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void usb_delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

extern "C" void usb_delay_us(uint32_t us)
{
    delay_us(us);
}

extern "C" void usb_usart_config(linecoding_type linecoding)
{
    /* The USB VCP is the application transport; no separate USART is driven by
       CDC line-coding requests, but the callback is required by the vendor class. */
    (void)linecoding;
}

int main(void)
{
    // 开启Cortex‑M4 FPU
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
    __asm volatile ("DSB");
    __asm volatile ("ISB");

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();

    at32_board_init();
    uart_print_init(115200);
    usb_device_init();

    g_protocol.attachDriver(&g_driver);
    g_protocol.configure(
        {32U, 256U, 0.8f, 0.2f, true, true, 0.110f, true, TMC2209_UART_GPIO, TMC2209_UART_PIN, 115200U});
    g_encoder.init();
    crm_clocks_freq_type clocks;
    auto apb1_freq = clocks.apb1_freq;
    auto apb2_freq = clocks.apb2_freq;
    printf("CPU Clock: %uHZ, APB1 Clock: %luHZ, APB2 Clock: %luHZ\r\n", system_core_clock, apb1_freq, apb2_freq);
    printf("KTH7823 init: tx=0x%04X raw=0x%04X MISO=%u MGH=%u MGL=%u\r\n",
           g_encoder.lastTxFrame(), g_encoder.lastRawFrame(), g_encoder.misoLevel(),
           g_encoder.magneticFieldHigh() ? 1U : 0U,
           g_encoder.magneticFieldLow() ? 1U : 0U);
    g_controller.setProtocol(&g_protocol);
    g_controller.setFaultPolicy(true, true);
    g_controller.enableLoopStats(true);
    g_controller.init(&g_driver, &g_encoder);
    // 外部 PUL/DIR 指令采集（TMR3 CH1/CH2 = PB4/PB5 捕获 + DMA 环形缓冲）。
    // 必须在控制器 init 之后：init 里会把这两个脚初始化成普通 GPIO。
    pulse_capture_init();
    g_usb_bridge.init(&g_controller, &g_usb_core);
    control_timer_init();
    encoder_timer_init();
    get_hw_time_ns();

    taskENTER_CRITICAL();

    if (xTaskCreate((TaskFunction_t)led5_task_function,
                    (const char*)"LED5_task",
                    (uint16_t)256,
                    (void*)NULL,
                    (UBaseType_t)2,
                    (TaskHandle_t*)&led5_handler) != pdPASS)
    {
        printf("LED5 task could not be created as there was insufficient heap memory remaining.\r\n");
    }

    if (xTaskCreate((TaskFunction_t)control_task_function,
                    (const char*)"Control_task",
                    (uint16_t)512,
                    (void*)NULL,
                    (UBaseType_t)3,
                    (TaskHandle_t*)&control_handler) != pdPASS)
    {
        printf("Control task could not be created as there was insufficient heap memory remaining.\r\n");
    }
    if (xTaskCreate((TaskFunction_t)telemetry_task_function,
                    (const char*)"Telemetry_task",
                    (uint16_t)384,
                    (void*)NULL,
                    (UBaseType_t)2,
                    (TaskHandle_t*)&telemetry_handler) != pdPASS)
    {
        printf("Telemetry task could not be created as there was insufficient heap memory remaining.\r\n");
    }

    if (xTaskCreate((TaskFunction_t)usb_task_function,
                    (const char*)"USB_task",
                    (uint16_t)256,
                    (void*)NULL,
                    (UBaseType_t)2,
                    (TaskHandle_t*)&usb_handler) != pdPASS)
    {
        printf("USB task could not be created as there was insufficient heap memory remaining.\r\n");
    }

    if (xTaskCreate((TaskFunction_t)current_adc_task_function,
                    (const char*)"Current_ADC_task",
                    (uint16_t)256,
                    (void*)NULL,
                    (UBaseType_t)2,
                    (TaskHandle_t*)&current_adc_handler) != pdPASS)
    {
        printf("Current ADC task could not be created as there was insufficient heap memory remaining.\r\n");
    }

    taskEXIT_CRITICAL();
    vTaskStartScheduler();
}

void led5_task_function(void* pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        at32_led_toggle(LED5);
        vTaskDelay(1000);
    }
}

void control_task_function(void* pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        // 等待TMR4定时器ISR通知，50μs唤醒一次（20kHz）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 读取真实硬件纳秒时间戳，传给rampUpdate
        uint64_t now_ns = get_hw_time_ns();
        g_controller.rampUpdate(now_ns);
    }
}

void telemetry_task_function(void* pvParameters)
{
    (void)pvParameters;
    TickType_t last_log_tick = xTaskGetTickCount();
    TickType_t last_usage_tick = last_log_tick;
    uint32_t previous_cycle_count = DWT->CYCCNT;
    uint32_t previous_idle_runtime = 0U;
    uint32_t previous_control_runtime = 0U;
    bool usage_baseline_valid = false;

    while (1)
    {
        const TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_usage_tick) >= pdMS_TO_TICKS(500))
        {
            TaskStatus_t idle_status;
            TaskStatus_t control_status;
            vTaskGetInfo(xTaskGetIdleTaskHandle(), &idle_status, pdTRUE, eInvalid);
            vTaskGetInfo(control_handler, &control_status, pdTRUE, eInvalid);

            const uint32_t current_cycle_count = DWT->CYCCNT;
            const uint32_t current_idle_runtime = static_cast<uint32_t>(idle_status.ulRunTimeCounter);
            const uint32_t current_control_runtime = static_cast<uint32_t>(control_status.ulRunTimeCounter);
            if (usage_baseline_valid)
            {
                const uint32_t elapsed_cycles = current_cycle_count - previous_cycle_count;
                const uint32_t idle_cycles = current_idle_runtime - previous_idle_runtime;
                const uint32_t control_cycles = current_control_runtime - previous_control_runtime;
                if (elapsed_cycles > 0U)
                {
                    const uint64_t cpu_usage = idle_cycles < elapsed_cycles
                        ? (static_cast<uint64_t>(elapsed_cycles - idle_cycles) * 10000ULL) / elapsed_cycles
                        : 0ULL;
                    const uint64_t control_usage =
                        (static_cast<uint64_t>(control_cycles) * 10000ULL) / elapsed_cycles;
                    g_controller.setCpuUsageTelemetry(
                        static_cast<uint16_t>(cpu_usage > 10000ULL ? 10000ULL : cpu_usage),
                        static_cast<uint16_t>(control_usage > 10000ULL ? 10000ULL : control_usage));
                }
            }
            previous_cycle_count = current_cycle_count;
            previous_idle_runtime = current_idle_runtime;
            previous_control_runtime = current_control_runtime;
            last_usage_tick = now_tick;
            usage_baseline_valid = true;
        }

        g_usb_bridge.sendTelemetry();
        // if ((xTaskGetTickCount() - last_log_tick) >= pdMS_TO_TICKS(2000))
        // {
        //     const float angle_mdeg = g_encoder.lastFrameAngle();
        //     printf("KTH7823: tx=0x%04X raw=0x%04X angle=%.3f, filter_angle=%.3f MISO=%u MGH=%u MGL=%u reads=%lu ff=%lu 00=%lu\r\n",
        //            g_encoder.lastTxFrame(), g_encoder.lastRawFrame(),
        //            angle_mdeg, g_encoder.readFilteredAngle(),
        //            g_encoder.misoLevel(), g_encoder.magneticFieldHigh() ? 1U : 0U,
        //            g_encoder.magneticFieldLow() ? 1U : 0U,
        //            static_cast<unsigned long>(g_encoder.readCount()),
        //            static_cast<unsigned long>(g_encoder.allOnesCount()),
        //            static_cast<unsigned long>(g_encoder.allZerosCount()));
        //     last_log_tick = xTaskGetTickCount();
        // }
        vTaskDelay(5);
    }
}

void usb_task_function(void* pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        g_usb_bridge.poll();
        vTaskDelay(1);
    }
}

void current_adc_task_function(void* pvParameters)
{
    (void)pvParameters;
    if (!g_phase_current.init())
    {
        printf("[CURRENT_ADC] ADC calibration timed out; sampling disabled\r\n");
        vTaskSuspend(NULL);
    }

    uint32_t notification_value = 0U;

    while (1)
    {
        if (xTaskNotifyWait(0U, 0xFFFFFFFFU, &notification_value, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        g_phase_current.handleDmaNotification(notification_value);
        g_controller.setPhaseCurrentTelemetry(
            g_phase_current.phaseCurrentA(),
            g_phase_current.phaseCurrentB());
    }
}
