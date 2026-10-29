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
    // 【采样周期是 32us，不是 2us】
    // 旧代码 k_tick_per_us = 4000 把数量级算反了：250MHz 下 1 tick = 4ns，
    // 1us 只有 250 tick，不是 4000 tick。按旧值算出的 ARR = 2*4000-1 = 7999
    // 实际周期是 8000 * 4ns = 32us（31.25kHz），注释里写的 2us 差了 16 倍。
    // 现在按真实主频推导，标注和数值终于对得上：32us * 250 = 8000 - 1 = 7999，
    // 寄存器值和旧版完全一样，行为不变，只是把真相写清楚。
    //
    // 为什么不真的改成 2us：SPI2 @ 3.9MHz 读一次 16bit 就要 4.1us，比 2us 还长，
    // 中断会自己把自己压死。31.25kHz 对 20kHz 控制环（每拍 1.5 个新样本）刚好合适。
    // 依赖“采样周期”这个数字的注释（滤波器窗口时间常数等）都该按 32us 重算。
    constexpr uint32_t k_encoder_sample_us = 32U;
    const uint32_t ticks_per_us = system_core_clock / 1000000U;
    const uint32_t k_arr_value = (k_encoder_sample_us * ticks_per_us) - 1U;

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
    // 【精确 20kHz，不是 20.83kHz】
    // 旧写法 system_core_clock / 20000000U - 1 = 250e6/20e6 - 1 = 12 - 1 = 11：
    // 整数除法把 12.5 截断成了 12，计数频率变成 250MHz/12 = 20.833MHz，
    // 配 ARR=999 得到的周期是 1000/20.833MHz = 48us，不是 50us。
    //
    // 现在改成两级整除：先把计数频率降到 10MHz（250MHz/25），再用 ARR=499 分频，
    // 得到 500 / 10MHz = 50.000us 精确 20kHz。主频换成 240MHz 也能自适应
    // （240MHz/24 = 10MHz）。
    const uint32_t counter_hz = 10000000U;                                  // 目标计数频率 10MHz
    const uint32_t control_hz = 20000U;                                     // 目标控制环 20kHz
    const uint32_t psc = (system_core_clock + counter_hz / 2U) / counter_hz - 1U;
    const uint32_t arr = counter_hz / control_hz - 1U;                      // 499
    tmr_base_init(TMR4, static_cast<uint16_t>(arr), static_cast<uint16_t>(psc));
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

// ===================== 【FreeRTOS 栈溢出回调】=====================
// configCHECK_FOR_STACK_OVERFLOW = 2 时，每次任务切换会检查栈尾 16 字节的填充图样，
// 被踩就调进这里。
// 注意：这里不能用 printf —— 栈已经溢出，再调一个吃 300+ 字节栈的浮点格式化函数，
// 很可能直接跳进 HardFault，反而什么都看不到。
// 所以只保存现场指针（调试器可以直接看 g_stack_overflow_task_name），
// 然后用 GPIO 忙等闪灯，现场肉眼可见“死在闪灯循环上”。
volatile const char* g_stack_overflow_task_name = nullptr;

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    g_stack_overflow_task_name = pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        at32_led_toggle(LED5);
        // 不能用 vTaskDelay（调度器已经不能用了），纯忙等做视觉上的“快闪”
        for (volatile uint32_t spin = 0U; spin < 400000U; ++spin)
        {
        }
    }
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
    // 旧代码先声明 crm_clocks_freq_type clocks; 就直接读 clocks.apb1_freq，
    // 从来没调过 crm_clocks_freq_get()，打印出来的是栈上的垃圾值。
    // 这类“未初始化就拿来打印”的写法不会报错，只会让人对着假时钟数找问题。
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    printf("CPU Clock: %luHZ, APB1 Clock: %luHZ, APB2 Clock: %luHZ\r\n",
           static_cast<unsigned long>(system_core_clock),
           static_cast<unsigned long>(clocks.apb1_freq),
           static_cast<unsigned long>(clocks.apb2_freq));
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
#if PULSE_CAPTURE_SELFCHECK_LOG
    TickType_t last_pulse_log_tick = last_log_tick;
    uint32_t pulse_sample_count = 0U; // 一秒内的采样点数（约 200）
    uint32_t pulse_high_count = 0U;   // 其中 PUL 被判为高的点数
#endif
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

#if PULSE_CAPTURE_SELFCHECK_LOG
        // 每次遥测循环（约 5ms，一秒约 200 点）对 PUL 引脚采样一次，统计“高电平占比”。
        // 这一路只读 IDT，不经过捕获/DMA，所以能把“线上没跳变”和“捕获没工作”分开：
        //   hi≈100% 且 cap 不涨 -> 高电平降不下来（源极跟随没有下拉 / 负载容太大 / 根本没在发数据）
        //   hi≈0%   且 cap 不涨 -> 高电平从没被 MCU 认成高（VIH 裕量不足，2.5V 就很危险）
        //   hi 在中间且有波动   -> 引脚确实在跳变，此时 cap 不涨才是捕获侧的问题
        ++pulse_sample_count;
        if (pulse_capture_get_pul_level() != 0U)
        {
            ++pulse_high_count;
        }

        // 每秒一行 PUL/DIR 自检，用来和 scripts/uart_pulse_gen.py 打印的“脉冲数”对拍。
        // 判读：
        //   hi≈100%、cap 不涨    -> 线上没跳变（UART 空闲恒高）或高电平降不下来
        //   hi≈0%、cap 不涨      -> 高电平没被 MCU 认成高（VIH 裕量不足/没共地/源极无下拉）
        //   hi 在中间、cap 不涨    -> 引脚在跳变但捕获没记到（不该出现，出现就是定时器/DMA 问题）
        //   cap 在涨但 deg 在 0 附近来回 -> DIR 悬空，逐脉冲快照随机，正负抵消
        //   pd 一直不为 0             -> 积压，rampUpdate 消费不过来（>256 就会丢脉冲）
        if ((xTaskGetTickCount() - last_pulse_log_tick) >= pdMS_TO_TICKS(1000))
        {
            uint8_t pul_level = 0U;
            uint8_t dir_level = 0U;
            uint16_t dma_pending = 0U;
            pulse_capture_get_probe(&pul_level, &dir_level, &dma_pending);
            const int64_t total = pulse_capture_get_total();
            const uint32_t pul_high_pct = pulse_sample_count != 0U
                ? (pulse_high_count * 100U) / pulse_sample_count
                : 0U;
            // 角度口径：1 个外部 PUL 脉冲 = 1 个驱动器微步，一圈脉冲数随细分变化，
            // 必须问控制器要实时值（200 步/圈 x 当前微步），不能再写死某个细分。
            const float steps_per_round = static_cast<float>(g_controller.getStepsPerRound());
            const double total_deg = steps_per_round > 0.0f
                ? static_cast<double>(total) * 360.0 / static_cast<double>(steps_per_round)
                : 0.0;
            printf("[PUL] cap=%lu deg=%.3f spr=%.0f PUL=%u hi=%lu%% DIR=%u pd=%u ovf=%u\r\n",
                   static_cast<unsigned long>(pulse_capture_get_captured_count()), total_deg,
                   static_cast<double>(steps_per_round),
                   pul_level, static_cast<unsigned long>(pul_high_pct), dir_level, dma_pending,
                   pulse_buf_overflow_flag ? 1U : 0U);
            if (g_controller.isExternalPulseFollowEnabled())
            {
                // rev = 窗内发生过换向的窗口数。一个 50us 窗口只能发一个方向，
                // 这种窗口里"先正走后反走"的脉冲被净和压平（终点位置仍对，中间往返不执行）。
                // 持续增长就说明外部信号频率已经高到一窗内会换向，要上段队列才能保轨迹。
                printf("[FOL] cmd=%ld out=%ld backlog=%lu peak=%lu drop=%lu rev=%lu\r\n",
                       static_cast<long>(g_controller.getExternalCmdSteps()),
                       static_cast<long>(g_controller.getExternalEmittedSteps()),
                       static_cast<unsigned long>(g_controller.getExternalBacklogSteps()),
                       static_cast<unsigned long>(g_controller.getExternalPeakBacklogSteps()),
                       static_cast<unsigned long>(g_controller.getExternalDroppedSteps()),
                       static_cast<unsigned long>(g_controller.getExternalWindowReversals()));
            }
            pulse_sample_count = 0U;
            pulse_high_count = 0U;
            last_pulse_log_tick = xTaskGetTickCount();
        }
#endif

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
