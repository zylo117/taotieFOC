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

// 定义在 at32f403a_407_board.c：USART1 收到字符串 "DFU" 时置 1。
// 刷机不靠它就不会进 bootloader，所以放在 telemetry 任务里消费（那里不是中断上下文）。
extern "C"
{
    extern volatile uint32_t g_dfu_uart_request;
    // 定义在 at32f403a_407_board.c：USART1 中断收到的协议帧字节环形缓冲
    extern uint16_t uart_protocol_read(uint8_t* dst, uint16_t max_len);
}

void led5_task_function(void* pvParameters);
void control_task_function(void* pvParameters);
void telemetry_task_function(void* pvParameters);
void usb_task_function(void* pvParameters);
void current_adc_task_function(void* pvParameters);

// 【编码器更新率诊断专用互锁】
// 诊断会在 telemetry_task 里连续调 g_encoder.readRaw()，而 TMR4 中断里的
// updateFilteredSample() 用的是同一个 SPI2。两个上下文同时操作 SPI2 会打乱
// 状态机（甚至互相等待对方清掉的标志），必须互斥。
//
// 注意不能用 taskENTER_CRITICAL 包住整段测试：那会屏蔽 TMR4 中断 4ms，直接丢 80 拍。
// 所以改成标志位：诊断期间 true，TMR4 中断看到它就跳过采样（用上一拍的旧值继续跑控制环），
// 测试结束立刻清 false。静止场景下跳过几拍没有影响（角度本来就不变）。
static volatile bool g_encoder_rate_test_active = false;

// 【Allan 简方差诊断】判断抖动是白噪声还是低频漂移。
// 原理：测不同时间尺度上的差值 RMS：
//   lag=1    → 50us  尺度
//   lag=10   → 500us
//   lag=100  → 5ms
//   lag=1000 → 50ms
// 判读：
//   各 lag 的 RMS 基本相等  → 白噪声（加大滤波窗口/拍内平均有效）
//   RMS 随 lag 明显增长    → 低频漂移（1/f），µs 级窗口没用，得用 ms 级低通
static constexpr uint16_t kAllanSamples = 2048U;  // 2048 x 50us = 102ms
static int16_t s_allan_buf[kAllanSamples];
static volatile bool s_allan_capture_active = false;
static volatile uint16_t s_allan_index = 0U;

// 【编码器采样：TMR5 高频独立采样 + TMR4 拍内取平均】
//
// 实测背景（[UPD] 诊断）：KTH7823 内部更新间隔约 13us（77kHz），
// 而原来在 TMR4 里“喷发 4 次”（14.4us）只能拿到约 1.1 个独立样本，降噪几乎为 0，
// 还白烧 29% 的 CPU（4 x 3.61us / 50us）。
//
// 新方案：
//   TMR5（13us，优先级 2）→ g_encoder.sampleRawFast()   只 SPI 读 + 存环形缓冲
//   TMR4（50us，优先级 1）→ g_encoder.takeRawAverage()   环绕安全平均（~0.5us）
//                          → g_encoder.updateFilteredSample(raw)  只滤波不碰 SPI
//
// 收益：50us 内 3.8 个独立样本 → 降噪上限 √3.8 ≈ 1.95 倍（原 1.05 倍）；
//       TMR4 中断时长从 14.4us 降到 ~2us（平均 + 滤波）；
//       非线性校准查表从“每读一次”改成“一拍一次”，省 1.56us/次。
//
// 13us 对应 ARR：TMR5 计数时钟 = 250MHz / 2 = 125MHz，13us x 125MHz = 1625 → ARR = 1624
static void encoder_sampler_timer_init(void)
{
    constexpr uint32_t kEncoderSampleIntervalUs = 13U;
    // TMR5 计数时钟固定为 PCLK/2（APB1 125MHz → 125MHz）
    const uint32_t sampler_clock_hz = system_core_clock / 2U;
    const uint32_t arr = (kEncoderSampleIntervalUs * (sampler_clock_hz / 1000000U)) - 1U;

    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR5, static_cast<uint16_t>(arr), 0U);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR5, TMR_CLOCK_DIV2);
    tmr_period_buffer_enable(TMR5, TRUE);
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, TRUE);
    // 优先级 2：低于 TMR4（1），但高于任务。ISR 只做一次 SPI 读（~2us）。
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
    // 【ADC 触发的真正开关，不能给 0】
    // PhaseCurrentMonitor 用的是 ADC12_ORDINARY_TRIG_TMR4CH4，即 TMR4 CH4 的比较事件。
    // PWM 模式 A 下 OC4REF 只在 CNT < CCR4 时为有效电平：
    //   CCR4 = 0  ->  CNT < 0 永不成立，OC4REF 恒低，没有上升沿，ADC 一次都不转换。
    //                （实测就是 [ADC] init=1 isr=0 rawA=0：校准过了，但 DMA 一次中断都没来）
    //   CCR4 = ARR/2 -> 每个周期 CNT=0 时拉高、CNT=ARR/2 时拉低，
    //                上升沿正好落在周期开头，ADC 采样点与原设计一致。
    // 保持 oc_output_state = FALSE 不变：CH4 的默认引脚会占用 PB14，
    // 而 PB14 是 KTH7823 编码器的 MISO，输出到引脚会把编码器 SPI 弄坏。
    // OC4REF 是内部参考信号，OEN=0 不影响它产生。
    tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, static_cast<uint16_t>(arr / 2U));
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

extern "C" void TMR5_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR5, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR5, TMR_OVF_FLAG);
        // 只做一次 SPI 读 + 存环形缓冲（~2us，不含校准查表）
        // 诊断期间跳过：那个诊断会独占 SPI2
        if (!g_encoder_rate_test_active)
        {
            g_encoder.sampleRawFast();
        }
    }
}

extern "C" void TMR4_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR4, TMR_OVF_FLAG) != RESET)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        tmr_flag_clear(TMR4, TMR_OVF_FLAG);

        // 【取平均后只滤波，不碰 SPI】
        // 这一段累积的所有样本（50us / 13us ≈ 3.8 个）由 takeRawAverage 环绕安全地平均，
        // 非线性校准也只在平均后查一次表。
        //
        // 编码器更新率诊断期间跳过：那个诊断会独占 SPI2。
        if (!g_encoder_rate_test_active)
        {
            const uint16_t raw = g_encoder.takeRawAverage();
            g_encoder.updateFilteredSample(raw);

            // Allan 诊断采样：每拍存一个原始值（50us 间隔）
            if (s_allan_capture_active && s_allan_index < kAllanSamples)
            {
                s_allan_buf[s_allan_index++] = static_cast<int16_t>(raw);
            }
        }

        if (control_handler != NULL)
        {
            vTaskNotifyGiveFromISR(control_handler, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
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
    // 编码器高频采样时基（TMR5，13us）——必须在内核启动前开，
    // 否则 TMR4 第一拍取平均时会因为环形缓冲为空而回退到 last_frame_raw_。
    encoder_sampler_timer_init();
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
                    // 512 words = 2KB。原来只有 1KB：poll() 里有 recv_buf[64] + handleFrame 调用链，
                    // 一旦碰上带 4 个格式化参数的 printf 就会撑爆，而 1KB 在当时看着“恰好够用”。
                    (uint16_t)512,
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
        // 刷机触发：UART 收到 "DFU" 后，在任务上下文里跳 bootloader。
        // 【已停用自动跳转】这块板子的 CH340 只接了 TX，PA10 悬空。
        // 内部 40k 上拉挡不住噪声，实测会误触发 → enter() 里 crm_reset() 把时钟降到 8MHz，
        // UART 波特率全错、日志变成空白，而且跳转失败还会卡死整个系统。
        // 现在只打印，不再自动跳。
        if (g_dfu_uart_request != 0U)
        {
            g_dfu_uart_request = 0U;
            printf("[DFU] uart trigger received (auto-jump disabled)\r\n");
        }

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
        // 自检行从每秒一行降到每 3 秒：三行日志（[PUL]+[ADC]+[FOL]）共约 300 字节，
        // 115200 下就是 26ms 的忙等。虽然 Control_task 优先级 3 高于 Telemetry_task 的 2，
        // 理论上能抢占，但实测脉冲捕获在测试开头会丢掉约 326 个（≈14.5ms 的 DMA 积压），
        // 先把打印频率降下来排除这个嫌疑；真要定位请看 [PUL] 的 wrap_drop 字段。
        if ((xTaskGetTickCount() - last_pulse_log_tick) >= pdMS_TO_TICKS(3000))
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
            printf("[PUL] cap=%lu deg=%.3f spr=%.0f PUL=%u hi=%lu%% DIR=%u pd=%u ovf=%u wrap_drop=%lu\r\n",
                   static_cast<unsigned long>(pulse_capture_get_captured_count()), total_deg,
                   static_cast<double>(steps_per_round),
                   pul_level, static_cast<unsigned long>(pul_high_pct), dir_level, dma_pending,
                   pulse_buf_overflow_flag ? 1U : 0U,
                   static_cast<unsigned long>(pulse_capture_get_dropped_on_wrap()));
            // 绕圈丢包现场：evt=绕圈次数 pend=当时未消费量 rd/wr=当时指针 cyc=当时 DWT 计数
            // 判读：pend 接近 2048 -> 真停摆；pend 很小且 wr-rd 对不上 -> 指针读错
            if (g_wrap_event_count != 0U)
            {
                printf("[WRAP] evt=%lu pend=%lu rd=%lu wr=%lu cyc=%lu\r\n",
                       static_cast<unsigned long>(g_wrap_event_count),
                       static_cast<unsigned long>(g_wrap_last_pending),
                       static_cast<unsigned long>(g_wrap_last_rd),
                       static_cast<unsigned long>(g_wrap_last_wr),
                       static_cast<unsigned long>(g_wrap_last_cyc));
            }

            // ============ 编码器内部更新率诊断 ============
            // 目的：不预设任何数值，直接实测「读到的角度多久才真变一次」。
            // 方法：连续调 readRaw()（每次 2.05us @7.8MHz），统计相邻样本里值发生变化的次数，
            //       再用 DWT 测总耗时，得到「每次读取的实际耗时」和「平均多久出现一个新值」。
            // 判读：
            //   avg_interval 接近 per_read   -> 内部基本每次都在更新（很短的时间常数）
            //   avg_interval 远大于 per_read -> 被内部滤波冻住，喷发采样拿到的都是同一个值
            //       （chip 手册型号表写 τ=0.51ms，但只用这个数字验证，不拿它当预设）
            // 注意：本测试会直接读走编码器的原始帧，这期间控制环拿到的是这些原始值，
            //       建议在电机完全静止时观察，避免影响运行中的控制。
            static TickType_t s_encoder_rate_test_tick = 0;
            // 只在完全静止时测：测试期间 TMR4 会跳过编码器采样，如果这时候有运动，
            // 控制环会拿到冻结 4ms 的角度，PID 可能因为“位置不变”而超调。
            // 转速 < 5 RPM 且没有规划运动在跑，才视为静止。
            const bool encoder_rate_test_idle =
                !g_controller.isMotionRunning() &&
                (fabsf(static_cast<float>(g_controller.getMotionSpeedRpm())) < 5.0f);
            if (encoder_rate_test_idle &&
                (xTaskGetTickCount() - s_encoder_rate_test_tick) >= pdMS_TO_TICKS(10000))
            {
                s_encoder_rate_test_tick = xTaskGetTickCount();
                // 先抢到 SPI2 的独占权，再做连续读取（详见 g_encoder_rate_test_active 的注释）
                g_encoder_rate_test_active = true;
                __DMB();
                // 置位那一刻，TMR4 中断可能正在执行 updateFilteredSample()（SPI 传输已启动）。
                // 等 2ms（40 个控制拍）让它跑完，确保之后的 SPI 完全独占。
                vTaskDelay(2);
                constexpr uint32_t kRateTestSamples = 2000U;  // 2000 x 2.05us ≈ 4.1ms
                uint32_t change_count = 0U;
                uint16_t previous_raw = 0xFFFFU;
                const uint32_t start_cycles = DWT->CYCCNT;
                for (uint32_t i = 0U; i < kRateTestSamples; ++i)
                {
                    const uint16_t raw = g_encoder.readRaw();
                    if (i > 0U && raw != previous_raw)
                    {
                        ++change_count;
                    }
                    previous_raw = raw;
                }
                const uint32_t total_cycles = DWT->CYCCNT - start_cycles;
                const float cycles_per_us = static_cast<float>(system_core_clock) / 1000000.0f;
                const float total_us = static_cast<float>(total_cycles) / cycles_per_us;
                const float per_read_us = total_us / static_cast<float>(kRateTestSamples);
                const float avg_interval_us =
                    change_count > 0U ? total_us / static_cast<float>(change_count) : 0.0f;
                // 测试结束，立刻把 SPI2 还给 TMR4 中断
                __DMB();
                g_encoder_rate_test_active = false;
                printf("[UPD] n=%lu chg=%lu total=%.0fus per_read=%.2fus avg_interval=%.1fus\r\n",
                       static_cast<unsigned long>(kRateTestSamples),
                       static_cast<unsigned long>(change_count),
                       static_cast<double>(total_us),
                       static_cast<double>(per_read_us),
                       static_cast<double>(avg_interval_us));
            }

            // ============ Allan 简方差诊断（判断白噪声 vs 低频漂移）============
            // 静止时每 30 秒跑一次，采 2048 个原始值（50us 间隔，共 102ms），
            // 统计不同时间尺度上的差值 RMS。
            static TickType_t s_allan_test_tick = 0;
            if (encoder_rate_test_idle &&
                (xTaskGetTickCount() - s_allan_test_tick) >= pdMS_TO_TICKS(30000))
            {
                s_allan_test_tick = xTaskGetTickCount();
                s_allan_index = 0U;
                __DMB();
                s_allan_capture_active = true;
                // 2048 x 50us = 102ms，留 150ms 余量
                vTaskDelay(pdMS_TO_TICKS(150));
                s_allan_capture_active = false;
                __DMB();
                const uint16_t captured = s_allan_index;

                if (captured > 100U)
                {
                    // 算某个 lag 上的差值 RMS（环绕安全）
                    auto lag_rms = [captured](uint16_t lag) -> float
                    {
                        double sum_sq = 0.0;
                        uint32_t count = 0U;
                        for (uint16_t i = 0U; i + lag < captured; ++i)
                        {
                            int32_t d = static_cast<int32_t>(s_allan_buf[i + lag]) -
                                        static_cast<int32_t>(s_allan_buf[i]);
                            while (d > 32768) d -= 65536;
                            while (d < -32768) d += 65536;
                            sum_sq += static_cast<double>(d) * static_cast<double>(d);
                            ++count;
                        }
                        return count > 0U
                            ? static_cast<float>(sqrt(sum_sq / static_cast<double>(count)))
                            : 0.0f;
                    };
                    // 1 LSB = 360/65536 = 0.00549°，换算成度方便判读
                    const float lsb_deg = 360.0f / 65536.0f;
                    const float r1 = lag_rms(1U) * lsb_deg;
                    const float r10 = lag_rms(10U) * lsb_deg;
                    const float r100 = lag_rms(100U) * lsb_deg;
                    const float r1000 = lag_rms(1000U) * lsb_deg;
                    printf("[ALLAN] n=%u lag1(50us)=%.4f lag10(0.5ms)=%.4f lag100(5ms)=%.4f lag1000(50ms)=%.4f deg\r\n",
                           static_cast<unsigned>(captured),
                           static_cast<double>(r1), static_cast<double>(r10),
                           static_cast<double>(r100), static_cast<double>(r1000));
                }
            }

            // 电流链路自检：把采集、搬运、换算三段分开看。
            //   isr 不涨           -> ADC 没触发 或 DMA 中断没使能
            //   isr 在涨但 raw=0   -> DMA 搬的是空数据（ADC 未转换 / 外设地址错）
            //   raw≈2048 但 mA≈0  -> 采样正常，INA240 输出确实在中点（无电流或未供电）
            printf("[ADC] init=%u isr=%lu rawA=%u rawB=%u mA=%.1f mB=%.1f\r\n",
                   g_phase_current.isInitialized() ? 1U : 0U,
                   static_cast<unsigned long>(g_phase_current.isrCount()),
                   static_cast<unsigned>(g_phase_current.lastRawA()),
                   static_cast<unsigned>(g_phase_current.lastRawB()),
                   static_cast<double>(g_controller.getPhaseCurrentTelemetryA() * 1000.0f),
                   static_cast<double>(g_controller.getPhaseCurrentTelemetryB() * 1000.0f));
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

    uint8_t uart_rx[32];

    while (1)
    {
        g_usb_bridge.poll();

        // 【USART1/CH340 也当协议通道】把中断入队的字节吐给同一个解析器。
        // 这样脚本只用一根 CH340 线就能写配置 + 发脉冲 + 收日志，
        // 不用再和网页抢 USB CDC。解析在任务上下文做（handleFrame 会 stopMotion/printf）。
        uint16_t n = 0U;
        while ((n = uart_protocol_read(uart_rx, sizeof(uart_rx))) > 0U)
        {
            g_usb_bridge.feedBytes(uart_rx, n);
        }

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
