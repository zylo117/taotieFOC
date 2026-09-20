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

TaskHandle_t led5_handler;
TaskHandle_t control_handler;
TaskHandle_t telemetry_handler;
TaskHandle_t usb_handler;

static ClosedLoopController g_controller;
static Tmc2209Driver g_driver;
static Tmc2209ProtocolAdapter g_protocol;
static Kth7823Encoder g_encoder;
static UsbCdcProtocolBridge g_usb_bridge;
static usbd_core_type g_usb_core;

void led5_task_function(void* pvParameters);
void control_task_function(void* pvParameters);
void telemetry_task_function(void* pvParameters);
void usb_task_function(void* pvParameters);


static void control_timer_init(void)
{
    crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR4, 1000U - 1U, system_core_clock / 20000000U - 1U);
    tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR4, TMR_CLOCK_DIV1);
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


// 全局统计变量，ISR写，任务读
volatile uint64_t g_max_isr_cost_ns = 0;
volatile uint64_t g_last_isr_cost_ns = 0;
volatile uint32_t g_isr_overrun_cnt = 0;
const uint64_t ISR_COST_WARN_NS = 25000ULL;  // 25us警告超时
extern "C" void TMR4_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR4, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR4, TMR_OVF_FLAG);

        uint64_t isr_start_ns = get_hw_time_ns();

        // ============【可选】全局关中断，完全隔绝所有中断，保证ns时序不被打断 ============
        // 警告：关中断总时间必须 <25us！否则USB、串口、其它外设中断会积压出错
        // __disable_irq();

        uint64_t now_ns = get_hw_time_ns();
        g_controller.consumeQueuedStepDirEvents();
        // ✅闭环rampUpdate（包含内部delay_ns自旋ns延时），运行在中断上下文
        g_controller.rampUpdate(now_ns);

        // 如果开启了全局关中断，务必配对打开
        // __enable_irq();

        uint64_t isr_end_ns = get_hw_time_ns();
        uint64_t exec_cost_ns = isr_end_ns - isr_start_ns;

        // 耗时超限告警，阈值设置25000ns =25us，小于50us定时器周期留安全余量
        UBaseType_t isr_mask;
        isr_mask = portSET_INTERRUPT_MASK_FROM_ISR();

        g_last_isr_cost_ns = exec_cost_ns;
        if(exec_cost_ns > g_max_isr_cost_ns)
        {
            g_max_isr_cost_ns = exec_cost_ns;
        }
        if(exec_cost_ns > ISR_COST_WARN_NS)
        {
            g_isr_overrun_cnt ++;
        }

        portCLEAR_INTERRUPT_MASK_FROM_ISR(isr_mask);

        // 通知control_task任务：闭环计算已经完成，任务做非实时后处理，不跑时序
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (control_handler != NULL)
        {
            vTaskNotifyGiveFromISR(control_handler, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
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
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();

    at32_board_init();
    uart_print_init(115200);
    usb_device_init();

    g_protocol.attachDriver(&g_driver);
    g_protocol.configure(
        {32U, 256U, 0.8f, 0.2f, true, true, 0.110f, true, TMC2209_UART_GPIO, TMC2209_UART_PIN, 115200U});
    g_encoder.init();
    printf("KTH7823 init: tx=0x%04X raw=0x%04X MISO=%u MGH=%u MGL=%u\r\n",
           g_encoder.lastTxFrame(), g_encoder.lastRawFrame(), g_encoder.misoLevel(),
           g_encoder.magneticFieldHigh() ? 1U : 0U,
           g_encoder.magneticFieldLow() ? 1U : 0U);
    g_controller.setProtocol(&g_protocol);
    g_controller.setFaultPolicy(true, true);
    g_controller.enableLoopStats(true);
    g_controller.init(&g_driver, &g_encoder);
    g_usb_bridge.init(&g_controller, &g_usb_core);
    control_timer_init();

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
    uint64_t local_max = 0;
    uint32_t local_overrun = 0;

    while (1)
    {
        // 等待TMR4 ISR通知，50Hz*1000=20kHz唤醒
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 拷贝volatile统计值，临界区保护简短读取
        taskENTER_CRITICAL();
        uint64_t current_cost = g_last_isr_cost_ns;
        uint32_t overrun = g_isr_overrun_cnt;
        taskEXIT_CRITICAL();

        if(current_cost > local_max)
        {
            local_max = current_cost;
        }
        if(overrun != local_overrun)
        {
            local_overrun = overrun;
            printf("[WARN] TMR4 ISR OVERRUN! count=%lu, last_cost_ns=%lu, max_cost_ns=%d\r\n",
                   local_overrun, current_cost, local_max);
        }

        // 这里放闭环的非实时逻辑：状态判断、故障标记，**严禁任何delay_ns自旋**
        // g_controller.postProcess();
    }
}

void telemetry_task_function(void* pvParameters)
{
    (void)pvParameters;
    TickType_t last_log_tick = xTaskGetTickCount();

    while (1)
    {
        g_usb_bridge.sendTelemetry();
        if ((xTaskGetTickCount() - last_log_tick) >= pdMS_TO_TICKS(2000))
        {
            const uint32_t angle_mdeg = (static_cast<uint32_t>(g_encoder.lastRawFrame()) * 360000UL) / 65536UL;
            // uint64_t n = get_hw_time_ns();
            // double nf = static_cast<double>(n) / 1e9;
            // printf("Time: %.9f s\n", nf);
            printf("KTH7823: tx=0x%04X raw=0x%04X angle=%lu.%03lu MISO=%u MGH=%u MGL=%u reads=%lu ff=%lu 00=%lu\r\n",
                   g_encoder.lastTxFrame(), g_encoder.lastRawFrame(),
                   static_cast<unsigned long>(angle_mdeg / 1000UL),
                   static_cast<unsigned long>(angle_mdeg % 1000UL),
                   g_encoder.misoLevel(), g_encoder.magneticFieldHigh() ? 1U : 0U,
                   g_encoder.magneticFieldLow() ? 1U : 0U,
                   static_cast<unsigned long>(g_encoder.readCount()),
                   static_cast<unsigned long>(g_encoder.allOnesCount()),
                   static_cast<unsigned long>(g_encoder.allZerosCount()));
            last_log_tick = xTaskGetTickCount();
        }
        vTaskDelay(20);
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
