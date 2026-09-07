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

static ClosedLoopController g_controller;
static Tmc2209Driver g_driver;
static Tmc2209ProtocolAdapter g_protocol;
static Kth7823Encoder g_encoder;
static UsbCdcProtocolBridge g_usb_bridge;
static usbd_core_type g_usb_core;

void led5_task_function(void *pvParameters);
void control_task_function(void *pvParameters);
void telemetry_task_function(void *pvParameters);

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
  g_protocol.configure({32U, 256U, 0.8f, 0.2f, true, true, 0.110f, true, TMC2209_UART_GPIO, TMC2209_UART_PIN, 115200U});
  g_encoder.init();
  g_controller.setProtocol(&g_protocol);
  g_controller.setFaultPolicy(true, true);
  g_controller.enableLoopStats(true);
  g_controller.init(&g_driver, &g_encoder);
  g_usb_bridge.init(&g_controller, &g_usb_core);

  taskENTER_CRITICAL();

  if (xTaskCreate((TaskFunction_t)led5_task_function,
                  (const char *)"LED5_task",
                  (uint16_t)256,
                  (void *)NULL,
                  (UBaseType_t)2,
                  (TaskHandle_t *)&led5_handler) != pdPASS)
  {
    printf("LED5 task could not be created as there was insufficient heap memory remaining.\r\n");
  }

  if (xTaskCreate((TaskFunction_t)control_task_function,
                  (const char *)"Control_task",
                  (uint16_t)512,
                  (void *)NULL,
                  (UBaseType_t)3,
                  (TaskHandle_t *)&control_handler) != pdPASS)
  {
    printf("Control task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  if (xTaskCreate((TaskFunction_t)telemetry_task_function,
                  (const char *)"Telemetry_task",
                  (uint16_t)384,
                  (void *)NULL,
                  (UBaseType_t)2,
                  (TaskHandle_t *)&telemetry_handler) != pdPASS)
  {
    printf("Telemetry task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  taskEXIT_CRITICAL();
  vTaskStartScheduler();
}

void led5_task_function(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    at32_led_toggle(LED5);
    vTaskDelay(1000);
  }
}

void control_task_function(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    g_controller.syncStepDirection();
    g_controller.process((uint32_t)xTaskGetTickCount() * 1000UL);
    g_usb_bridge.poll();
    vTaskDelay(1);
  }
}

void telemetry_task_function(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    g_usb_bridge.sendTelemetry();
    vTaskDelay(20);
  }
}
