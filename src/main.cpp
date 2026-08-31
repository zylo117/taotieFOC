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

#include "closed_loop_controller.h"
#include "tmc2209_driver.h"
#include "kth7823_encoder.h"

TaskHandle_t led2_handler;
TaskHandle_t led3_handler;

static ClosedLoopController g_controller;
static Tmc2209Driver g_driver;
static Kth7823Encoder g_encoder;

void led2_task_function(void *pvParameters);
void led3_task_function(void *pvParameters);

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();

  at32_led_init(LED2);
  at32_led_init(LED3);
  uart_print_init(115200);

  g_driver.init();
  g_encoder.init();
  g_controller.init(&g_driver, &g_encoder);

  taskENTER_CRITICAL();

  if (xTaskCreate((TaskFunction_t)led2_task_function,
                  (const char *)"LED2_task",
                  (uint16_t)512,
                  (void *)NULL,
                  (UBaseType_t)2,
                  (TaskHandle_t *)&led2_handler) != pdPASS)
  {
    printf("LED2 task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  else
  {
    printf("LED2 task was created successfully.\r\n");
  }

  if (xTaskCreate((TaskFunction_t)led3_task_function,
                  (const char *)"LED3_task",
                  (uint16_t)512,
                  (void *)NULL,
                  (UBaseType_t)2,
                  (TaskHandle_t *)&led3_handler) != pdPASS)
  {
    printf("LED3 task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  else
  {
    printf("LED3 task was created successfully.\r\n");
  }

  taskEXIT_CRITICAL();
  vTaskStartScheduler();
}

void led2_task_function(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    g_controller.syncStepDirection();
    g_controller.process((uint32_t)xTaskGetTickCount() * 1000UL);
    at32_led_toggle(LED2);
    vTaskDelay(1000);
  }
}

void led3_task_function(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    at32_led_toggle(LED3);
    vTaskDelay(500);
  }
}
