/**
  **************************************************************************
  * @file     at32f403a_407_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407_int.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup FreeRTOS_demo
  * @{
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
}

/* ===================== 【故障现场捕获】=====================
 * 旧实现的五个 fault handler 全是裸 while(1)：现场出问题时设备只是不动了，
 * 看不出是 HardFault、总线错误还是栈溢出，也没法定位到具体指令。
 *
 * 这里做两件事：
 *   1. 在进 C 代码前用汇编把异常栈帧指针存下来（naked 函数，因为任何 C 语句
 *      都会 PUSH 改变 MSP，再取就晚了）；
 *   2. 把 R0~R3/R12/LR/PC/xPSR 以及 SCB 的 HFSR/CFSR/MMFAR/BFAR 全部扒到全局变量，
 *      接调试器可以直接看 g_fault_pc 定位死在哪一行；
 *   3. 用 PA8(LED5) 裸寄存器忙等快闪 —— 现场没调试器时，亮着闪就是“已经挂了”，
 *      和正常待机的慢闪区分开。
 *
 * 为什么四个 fault 里只改 HardFault：不使用调试器时，MemManage/BusFault/UsageFault
 * 都会被硬件 escalate 成 HardFault，改这一个就盖住了绝大多数情况。
 * ============================ 【故障现场捕获】===================== */
volatile uint32_t *g_fault_frame = 0;
volatile uint32_t g_fault_exc_return = 0;
volatile uint32_t g_fault_r0 = 0;
volatile uint32_t g_fault_r1 = 0;
volatile uint32_t g_fault_r2 = 0;
volatile uint32_t g_fault_r3 = 0;
volatile uint32_t g_fault_r12 = 0;
volatile uint32_t g_fault_lr = 0;
volatile uint32_t g_fault_pc = 0;
volatile uint32_t g_fault_psr = 0;
volatile uint32_t g_fault_hfsr = 0;
volatile uint32_t g_fault_cfsr = 0;
volatile uint32_t g_fault_mmfar = 0;
volatile uint32_t g_fault_bfar = 0;

__attribute__((noreturn, noinline, used)) static void fault_halt(void)
{
  if (g_fault_frame != 0)
  {
    g_fault_r0  = g_fault_frame[0];
    g_fault_r1  = g_fault_frame[1];
    g_fault_r2  = g_fault_frame[2];
    g_fault_r3  = g_fault_frame[3];
    g_fault_r12 = g_fault_frame[4];
    g_fault_lr  = g_fault_frame[5];
    g_fault_pc  = g_fault_frame[6];
    g_fault_psr = g_fault_frame[7];
  }
  g_fault_hfsr  = SCB->HFSR;
  g_fault_cfsr  = SCB->CFSR;
  g_fault_mmfar = SCB->MMFAR;
  g_fault_bfar  = SCB->BFAR;

  /* 之后不能用任何库函数（栈和时钟都可能已不可信），只裸寄存器闪 PA8 */
  __asm volatile ("cpsid i");
  for (;;)
  {
    GPIOA->odt ^= GPIO_PINS_8;
    for (volatile uint32_t spin = 0U; spin < 400000U; ++spin)
    {
    }
  }
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile
  (
    "mov   r2, lr                              \n"
    "tst   r2, #4                              \n"
    "ite eq                                    \n"
    "mrseq r0, msp                             \n"
    "mrsne r0, psp                             \n"
    "movw  r1, #:lower16:g_fault_frame         \n"
    "movt  r1, #:upper16:g_fault_frame         \n"
    "str   r0, [r1]                            \n"
    "movw  r1, #:lower16:g_fault_exc_return    \n"
    "movt  r1, #:upper16:g_fault_exc_return    \n"
    "str   r2, [r1]                            \n"
    "b     fault_halt                          \n"
  );
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
    /* go to infinite loop when memory manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
    /* go to infinite loop when bus fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
    /* go to infinite loop when usage fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
//void SysTick_Handler(void)
//{
//}

/**
  * @}
  */

/**
  * @}
  */
