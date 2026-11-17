/**
 * bl_handshake.c —— 应用侧的 BL 交接页访问。见 bl_handshake.h 的说明。
 */
#include "bl_handshake.h"

#include "bl_layout.h"

#include "at32f403a_407.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t s_deadline_tick;   /* 0 = 没有待办的复位 */

static bl_hsk_t *hsk(void)
{
    return (bl_hsk_t *)BL_HSK_ADDR;
}

static uint32_t hsk_crc(const bl_hsk_t *h)
{
    return h->magic ^ h->request ^ h->boot_attempt ^ h->status;
}

static int hsk_valid(void)
{
    const bl_hsk_t *h = hsk();
    return (h->magic == BL_HSK_MAGIC) && (h->crc == hsk_crc(h)) ? 1 : 0;
}

static void hsk_store(bl_hsk_t *h)
{
    h->magic = BL_HSK_MAGIC;
    h->crc = hsk_crc(h);
}

void bl_hsk_confirm_boot(void)
{
    bl_hsk_t *h = hsk();
    if (!hsk_valid())
    {
        /* 冷上电、或者用 ST-Link 绕过 BL 直接刷的应用：没有 BL 记录可确认，不动它。
           绝不能在这里"顺手初始化"—— 那会把 BL 写的诊断码抹掉。 */
        return;
    }
    h->boot_attempt = 0U;
    h->request = BL_HSK_REQ_NONE;
    hsk_store(h);
}

void bl_arm_dfu_reboot(uint32_t delay_ms)
{
    if (s_deadline_tick != 0U)
    {
        return;     /* 已经排过了，别把 deadline 往后推 */
    }
    s_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(delay_ms);
    if (s_deadline_tick == 0U)
    {
        s_deadline_tick = 1U;   /* 0 是"未武装"的哨兵值，绕开 */
    }
}

void bl_poll_dfu_reboot(void)
{
    if (s_deadline_tick == 0U)
    {
        return;
    }
    if ((int32_t)(xTaskGetTickCount() - s_deadline_tick) < 0)
    {
        return;     /* tick 回绕安全的比较 */
    }

    bl_hsk_t *h = hsk();
    if (!hsk_valid())
    {
        h->boot_attempt = 0U;
        h->status = 0U;
    }
    h->request = BL_HSK_REQ_DFU;
    hsk_store(h);

    /* 复位前把中断关掉：BL 起来之前如果还吃到一个 TMR4/USB 中断，
       向量表已经不再指向我们的处理函数，直接跑飞。 */
    __disable_irq();
    NVIC_SystemReset();
}

void bl_hsk_read(uint32_t *status, uint32_t *boot_attempt)
{
    const bl_hsk_t *h = hsk();
    const int ok = hsk_valid();
    if (status != 0)
    {
        *status = ok ? h->status : 0U;
    }
    if (boot_attempt != 0)
    {
        *boot_attempt = ok ? h->boot_attempt : 0U;
    }
}
