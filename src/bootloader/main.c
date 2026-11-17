/**
 * main.c —— taotieFOC 自定义 Bootloader。
 *
 * 目标（用户原话）：USB 进自定义 DFU 刷机，不再手动按键；BL 区不能被 DFU 覆写；
 * 刷到一半断线下次还能重来；最终实现改代码 → 刷机 → 测试全自动。
 *
 * 启动决策（每次复位都走一遍，全部只用"读一下就能判"的信息，没有任何会卡的等待）：
 *
 *   1. RAM 交接页 request == BL_HSK_REQ_DFU   → 留在 DFU（应用 0x19A 主动请求进 BL）
 *   2. 交接页 boot_attempt >= 3               → 留在 DFU（应用连续启不来，不再无限重启）
 *   3. 应用镜像尾签 + CRC + 向量表三重校验失败 → 留在 DFU（没刷过 / 刷到一半掉电）
 *   4. 否则                                   → boot_attempt++ 后跳应用
 *
 * 1/2/3 都不满足就跳应用，跳转本身只有几微秒，上电体验和没有 BL 时一样。
 *
 * 为什么这套设计"刷不死"：
 *   - BL 区 8KB：DFU 的每一次擦/写都先过 bl_addr_in_app()，BL 区在允许区间之外；
 *     可选再上 EPP 硬件写保护（bit0/bit1），那时连 BL 自己都擦不掉。
 *   - 有效镜像的判据是"尾签 + 全镜像 CRC"，尾签只在主机把整包传完、BL 校验通过之后
 *     才存在；下载中途断电 → CRC 不符 → 停在 DFU，重新下发即可，不需要任何"续传"。
 *   - BL 从不假设上一次刷到哪儿：每次下载第一块都先整片擦，写完由主机 --verify 回读比对。
 */
#include "at32f403a_407.h"

#include "usbd_int.h"

#include "bl_desc.h"
#include "bl_dfu.h"
#include "bl_flash.h"
#include "bl_layout.h"

/* ---------------- 时钟 ---------------- */
/* BL 跑在复位默认的 HICK 8MHz，不起 HEXT、不开 PLL —— 这不是偷懒，是三笔账：
   1) USB 全速的 48MHz 来自**校准后的 HICK**，和 sysclk 完全无关（和应用
      main.cpp:167 同一条路线），所以主频高低对刷写速度几乎没影响：擦除和编程的耗时
      由 Flash 硬件决定，不由 CPU 决定。
   2) 少掉 crm_pll_config / HEXT 起振等待这一大坨代码，BL 才塞得进 8KB —— 而 8KB 是
      应用还能有 6.6KB 生长空间的前提。
   3) BL 不依赖外部晶振：晶振坏了、PLL 配置错了，BL 照样能枚举成 DFU 把新应用刷进去。
      这是"刷不死"的一部分。应用自己会把它那套 250MHz 配好，BL 不管。 */
/* 复位后 sysclk = HICK，AHB/APB 分频都是 1，所以就是 8MHz。直接写死而不调
   system_core_clock_update()：后者会拉进整个 crm_clocks_freq_get()（好几百字节），
   而 BL 只把这个数用来算 DWT 延时 —— 宁可要一个"差一点也无所谓"的常数，
   也不要为了精确多占 8KB 预算里 5% 的空间。应用自己会算准确的。 */
static void bl_clock_config(void)
{
    system_core_clock = 8000000U;
}

/* ---------------- DWT 延时 ---------------- */
/* 不用 SysTick：SysTick 会被应用/RTOS 重新配置，BL 用 DWT->CYCCNT 更省事，
   也避免跳转前忘了关掉它导致应用一进来就吃到一个残留中断。 */
static inline uint32_t bl_cycles_per_us(void)
{
    return (system_core_clock / 1000000U) + ((system_core_clock % 1000000U) != 0U ? 1U : 0U);
}

static void bl_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void usb_delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t need = us * bl_cycles_per_us();
    while ((DWT->CYCCNT - start) < need)
    {
    }
}

void usb_delay_ms(uint32_t ms)
{
    while (ms--)
    {
        usb_delay_us(1000U);
    }
}

/* ---------------- USB ---------------- */
static usbd_core_type g_bl_usb;

void USBFS_L_CAN1_RX0_IRQHandler(void)
{
    usbd_irq_handler(&g_bl_usb);
}

static void bl_usb_start(void)
{
    /* 和应用 main.cpp:167 完全一致的 USB 时钟路线：校准 HICK 当 48MHz 源。
       这条路线已经被应用的 CDC 长跑验证过，BL 不去发明第二套。 */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

    nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 0U, 0U);
    usbd_core_init(&g_bl_usb, USB, &dfu_class_handler, &dfu_desc_handler, 0);
    usbd_connect(&g_bl_usb);
}

/* ---------------- 启动 ---------------- */

int main(void)
{
    uint32_t payload_len = 0U;
    uint32_t diag = 0U;
    bl_hsk_t h;

    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
    __DSB();
    __ISB();

    bl_clock_config();
    bl_dwt_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    if (!bl_hsk_valid())
    {
        bl_hsk_init_fresh();        /* 冷上电随机值：原地初始化成"干净"状态 */
    }
    h = *bl_hsk();

    const int want_dfu = (h.request == BL_HSK_REQ_DFU) ? 1 : 0;
    const int stuck = (h.boot_attempt >= BL_MAX_BOOT_ATTEMPT) ? 1 : 0;
    const int app_ok = bl_app_valid(&payload_len, &diag);

    if (app_ok && !want_dfu && !stuck)
    {
        h.boot_attempt += 1U;
        h.status = 0x4A4D0000U | (diag & 0xFFu);      /* 'JM' = jump */
        bl_hsk_store(&h);
        bl_jump_to_app();
        /* 只有向量表不合理才会走到这里；落到下面的 DFU 分支，主机能看到我们 */
        h = *bl_hsk();
        h.status = 0x4A460000U;                        /* 'JF' = jump failed */
        bl_hsk_store(&h);
    }
    else
    {
        /* 请求是一次性的：进 DFU 就消费掉，避免之后任何复位又被它拉回 BL */
        h.request = BL_HSK_REQ_NONE;
        h.status = 0x44460000U                        /* 'DF' = DFU，低 8 位是原因 */
                   | (want_dfu ? 0x01u : (stuck ? 0x02u : (diag & 0xFFu)));
        bl_hsk_store(&h);
    }

    bl_usb_start();

    for (;;)
    {
        bl_dfu_poll_work();

        if (g_dfu.manifest_done)
        {
            /* 镜像已校验通过（CRC 在 bl_dfu_poll_work 里算过）。什么时候复位？
               "主机连续 2.5 秒没有再发任何请求"之后 —— 这样上位机可以从容地用
               DFU_UPLOAD 把 104KB 读回去逐字节比对，比对完它自己就不来了，2.5s 后
               BL 安静地复位进新应用。定长延时（400ms）会把这场回读抢掉。
               DWT 计数在 8MHz 下 537 秒回绕一次，无符号减法天然抗回绕。 */
            const uint32_t idle_cyc = DWT->CYCCNT - g_dfu_last_req_cyc;
            const uint32_t need_cyc = 2500U * (system_core_clock / 1000U);
            if (idle_cyc >= need_cyc)
            {
                h = *bl_hsk();
                h.request = BL_HSK_REQ_NONE;
                h.boot_attempt = 0U;
                bl_hsk_store(&h);
                NVIC_SystemReset();
            }
        }
    }
}
