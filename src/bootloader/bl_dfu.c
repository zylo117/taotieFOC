/**
 * bl_dfu.c —— DFU 1.1 类实现（EP0-only，下载 + 上传 + manifest）。
 *
 * 状态机骨架（严格按 DFU 1.1 Figure 8 的必需路径实现）：
 *   dfuIDLE --DNLOAD(数据)--> dfuDNLOAD-SYNC --GETSTATUS--> dfuDNBUSY --主循环干完-->
 *   dfuDNLOAD-IDLE --DNLOAD(0长度)--> dfuMANIFEST-SYNC --GETSTATUS--> dfuMANIFEST-WAIT-RESET
 *   --BL 自己 NVIC_SystemReset--> 应用跑起来
 *
 * 关键设计：所有 Flash 擦写都在主循环里做，USB 中断只登记请求 + 回 dfuDNBUSY。
 * 擦 104KB 要几百毫秒，在中断里做会让主机控制传输超时、dfu-util 直接报 I/O error。
 */
#include "bl_dfu.h"

#include <string.h>
#include "at32f403a_407.h"
#include "bl_flash.h"

dfu_dev_t g_dfu;
volatile uint32_t g_dfu_last_req_cyc;

static usb_sts_type dfu_init_handler(void *udev);
static usb_sts_type dfu_clear_handler(void *udev);
static usb_sts_type dfu_setup_handler(void *udev, usb_setup_type *setup);
static usb_sts_type dfu_ept0_tx_handler(void *udev);
static usb_sts_type dfu_ept0_rx_handler(void *udev);
static usb_sts_type dfu_event_handler(void *udev, usbd_event_type event);

usbd_class_handler dfu_class_handler = {
    dfu_init_handler,
    dfu_clear_handler,
    dfu_setup_handler,
    dfu_ept0_tx_handler,
    dfu_ept0_rx_handler,
    0,                    /* in_handler：DFU 只用 EP0 */
    0,                    /* out_handler */
    0,                    /* sof_handler */
    dfu_event_handler,
    &g_dfu
};

/* GETSTATUS 应答 6 字节：bStatus / bwPollTimeout[3] / bState / iString */
typedef struct
{
    uint8_t  bStatus;
    uint8_t  bwPollTimeout[3];
    uint8_t  bState;
    uint8_t  iString;
} dfu_status_t;

static dfu_status_t g_dfu_status;
static uint8_t g_dfu_alt;     /* 只有一个 alt setting，恒 0 */

static void dfu_state_reset(void)
{
    g_dfu.state = (uint8_t)DFU_ST_DFU_IDLE;
    g_dfu.status = DFU_STATUS_OK;
    g_dfu.busy = 0U;
    g_dfu.need_erase = 1U;      /* 下一次下载从"先擦整片"开始 */
    g_dfu.manifest_done = 0U;
    g_dfu.block = 0U;
    g_dfu.rx_len = 0U;
}

static usb_sts_type dfu_init_handler(void *udev)
{
    (void)udev;
    dfu_state_reset();
    return USB_OK;
}

static usb_sts_type dfu_clear_handler(void *udev)
{
    (void)udev;
    dfu_state_reset();
    return USB_OK;
}

/* 总线复位/挂起：丢队列不丢已写进 Flash 的内容 —— 断线重连后从块 0 重来即可 */
static usb_sts_type dfu_event_handler(void *udev, usbd_event_type event)
{
    (void)udev;
    if (event == USBD_RESET_EVENT)
    {
        dfu_state_reset();
    }
    return USB_OK;
}

static usb_sts_type dfu_setup_handler(void *udev, usb_setup_type *setup)
{
    usbd_core_type *pudev = (usbd_core_type *)udev;

    g_dfu_last_req_cyc = DWT->CYCCNT;     /* 主机还活着的时间戳（见 bl_dfu.h） */

    switch (setup->bmRequestType & USB_REQ_TYPE_RESERVED)
    {
    case USB_REQ_TYPE_CLASS:
        switch (setup->bRequest)
        {
        case DFU_REQ_DNLOAD:
            if (setup->wLength == 0U)
            {
                /* 0 长度 = manifest：把"校验镜像"排进主循环的活 */
                if (g_dfu.busy)
                {
                    g_dfu.state = (uint8_t)DFU_ST_DNLOAD_SYNC;
                    break;
                }
                g_dfu.state = (uint8_t)DFU_ST_MANIFEST_SYNC;
                g_dfu.busy = 1U;
                g_dfu.block = 0U;
                /* wLength==0 的请求由 usbd_sdr.c:421 统一回状态阶段，这里不能再发一次 */
            }
            else
            {
                if (setup->wLength > DFU_XFER_SIZE)
                {
                    g_dfu.status = DFU_STATUS_ERR_WRITE;
                    g_dfu.state = (uint8_t)DFU_ST_ERROR;
                    usbd_ctrl_unsupport(pudev);
                    break;
                }
                g_dfu.block = setup->wValue;
                g_dfu.rx_len = setup->wLength;
                g_dfu.state = (uint8_t)DFU_ST_DNLOAD_SYNC;
                usbd_ctrl_recv(pudev, g_dfu.buf, setup->wLength);
            }
            break;

        case DFU_REQ_UPLOAD:
        {
            /* 回读校验用：只允许读应用区，读不到就回 0 长度表示"上传结束" */
            uint32_t off = (uint32_t)setup->wValue * DFU_XFER_SIZE;
            uint16_t len = setup->wLength;
            if (len > DFU_XFER_SIZE)
            {
                len = DFU_XFER_SIZE;
            }
            if ((off + DFU_XFER_SIZE) > APP_REGION_SIZE)
            {
                len = 0U;
                g_dfu.state = (uint8_t)DFU_ST_DFU_IDLE;
            }
            else
            {
                memcpy(g_dfu.tx_buf, (const void *)(APP_REGION_BASE + off), len);
                g_dfu.state = (uint8_t)DFU_ST_UPLOAD_IDLE;
            }
            usbd_ctrl_send(pudev, g_dfu.tx_buf, len);
            break;
        }

        case DFU_REQ_GETSTATUS:
            if (g_dfu.busy)
            {
                g_dfu_status.bStatus = DFU_STATUS_OK;
                g_dfu_status.bState = (uint8_t)DFU_ST_DNBUSY;
                g_dfu_status.bwPollTimeout[0] = 50U;   /* 20ms 轮询太急，50ms 够了 */
                g_dfu_status.bwPollTimeout[1] = 0U;
                g_dfu_status.bwPollTimeout[2] = 0U;
            }
            else
            {
                g_dfu_status.bStatus = g_dfu.status;
                g_dfu_status.bState = g_dfu.state;
                if (g_dfu.state == (uint8_t)DFU_ST_MANIFEST_WAIT_RESET)
                {
                    g_dfu_status.bwPollTimeout[0] = (uint8_t)(1000U & 0xFFU);
                    g_dfu_status.bwPollTimeout[1] = (uint8_t)((1000U >> 8) & 0xFFU);
                    g_dfu_status.bwPollTimeout[2] = (uint8_t)((1000U >> 16) & 0xFFU);
                }
                else
                {
                    g_dfu_status.bwPollTimeout[0] = 1U;
                    g_dfu_status.bwPollTimeout[1] = 0U;
                    g_dfu_status.bwPollTimeout[2] = 0U;
                }
            }
            g_dfu_status.iString = 0U;
            usbd_ctrl_send(pudev, (uint8_t *)&g_dfu_status, sizeof(g_dfu_status));
            break;

        case DFU_REQ_CLRSTATUS:
            if (g_dfu.state == (uint8_t)DFU_ST_ERROR)
            {
                dfu_state_reset();
            }
            g_dfu_status.bStatus = DFU_STATUS_OK;
            break;

        case DFU_REQ_GETSTATE:
            g_dfu_status.bState = g_dfu.busy ? (uint8_t)DFU_ST_DNBUSY : g_dfu.state;
            usbd_ctrl_send(pudev, &g_dfu_status.bState, 1U);
            break;

        case DFU_REQ_ABORT:
            g_dfu.busy = 0U;
            g_dfu.state = (uint8_t)DFU_ST_DFU_IDLE;
            g_dfu.status = DFU_STATUS_OK;
            break;

        default:
            usbd_ctrl_unsupport(pudev);
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (setup->bRequest)
        {
        case USB_STD_REQ_GET_INTERFACE:
            usbd_ctrl_send(pudev, &g_dfu_alt, 1U);   /* 只有一个 alt setting，恒 0 */
            break;
        case USB_STD_REQ_SET_INTERFACE:
        case USB_STD_REQ_SET_FEATURE:
        case USB_STD_REQ_CLEAR_FEATURE:
            break;
        default:
            usbd_ctrl_unsupport(pudev);
            break;
        }
        break;

    default:
        usbd_ctrl_unsupport(pudev);
        break;
    }
    return USB_OK;
}

/* DNLOAD 数据阶段收完：此刻才真正登记要写的地址，Flash 活留给主循环 */
static usb_sts_type dfu_ept0_rx_handler(void *udev)
{
    (void)udev;
    g_dfu.busy = 1U;
    return USB_OK;
}

static usb_sts_type dfu_ept0_tx_handler(void *udev)
{
    (void)udev;
    return USB_OK;
}

/* ------------------------------------------------------------------ */
/* 主循环：真正执行擦 / 写 / 校验                                        */
/* ------------------------------------------------------------------ */

static bl_err_t g_last_err;

void bl_dfu_poll_work(void)
{
    if (!g_dfu.busy)
    {
        return;
    }

    if (g_dfu.state == (uint8_t)DFU_ST_MANIFEST_SYNC)
    {
        uint32_t len = 0U;
        uint32_t st = 0U;
        const volatile uint32_t *t = (const volatile uint32_t *)APP_TRAILER_ADDR;

        g_last_err = BL_OK;
        if (bl_app_valid(&len, &st))
        {
            /* 镜像可信。若上位机在尾签 flags 里请求 EPP 加固 BL 区，现在做（幂等） */
            if ((t[3] & 0x00000001u) != 0u)
            {
                if (bl_epp_protect_bl() != BL_OK)
                {
                    /* EPP 是加固步骤，失败不该谎报成擦写错误：用 vendor 码，
                       上位机一眼能看出"镜像本身没问题，是保护位没写上" */
                    g_dfu.status = DFU_STATUS_ERR_VENDOR;
                    g_dfu.state = (uint8_t)DFU_ST_ERROR;
                    g_dfu.busy = 0U;
                    return;
                }
            }
            bl_hsk_t h;
            if (!bl_hsk_valid())
            {
                bl_hsk_init_fresh();
            }
            h = *bl_hsk();
            h.request = BL_HSK_REQ_NONE;
            h.boot_attempt = 0U;
            h.status = 0x44460000u | st;   /* 'DF' + 上次校验诊断码 */
            bl_hsk_store(&h);

            g_dfu.status = DFU_STATUS_OK;
            g_dfu.state = (uint8_t)DFU_ST_MANIFEST_WAIT_RESET;
            g_dfu.busy = 0U;
            g_dfu.manifest_done = 1U;
            g_dfu_last_req_cyc = DWT->CYCCNT;   /* 静默计时从这里开始 */
        }
        else
        {
            g_dfu.status = DFU_STATUS_ERR_VERIFY;
            g_dfu.state = (uint8_t)DFU_ST_ERROR;
            g_dfu.busy = 0U;
        }
        return;
    }

    /* DNLOAD：第一块先擦整片（擦完断电 → 镜像无效 → 下次仍停在 DFU，可重来） */
    const uint32_t addr = APP_REGION_BASE + g_dfu.block * DFU_XFER_SIZE;

    if (g_dfu.need_erase)
    {
        g_last_err = bl_app_erase_all();
        if (g_last_err != BL_OK)
        {
            g_dfu.status = (g_last_err == BL_ERR_GUARD) ? DFU_STATUS_ERR_ADDRESS
                             : (g_last_err == BL_ERR_VERIFY) ? DFU_STATUS_ERR_CHECK_ERASED
                                                             : DFU_STATUS_ERR_ERASE;
            g_dfu.state = (uint8_t)DFU_ST_ERROR;
            g_dfu.busy = 0U;
            return;
        }
        g_dfu.need_erase = 0U;
    }

    if (!bl_addr_in_app(addr, g_dfu.rx_len))
    {
        g_dfu.status = DFU_STATUS_ERR_ADDRESS;
        g_dfu.state = (uint8_t)DFU_ST_ERROR;
        g_dfu.busy = 0U;
        return;
    }

    g_last_err = bl_app_write(addr, g_dfu.buf, g_dfu.rx_len);
    if (g_last_err != BL_OK)
    {
        /* 编程失败用 errPROG（0x06），和"擦除失败"(0x04)、"擦不干净"(0x05) 分开：
           这三者在现场是完全不同的病因（写保护 / 扇区图 / 供电时序） */
        g_dfu.status = (g_last_err == BL_ERR_GUARD) ? DFU_STATUS_ERR_ADDRESS
                                                    : DFU_STATUS_ERR_PROG;
        g_dfu.state = (uint8_t)DFU_ST_ERROR;
        g_dfu.busy = 0U;
        return;
    }

    g_dfu.busy = 0U;
    g_dfu.state = (uint8_t)DFU_ST_DNLOAD_IDLE;
}
