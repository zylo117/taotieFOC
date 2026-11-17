/**
 * bl_dfu.h —— 手写 USB DFU 1.1 类（EP0-only），Artery USBD 栈里没有现成 DFU 类。
 *
 * 为什么手写：middlewares/usbd_class 只有 audio/cdc/custom_hid/hid_iap/msc/winusb…，
 * 没有 dfu；而官方 IAP demo 走的是 HID-IAP，上位机要自己写协议，不如直接用标准 DFU
 * —— 现成 dfu-util 就能刷，PlatformIO 的 upload_protocol = dfu 也能继续用。
 *
 * 只做 EP0：wTransferSize = 64 = EP0 最大包，数据阶段永远单包，绕开 AT32 栈的
 * EP0 多包续传路径（那条路径在 usbd_core.c:108 用同一个 trans_buf 重新 arm，
 * 不敢保证偏移正确）。64 字节/包刷 104KB ≈ 1664 次控制传输，实测通常 2~4 秒，够用。
 */
#ifndef BL_DFU_H
#define BL_DFU_H

#include "usb_std.h"
#include "usbd_core.h"
#include "bl_layout.h"

/* DFU 类请求（DFU 1.1 §6.1） */
#define DFU_REQ_DNLOAD        0x01U
#define DFU_REQ_UPLOAD        0x02U
#define DFU_REQ_GETSTATUS     0x03U
#define DFU_REQ_CLRSTATUS     0x04U
#define DFU_REQ_GETSTATE      0x05U
#define DFU_REQ_ABORT         0x06U

/* bStatus（DFU 1.1 Table 13，只列用到的） */
/* bStatus 必须严格照 DFU 1.1 规范 Table 13 的编号。
   【这里曾经整张表错位】我当时按"自己的顺序"排了 02/04/06/07/0B，而规范是
   03=errWRITE、04=errERASE、05=errCHECK_ERASED、06=errPROG、07=errVERIFY、08=errADDRESS。
   后果非常坏：擦除失败我发 0x07，dfu-util 拿正确的表翻译成 "Programmed memory failed
   verification"，于是所有人（包括我）都顺着"校验失败"去查 manifest，而真凶是擦除。
   上位机的报错文本是按规范生成的，设备侧自己另立编号等于把诊断信息搅浑。 */
#define DFU_STATUS_OK               0x00U
#define DFU_STATUS_ERR_TARGET       0x01U
#define DFU_STATUS_ERR_FILE         0x02U
#define DFU_STATUS_ERR_WRITE        0x03U
#define DFU_STATUS_ERR_ERASE        0x04U
#define DFU_STATUS_ERR_CHECK_ERASED 0x05U   /* 擦完仍有非 0xFF 的位置 */
#define DFU_STATUS_ERR_PROG         0x06U
#define DFU_STATUS_ERR_VERIFY       0x07U   /* 镜像 CRC / 内容不符 */
#define DFU_STATUS_ERR_ADDRESS      0x08U   /* 地址守卫拒绝 */
#define DFU_STATUS_ERR_NOTDONE      0x0AU
#define DFU_STATUS_ERR_FIRMWARE     0x0BU
#define DFU_STATUS_ERR_VENDOR       0x0CU
#define DFU_STATUS_ERR_USBR         0x0DU
#define DFU_STATUS_ERR_POR          0x0EU
#define DFU_STATUS_ERR_UNKNOWN      0x0FU
#define DFU_STATUS_ERR_NOTSTATED    0xFFU

/* bState（DFU 1.1 Table 14） */
typedef enum
{
    DFU_ST_APP_IDLE = 0,
    DFU_ST_APP_DETACH = 1,
    DFU_ST_DFU_IDLE = 2,
    DFU_ST_DNLOAD_SYNC = 3,
    DFU_ST_DNBUSY = 4,
    DFU_ST_DNLOAD_IDLE = 5,
    DFU_ST_MANIFEST_SYNC = 6,
    DFU_ST_MANIFEST = 7,
    DFU_ST_MANIFEST_WAIT_RESET = 8,
    DFU_ST_UPLOAD_IDLE = 9,
    DFU_ST_ERROR = 10
} dfu_state_t;

typedef struct
{
    __IO uint8_t  state;                          /* dfu_state_t */
    __IO uint8_t  status;                         /* bStatus */
    __IO uint8_t  busy;                           /* 1 = 有擦/写活等主循环干 */
    __IO uint8_t  need_erase;                     /* 本轮下载第一块之前先擦整片 */
    __IO uint8_t  manifest_done;                  /* manifest 校验通过，主循环延时后复位 */
    __IO uint32_t block;                          /* 当前块号 → 目标地址 */
    __IO uint16_t rx_len;
    uint8_t       buf[DFU_XFER_SIZE];             /* DNLOAD 暂存 */
    uint8_t       tx_buf[DFU_XFER_SIZE];          /* UPLOAD 暂存 */
} dfu_dev_t;

extern usbd_class_handler dfu_class_handler;
extern dfu_dev_t g_dfu;

/* 最后一次收到 DFU 请求时的 DWT->CYCCNT 快照。
   manifest 之后主循环用它判断"上位机已经完事了"：静默 2.5s 才复位。
   为什么不能定长延时（比如 400ms）就复位：上位机还要用 DFU_UPLOAD 把整片 104KB
   读回来逐字节校验，设备半路消失，校验就变成"读不到"，看起来像刷机失败。 */
extern volatile uint32_t g_dfu_last_req_cyc;

/* 主循环：把 GETSTATUS 期间报成 dfuDNBUSY 的活真正干掉（擦 104KB 约 0.3~1s，
   绝不能在 USB 中断里做，否则主机会 STALL 超时） */
void bl_dfu_poll_work(void);

#endif /* BL_DFU_H */
