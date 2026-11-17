/**
 * bl_desc.h —— DFU 设备描述符（VID/PID 刻意复用芯片内置 DFU 的那一对）。
 *
 * Windows 设备树里已经有 "AT32 Bootloader DFU" USB\VID_2E3C&PID_DF11 这条 HWID
 * 记录并绑好了驱动（就是现在按 BOOT0 进的那个内置 bootloader）。自定义 BL 用同一对
 * VID/PID，插上就能被 dfu-util 认到，不用再用 Zadig 装一遍过滤驱动 —— 这是"自动刷机"
 * 能不能真的自动的关键一环。应用侧 CDC 用的是 VID_2E3C&PID_5740，两者不冲突。
 */
#ifndef BL_DESC_H
#define BL_DESC_H

#include "usb_std.h"
#include "usbd_core.h"

#define BL_USBD_VENDOR_ID     0x2E3CU
#define BL_USBD_PRODUCT_ID    0xDF11U

extern usbd_desc_handler dfu_desc_handler;

#endif /* BL_DESC_H */
