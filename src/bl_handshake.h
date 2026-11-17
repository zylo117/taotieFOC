/**
 * bl_handshake.h —— 应用 ↔ 自定义 Bootloader 的 RAM 交接页访问（应用侧）。
 *
 * 交接页在 0x20017C00（RAM 顶 1KB），两份链接脚本都把它让了出来，
 * 所以它不被 .bss 清零、不被栈踩到，又能活过 NVIC_SystemReset —— 这正是
 * "应用不用按键就能回到 BL"需要的全部持久性。分区定义见 include/bl_layout.h。
 */
#ifndef BL_HANDSHAKE_H
#define BL_HANDSHAKE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 应用起来第一件事调用：告诉 BL"这次启动成功了"，清掉连续失败计数。
   计数到 3 次 BL 就常驻 DFU，避免坏镜像无限重启把板子变成只能拔插的哑巴。 */
void bl_hsk_confirm_boot(void);

/* 武装一次"下次复位留在 DFU"：只置标志 + 记 deadline，不立刻复位。
   真正复位由 main.cpp 的 usb_task 在 0x82 ACK 发出去之后调 bl_poll_dfu_reboot() 完成
   —— 在寄存器写处理函数里直接 NVIC_SystemReset() 会把 ACK 咽掉，
   上位机就只能靠超时猜"到底听没听见"。 */
void bl_arm_dfu_reboot(uint32_t delay_ms);

/* 到点就写握手页并复位；没到点什么都不做。返回不会真的返回（复位）。 */
void bl_poll_dfu_reboot(void);

/* BL 留下的诊断，给 0x19B 读回：status（'DF'/'JM' + 原因码）与 boot_attempt */
void bl_hsk_read(uint32_t *status, uint32_t *boot_attempt);

#ifdef __cplusplus
}
#endif

#endif /* BL_HANDSHAKE_H */
