/**
 * bl_layout.h —— taotieFOC 自定义 Bootloader 的 Flash / RAM 分区单一真相源。
 *
 * bootloader/src 和 src（应用）都 include 这一份：BL 和应用对"哪里是应用、
 * 哪里是交接页"的理解必须逐字节一致，写两份迟早对不上。
 *
 * ============================ Flash 分区（256KB）============================
 *   0x08000000 ~ 0x08001FFF   Bootloader   8KB   ← DFU 永不写（软件守卫 + EPP 硬件写保护）
 *   0x08002000 ~ 0x0801BFFF   应用        104KB  ← DFU 唯一允许写入的区间
 *   0x0801C000 ~ 0x0801C7FF   滞后角拟合元数据     ← 永不触碰（lag_angle.h FLASH_META_BASE）
 *   0x0801D000 ~ 0x0801E7FF   滞后角拟合点数据     ← 永不触碰
 *   0x0801F000 / 0x0801F800   编码器校准元数据/主表 ← 永不触碰
 *   0x08020000 ~ 0x0803FFFF   FastCal 稠密表 128KB ← 永不触碰
 *
 * 为什么应用起点是 0x08002000 而不是更"整齐"的 0x08004000：
 *   当前应用镜像 99,720 字节。16KB BL 只剩 96KB 代码区，装不下（差 1.4KB）；
 *   8KB BL 剩 104KB，还能长 6.6KB。0x08002000 在 1K/2K/4K 任意扇区图下都是
 *   扇区边界（AT32F403A 低 16 个扇区是 1KB），所以擦应用不会连累 BL 最后一个扇区；
 *   bl_flash.c::bl_app_erase_all() 还在擦除前后各做一次 BL 边界哨兵校验兜底。
 *
 * ============================ RAM 交接页 ==================================
 *   0x20017C00 ~ 0x20017FFF   1KB 交接页
 * 两份链接脚本都把 RAM 长度从 96K 改成 95K、_estack 从 0x20018000 改成
 * 0x20017C00，所以这一页既不在任何 .bss/.data/堆栈范围内（不会被启动代码清零，
 * 也不会被栈踩到），NVIC_SystemReset / FWDGT 复位后又不会被断电清掉 —— 这正是
 * "应用主动跳回 BL"和"连续启动失败计数"需要的持久性。冷上电时这里是随机值，
 * 所以一切读取都必须先验 magic + crc。
 */
#ifndef BL_LAYOUT_H
#define BL_LAYOUT_H

/* ---------------- Flash ---------------- */
#define BL_REGION_BASE        0x08000000u
#define BL_REGION_SIZE        0x00002000u                    /* 8KB */
#define BL_REGION_END         (BL_REGION_BASE + BL_REGION_SIZE)

#define APP_REGION_BASE       0x08002000u
#define APP_REGION_SIZE       0x0001A000u                    /* 104KB，到 0x0801C000 */
#define APP_REGION_END        (APP_REGION_BASE + APP_REGION_SIZE)

/* 校准/滞后角/FastCal 数据区起点：BL 与 DFU 的写守卫用这条线一票否决 */
#define DATA_REGION_START     0x0801C000u

/* 镜像尾签：固定在应用分区最后 16 字节，校验通过才允许跳转 */
#define APP_TRAILER_ADDR      (APP_REGION_END - 16u)         /* 0x0801BFF0 */
#define APP_TRAILER_MAGIC     0x4C49465Au                    /* "ZFIL" */
#define APP_PAYLOAD_MAX       (APP_REGION_SIZE - 16u)        /* 0x1AFF0 */

/* DFU 一次下载块大小 = EP0 最大包，单包数据阶段，绝不走 EP0 多包路径 */
#define DFU_XFER_SIZE         64u

/* ---------------- RAM 交接页 ---------------- */
#define BL_HSK_ADDR           0x20017C00u
#define BL_HSK_MAGIC          0x544F4154u                    /* "TAOT" */
#define BL_HSK_REQ_NONE       0x00000000u
#define BL_HSK_REQ_DFU        0x00010001u                    /* 应用请求：下次复位留在 BL */

/* 连续启动失败达到这个次数就常驻 DFU（坏镜像反复复位不再无限重启） */
#define BL_MAX_BOOT_ATTEMPT   3u

#ifndef __ASSEMBLER__
#include <stdint.h>

typedef struct
{
    uint32_t magic;         /* BL_HSK_MAGIC 才有效；冷上电随机值会被判无效 */
    uint32_t request;       /* BL_HSK_REQ_DFU = 请求进 DFU */
    uint32_t boot_attempt;  /* BL 跳转前 +1，应用确认启动后清 0 */
    uint32_t status;        /* BL 写的诊断码（上位机可经 0x19B 读回） */
    uint32_t crc;           /* 前 4 个字异或 */
} bl_hsk_t;

#endif /* __ASSEMBLER__ */
#endif /* BL_LAYOUT_H */
