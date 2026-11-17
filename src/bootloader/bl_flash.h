/**
 * bl_flash.h —— Bootloader 侧的守卫式 Flash 操作 + 镜像校验 + 跳应用。
 * 分区与交接页定义见 include/bl_layout.h（BL 和应用共用同一份）。
 */
#ifndef BL_FLASH_H
#define BL_FLASH_H

#include <stdint.h>
#include "bl_layout.h"

typedef enum
{
    BL_OK = 0,
    BL_ERR_GUARD,      /* 地址越界：试图碰 BL 区或校准数据区，一票否决 */
    BL_ERR_ERASE,      /* flash_sector_erase 报错 */
    BL_ERR_PROGRAM,    /* flash_word_program 报错 */
    BL_ERR_VERIFY,     /* 擦完不是 0xFF / 写回不一致 / 向量表不合理 */
    BL_ERR_NO_IMAGE,   /* 尾签缺失或 CRC 不符（不是硬件错误，是"该留在 DFU"） */
    BL_ERR_EPP         /* EPP 操作失败 */
} bl_err_t;

/* ---------------- RAM 交接页 ---------------- */
bl_hsk_t *bl_hsk(void);
uint32_t  bl_hsk_crc(const bl_hsk_t *h);
/* 有效返回 1；冷上电随机值会被 magic/crc 挡掉并原地初始化 */
int       bl_hsk_valid(void);
void      bl_hsk_init_fresh(void);
void      bl_hsk_store(const bl_hsk_t *h);

/* ---------------- 镜像校验 ---------------- */
uint32_t bl_crc32(const volatile uint8_t *p, uint32_t len);
/* 读尾签 + CRC + 向量表三重检查。有效返回 1，并可选回填诊断 */
int        bl_app_valid(uint32_t *payload_len, uint32_t *status_out);

/* ---------------- 守卫式擦写 ---------------- */
/* 地址必须整段落在 [APP_REGION_BASE, DATA_REGION_START) 且 4 字节对齐 */
int      bl_addr_in_app(uint32_t addr, uint32_t len);
bl_err_t bl_app_erase_all(void);
bl_err_t bl_app_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/* ---------------- 跳转 ---------------- */
bl_err_t bl_jump_to_app(void);

/* ---------------- EPP 硬件写保护（可选的二次保险） ---------------- */
/* BL 区 8KB = EPP bit0/bit1（每 bit 管 4KB，见 flash 库 flash_epp_set 注释）。
   置位后 BL 区连 BL 自己也擦不掉；解除要 flash_user_system_data_erase()，
   之后必须用 ST-Link 重刷 BL。所以它是"手动加固"，不是运行时开关。 */
bl_err_t bl_epp_protect_bl(void);
uint32_t bl_epp_bits(void);   /* FLASH->epps 原值：某 bit 为 0 = 该 4KB 已受保护 */

#endif /* BL_FLASH_H */
