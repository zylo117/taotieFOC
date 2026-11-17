/**
 * fw_version.h —— 固件身份：git 版本 + 编译时刻，给寄存器、UART 和上位机用。
 *
 * 三层出口，覆盖"怎么证明板上跑的是哪一版"：
 *   1. 寄存器 0x19C(git) / 0x19D(编译时刻 epoch) / 0x19E(标志) —— 运行中随时可读，
 *      tools/fw_version.py 一条命令就能问出来；
 *   2. 开机 UART 横幅 —— 接串口线就能看见；
 *   3. 镜像里的 "TTFW1;" 魔数 —— 不用跟设备说话：无论 .pio 里的 bin、
 *      DFU 回读出来的 104KB、还是全片备份，扫这个魔数就知道这片 flash 里是哪版。
 *
 * 生成部分见 extra_version.py → include/fw_build.h（每次编译重写）。
 */
#ifndef FW_VERSION_H
#define FW_VERSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 镜像里的身份标记。改这个前缀要同步改 tools/ 里所有扫描它的脚本。 */
#define FW_BANNER_MAGIC "TTFW1;"

/* "TTFW1;<hash>[-dirty] @ <ISO> (<branch>)"，开机横幅和上位机解析都靠它 */
extern const char fw_version_string[];

uint32_t fw_git_num(void);      /* git 短哈希按十六进制转成的 32 位（回读时按 %07x 还原） */
uint32_t fw_build_epoch(void);  /* 编译时刻 Unix 秒 */
uint32_t fw_version_flags(void);/* bit0 = 工作区脏（板上代码在仓库里没有对应提交） */

#ifdef __cplusplus
}
#endif

#endif /* FW_VERSION_H */
