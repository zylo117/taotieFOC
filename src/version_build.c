/**
 * version_build.c —— 固件身份的具体落地。见 include/fw_version.h。
 *
 * 这个文件唯一存在的意义就是"每次编译都会变"：extra_version.py 重写
 * include/fw_build.h，只有本文件依赖它，所以版本戳变化只重编这一个目标文件。
 *
 * fw_version_string 被 main.cpp 的开机 printf 引用，不会被 --gc-sections 回收，
 * 因此 "TTFW1;" 魔数一定在最终 bin 里 —— 上位机扫得到。
 */
#include "fw_version.h"

#include "fw_build.h"

const char fw_version_string[] = FW_BANNER_MAGIC FW_VERSION_STR;

uint32_t fw_git_num(void)
{
    return (uint32_t)FW_GIT_NUM;
}

uint32_t fw_build_epoch(void)
{
    return (uint32_t)FW_BUILD_EPOCH;
}

uint32_t fw_version_flags(void)
{
    return FW_GIT_DIRTY ? 0x00000001u : 0x00000000u;
}
