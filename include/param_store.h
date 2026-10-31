#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

// ===========================================================================
// 参数持久化：把上位机调的参数存进 Flash，重启后自动恢复。
//
// 背景：固件原来没有任何参数持久化（flash.h 只有底层 flash_read/flash_write），
// PID / 电流 / 微步 / 角度限速 全在 RAM 里，掉电即失。
// 这里用一个独立 2KB 扇区（0x0801E000，夹在负载角数据 0x0801D000 与
// 校准表元数据 0x0801F000 之间）存一份紧凑的结构体 + 校验。
//
// 注意：写入会 stall Flash 总线（几毫秒），只能在用户显式点「写入参数」时做，
// 绝不能在控制环或运动中自动触发。
// ===========================================================================

struct PersistedMotorParams
{
    uint32_t magic;             // 固定 0x54414F54 ('TAOT')
    uint16_t version;           // 结构体版本，改字段时递增
    uint16_t checksum;          // 对后续所有字节的累加和
    uint16_t microsteps;        // 细分
    uint16_t run_current_ma;    // 运行电流（mA）
    uint16_t hold_current_ma;   // 保持电流（mA）
    uint16_t step_pulse_ns;     // STEP 脉宽（ns，16 位够用到 65us）
    uint16_t angle_max_rpm;     // 角度模式限速（已不存，保留字段兼容）
    uint16_t angle_accel_rpm_s; // 角度模式加速度（已不存，保留字段兼容）

    // PID：上位机口径（实际增益 × 1000）。
    // 【必须是 32 位】kp 可到 30000（增益 30 × 1000），16 位会溢出：
    // 实测 9000×1000 = 9,000,000 被截成 21568，日志出现 pid 21568/...
    uint32_t position_kp;
    uint32_t position_ki;
    uint32_t position_kd;
    uint32_t velocity_kp;
    uint32_t velocity_ki;
    uint32_t velocity_kd;
    uint16_t reserved[7];       // 预留到字号对齐，方便以后扩展
};

// 从 Flash 读取参数到 out。返回 true 表示校验通过、内容有效。
bool param_store_load(PersistedMotorParams* out);

// 把 out 写入 Flash。返回 true 表示写入并回读校验成功。
bool param_store_save(const PersistedMotorParams* in);

#endif // PARAM_STORE_H
