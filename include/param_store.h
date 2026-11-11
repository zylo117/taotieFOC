#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

// ===========================================================================
// 参数持久化：把上位机调的参数存进 Flash，重启后自动恢复。
//
// 背景：固件原来没有任何参数持久化（flash.h 只有底层 flash_read/flash_write），
// PID / 电流 / 微步 / 角度限速 全在 RAM 里，掉电即失。
// 这里用一个独立 2KB 扇区（0x0801E000，夹在滞后角数据 0x0801D000 与
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

    // 【v2 新增】角度模式（串级）的独立增益。同样的上位机口径（实际增益 × 1000）。
    // 为什么要分离：同一个 kp 在两条路径里含义完全不同 —— 角度模式里 PID 输出就是
    // 全部速度指令；规划运动/外部跟随里梯形（或外部脉冲）是主驱动、PID 只做小修正。
    // 共用一组参数等于让两种回路互相 “偷走”对方的整定结果，而且三个模式还共用
    // 同一份 PID 实例与积分状态 —— 这正是本项目反复踩过的“模式切换残留”坑。
    uint32_t angle_position_kp;
    uint32_t angle_position_ki;
    uint32_t angle_position_kd;
    uint32_t angle_velocity_kp;
    uint32_t angle_velocity_ki;
    uint32_t angle_velocity_kd;
    // 【v3 复用原 reserved[0]，结构体大小不变】使能来源：0=上位机，1=外部 EN 引脚(PC13)。
    // 旧记录这里读出来是 0（=上位机），正好是安全默认值，不需要结构体版本迁移。
    uint16_t en_mode;

    // 【v4】滞后角模型的两个可调参数（都不该写死，见 docs/lag_angle_and_pid.md）：
    //   lag_accel_coef_mdeg_per_rpm_s：加速时转子要多落后多少。物理上 ΔL ≈ (J/k)·a，
    //       所以用"毫度/(RPM/s)"的线性系数，默认 1（600RPM/s 时约 0.6°）。
    //       0 = 关闭加减速项（退回只按转速查表）。
    //   lag_deadband_mdeg：恒速段的残差死区（毫度），默认 150。
    //       它应该和"滞后角表有多准"匹配：表越准可以越小；标定不充分就给它大一点。
    uint16_t lag_accel_coef_mdeg_per_rpm_s;
    uint16_t lag_deadband_mdeg;

    // 【v5】运动源 + 共振速度区间。
    //   motion_source：0=内部规划，1=外部脉冲跟随（**默认 1**）。做成可存是因为
    //       "上电默认进哪种模式"是设备形态问题（PLC 集成 vs 网页调试），不同用户
    //       答案不同；而且它是运动信号的绝对互斥开关，重启后必须保持用户的选择。
    //   resonance_lo/hi_rpm：共振区间 [lo,hi]，默认 300~600（实测剧烈抖动带）。
    //       规划/速度模式的巡航目标落进区间会被钳到 lo（"快切通过"），
    //       自动测试用 0x171 旁路。0~0 或 lo>=hi 表示关闭。
    uint16_t motion_source;
    uint16_t resonance_lo_rpm;
    uint16_t resonance_hi_rpm;

    // 【v6】堵转保护阈值（网页可调；默认 = 编译期口径 200ms / 100LSB）。
    //   stall_quiet_ms：命令转动时，编码器"净位移不足"持续这么久才判堵转。
    //       加大 = 更宽容（低速大惯量/慢响应系统防误报），减小 = 更灵敏。
    //   stall_move_lsb：窗口内净位移小于它算"没动"。必须明显大于静止噪声的
    //       窗口净位移（实测噪声 pp~31LSB、200ms 窗内净位移 ≤100LSB），
    //       又必须明显小于最低速的真实位移（8RPM×200ms ≈ 1750LSB）。
    uint16_t stall_quiet_ms;
    uint16_t stall_move_lsb;

    // 【v7】磁钢极性（0=正常，1=反装已反转读数）。
    //   换电机/换磁铁时磁钢可能轴向翻转，读数方向镜像；校准入口的极性探针
    //   自动鉴别并反转（见 encoder_calibrator.cpp updatePolarityProbe），
    //   校准完成时随校准表一并固化 —— 标志必须和非线性表同域，丢了就全乱。
    uint16_t encoder_invert;
};

// 从 Flash 读取参数到 out。返回 true 表示校验通过、内容有效。
bool param_store_load(PersistedMotorParams* out);

// 把 out 写入 Flash。返回 true 表示写入并回读校验成功。
bool param_store_save(const PersistedMotorParams* in);

#endif // PARAM_STORE_H
