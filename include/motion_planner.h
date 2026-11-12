#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <cstdint>

class ClosedLoopController;

// 运动规划器：把 rampUpdate 里三块规划逻辑逐步搬出来（A/B/C 三步）。
//
// 为什么用 friend 而不是接口/虚函数：
//   rampUpdate 跑在 20kHz 硬实时路径上，20kHz × 虚函数间接调用是实实在在的开销，
//   而且这块代码要读写的 30 多个状态（desired_rps / motion_step_accumulator_ /
//   compensated_rps_ ...）拆成 getter/setter 反而更慢。所以沿用项目里
//   ExternalPulseFollower / ParameterRouter 的既有模式：friend + 直接成员访问，零开销。
//
//   已迁移：
//   A. updateAngleMode()     —— 角度模式的位置/速度环 + 加减速限制
//   B. updateTrapezoid()     —— 往返换向、运动结束判定、梯形加减速
//   C. emitSteps()           —— DDA 累加 + 脉冲出口
class MotionPlanner
{
public:
    explicit MotionPlanner(ClosedLoopController& owner);

    // 角度模式：以真实机械角度误差做位置环 -> 速度环 -> 加减速限制，
    // 最后更新 current_step_speed_ / motion_speed_rpm_ / motion_direction_。
    void updateAngleMode(uint64_t delta_ramp_ns);

    // 梯形规划（仅限非角度模式）：
    //   往返运动的换向处理 -> 全部脉冲发完后的收尾/沉降 -> 加速/匀速/减速阶段推进。
    // 返回 true 表示本拍已经处理完（结束或进入沉降），调用方应立即 return，
    // 不要再跑后面的 DDA 和脉冲出口 —— 这是原代码里那两个 return 的等价表达。
    bool updateTrapezoid(uint64_t delta_ramp_ns, uint64_t now_ns);

    // DDA 微分累加 + 脉冲出口：把本拍应该走的步数累加到累加器，铺成 ARR 序列并启动 TMR2/DMA。
    // 返回 true 表示出口拒绍（窗口连头尾 guard 都放不下），调用方应立即 return，
    // 累加器里的步数留到下一拍慢慢补。
    bool emitSteps(uint64_t delta_ramp_ns, uint64_t now_ns);

    // ---------- 速度模式连续硬件 PWM（2026-11-12，450RPM 失步根因修复） ----------
    // 连续速度模式（0x11E=2/3 + 0x11D=0）不走"每 50µs 重启 ARR 序列"的窗口出口，
    // 而是让 TMR2 自由连续输出恒定周期 PWM（每拍只更新一次带预装载的 PR）：
    // 脉冲间隔全局均匀，没有窗口边界死区、没有每窗重初始化 —— 与原版开环固件
    // 同一拓扑。窗口重启的边界波纹在中高速吃掉了转矩裕量（实测 400RPM 持续丢
    // 4% 步、430+ 崩溃，两台电机同速死）。位置/角度/校准/带计数速度段仍走窗口。
    // startMotion 置位，stopMotion 经 velocityPwmHalt 清零。
    bool velocity_pwm_active_ = false;
    bool velocity_pwm_running_ = false;
    // 分频档：0=PSC0（4ns tick，3815pps~500kpps），1=PSC7（32ns tick，477~496kpps）。
    // 64 细分下 16 位 PR + PSC0 的最低可表示速率是 3815pps≈17.9RPM——低于它的
    // 指令原来直接不发脉冲（死区，低速起步会被误判堵转）。加粗分频档后低速
    // 下探到 477pps≈2.2RPM；两档重叠区用迟滞（>8000 升档、<3000 降档）避免抖档。
    uint8_t velocity_pwm_tier_ = 0;
    // 窄脉冲模式：pps>450k 时周期已 <2.2µs，1µs 高电平放不下，切 300ns 高电平
    //（周期下限放宽到 1µs → 上限 1M pps ≈4687RPM@64细分）。依据是用户的实地
    // 事实：老固件 64 细分 4000RPM（853kpps）近乎零误差——数据手册的 2µs
    // 下限对这块硬件不是墙，把它当墙是我的错误（低速段仍用 1µs 宽脉冲抗振铃，
    // 那是 24V 窄脉冲丢步修复保留的部分）。迟滞 450k 进 / 420k 出。
    bool velocity_pwm_narrow_ = false;
    uint32_t velocity_pwm_ccr_ = 0U;   // 当前 CCR 装载值；0x123 改脉宽后触发重装
    int8_t velocity_pwm_dir_ = 0;      // 当前 DIR（0=未知，强制重装）；换向判据

    // 每拍调用（rampUpdate，updateTrapezoid 与补偿之后）。返回 true = 本拍脉冲
    // 已由连续 PWM 处理并完成记账，不要再走 emitSteps。
    bool velocityPwmTick(uint64_t delta_ramp_ns);

    // 停止连续输出并清 active（stopMotion 调用；重复调用安全）。
    void velocityPwmHalt();

private:
    ClosedLoopController& owner_;
};

#endif
