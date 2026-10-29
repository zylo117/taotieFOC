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
// 待迁移：
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

private:
    ClosedLoopController& owner_;
};

#endif
