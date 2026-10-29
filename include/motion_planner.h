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
// 已迁移：
//   A. updateAngleMode()  —— 角度模式的位置/速度环 + 加减速限制（原 rampUpdate 1002~1076）
// 待迁移：
//   B. updateTrapezoid()  —— 梯形规划 + 往返 + 结束判定（原 1077~1183）
//   C. emitSteps()        —— DDA 累加 + 脉冲出口（原 1281~1350）
class MotionPlanner
{
public:
    explicit MotionPlanner(ClosedLoopController& owner);

    // 角度模式：以真实机械角度误差做位置环 -> 速度环 -> 加减速限制，
    // 最后更新 current_step_speed_ / motion_speed_rpm_ / motion_direction_。
    // 参数是本拍的时间增量（ns）。
    void updateAngleMode(uint64_t delta_ramp_ns);

private:
    ClosedLoopController& owner_;
};

#endif
