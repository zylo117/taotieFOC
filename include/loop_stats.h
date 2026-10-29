#ifndef LOOP_STATS_H
#define LOOP_STATS_H

#include <cstdint>

// 控制环频率统计。
//
// 从 ClosedLoopController 里独立出来的原因：这是一段纯统计逻辑，和运动控制没有任何交集，
// 却占着 12 个成员和 4 个函数。单独成类之后，控制器那边只剩一个 loop_stats_ 句柄。
//
// 采样口径说明（踩过的坑）：
//   位置环原来用「单拍瞬时值」= 1e9 / delta_ns，每拍覆盖。但 USB 中断优先级 0 高于
//   TMR4 的 1，它一抢占 5.8us，瞬时值就从 20000 掉到 17919，看着像掉频其实只是观测抖动
//   （真漏一拍会显示 10000 而不是 17919）。所以这里改成 500ms 窗口内的计数平均：
//   既滤掉单拍抖动，又能真实暴露漏拍。
//   速度环和电流环仍然保留瞬时值，它们的采样点不在 TMR4 里，抖动不影响判读。
class LoopStats
{
public:
    // 每拍调用一次，传入 DWT 纳秒时间戳。
    void update(uint64_t time_ns);

    void setEnabled(bool enable) { enabled_ = enable; }
    bool enabled() const { return enabled_; }

    uint32_t positionLoopHz() const { return position_hz_; }
    uint32_t velocityLoopHz() const { return velocity_hz_; }
    uint32_t currentLoopHz() const { return current_hz_; }

    uint32_t positionSamples() const { return position_samples_; }
    uint32_t velocitySamples() const { return velocity_samples_; }
    uint32_t currentSamples() const { return current_samples_; }

    void reset();

private:
    bool enabled_ = false;

    uint64_t last_position_tick_ns_ = 0ULL;
    uint64_t last_velocity_tick_ns_ = 0ULL;
    uint64_t last_current_tick_ns_ = 0ULL;

    uint32_t position_hz_ = 0U;
    uint32_t velocity_hz_ = 0U;
    uint32_t current_hz_ = 0U;

    // 位置环 500ms 计数平均用的滑动窗口
    uint64_t position_window_ns_ = 0ULL;
    uint32_t position_window_count_ = 0U;

    uint32_t position_samples_ = 0U;
    uint32_t velocity_samples_ = 0U;
    uint32_t current_samples_ = 0U;
};

#endif
