#include "loop_stats.h"

void LoopStats::update(uint64_t time_ns)
{
    if (!enabled_)
    {
        return;
    }

    // 位置环：500ms 窗口计数平均，滤掉单拍被抢占造成的假掉频
    if (time_ns > last_position_tick_ns_)
    {
        const uint64_t delta_ns = time_ns - last_position_tick_ns_;
        if (delta_ns > 0ULL)
        {
            position_window_ns_ += delta_ns;
            position_window_count_++;
            if (position_window_ns_ >= 500000000ULL)
            {
                position_hz_ = static_cast<uint32_t>(
                    (static_cast<uint64_t>(position_window_count_) * 1000000000ULL) / position_window_ns_);
                position_window_ns_ = 0ULL;
                position_window_count_ = 0U;
            }
            position_samples_++;
        }
        last_position_tick_ns_ = time_ns;
    }

    // 速度环：瞬时值（采样点不在 TMR4 里，抖动不影响判读）
    if (time_ns > last_velocity_tick_ns_)
    {
        const uint64_t delta_ns = time_ns - last_velocity_tick_ns_;
        if (delta_ns > 0ULL)
        {
            velocity_hz_ = static_cast<uint32_t>(1000000000ULL / delta_ns);
            velocity_samples_++;
        }
        last_velocity_tick_ns_ = time_ns;
    }

    // 电流环：同上
    if (time_ns > last_current_tick_ns_)
    {
        const uint64_t delta_ns = time_ns - last_current_tick_ns_;
        if (delta_ns > 0ULL)
        {
            current_hz_ = static_cast<uint32_t>(1000000000ULL / delta_ns);
            current_samples_++;
        }
        last_current_tick_ns_ = time_ns;
    }
}

void LoopStats::reset()
{
    last_position_tick_ns_ = 0ULL;
    last_velocity_tick_ns_ = 0ULL;
    last_current_tick_ns_ = 0ULL;
    position_hz_ = 0U;
    velocity_hz_ = 0U;
    current_hz_ = 0U;
    position_window_ns_ = 0ULL;
    position_window_count_ = 0U;
    position_samples_ = 0U;
    velocity_samples_ = 0U;
    current_samples_ = 0U;
}
