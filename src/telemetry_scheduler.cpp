#include "telemetry_scheduler.h"

TelemetryScheduler::Slot TelemetryScheduler::tick()
{
    ++tick_;
    // 先判慢档：kSlowDivider 是 kMediumDivider 的整数倍，
    // 所以慢档那一拍必然也是中档，必须让慢档优先命中，否则慢变量永远刷不到。
    if ((tick_ % kSlowDivider) == 0U)
    {
        return Slot::kSlow;
    }
    if ((tick_ % kMediumDivider) == 0U)
    {
        return Slot::kMedium;
    }
    return Slot::kNone;
}

void TelemetryScheduler::reset()
{
    tick_ = 0U;
}
