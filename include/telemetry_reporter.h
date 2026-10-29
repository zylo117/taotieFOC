#ifndef TELEMETRY_REPORTER_H
#define TELEMETRY_REPORTER_H

#include <cstdint>

#include "telemetry_scheduler.h"

class ClosedLoopController;

// 遥测上报：把控制器状态按档位刷进协议寄存器表。
//
// 从 ClosedLoopController::syncProtocolTelemetry 独立出来。那边是 94 行的
// 字典式 setCustomParameter 映射，和运动控制逻辑没有任何交集，却和 rampUpdate
// 挤在一个文件里。搬走之后，控制器只需要"决定什么时候刷"，"刷什么"归这里管。
//
// 档位（由 TelemetryScheduler 产生，20kHz 一拍推进一格）：
//   kSlow   —— 约 1Hz：故障位、环频率、运动参数、脉宽
//   kMedium —— 约 100Hz：电流、位置、跟随误差、编码器角、外部跟随计数
//   kNone   —— 不刷（stopMotion / enableExternalPulseFollow 这类非节拍路径会调到）
//
// 注意：这里故意不刷 0x121 POSITION_DEG。它由 UsbCdcProtocolBridge::sendTelemetry
// 的快帧（约 200Hz）单独推送，在 20kHz 路径上再刷一遍是纯浪费。
class TelemetryReporter
{
public:
    explicit TelemetryReporter(ClosedLoopController& owner);

    void report(TelemetryScheduler::Slot slot);

private:
    ClosedLoopController& owner_;
};

#endif
