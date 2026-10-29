#ifndef TELEMETRY_SCHEDULER_H
#define TELEMETRY_SCHEDULER_H

#include <cstdint>

// 控制环里"多久刷一次遥测"的节拍器。
//
// 以前这段逻辑是 syncProtocolTelemetry() 开头的一串取模判断，直接依赖
// rampUpdate 里的 ramp_loop_idx（而且那个计数器原本是 uint16_t，65535 回绕后
// 一秒钟只能命中 4 次 % 20000）。现在把节拍独立出来，控制环只问它"这一拍该刷哪档"。
//
// 分档：
//   kSlow   ：约 1Hz。故障状态、环路频率、参数回显这些慢变量
//   kMedium ：约 100Hz。电流、编码器角度、跟随误差、外部跟随积压这些要画波形的
//   kNone   ：本拍不刷（绝大多数拍都走这里）
//
// 注意这里刻意不做 kFast：0x121 跟随误差已经由 UsbCdcProtocolBridge::sendTelemetry
// 的 case 0 用快帧单独推送（约 200Hz），控制环再按 20kHz 刷一遍纯属浪费。
class TelemetryScheduler
{
public:
    enum class Slot : uint8_t
    {
        kNone = 0U,
        kSlow,
        kMedium
    };

    // 推进一拍并返回本拍应刷的档位。
    Slot tick();

    // 控制环重启/重新初始化时把节拍对齐回 0
    void reset();

    // 20kHz 控制环下的分频数
    static constexpr uint32_t kControlLoopHz = 20000U;
    static constexpr uint32_t kSlowDivider = kControlLoopHz;        // 1Hz
    static constexpr uint32_t kMediumDivider = kControlLoopHz / 100U;  // 100Hz

private:
    uint32_t tick_ = 0U;
};

#endif
