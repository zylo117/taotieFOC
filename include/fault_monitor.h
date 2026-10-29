#ifndef FAULT_MONITOR_H
#define FAULT_MONITOR_H

#include <cstdint>

class ClosedLoopController;

// 故障监控：编码器读失败 / 磁场报警的处理策略。
//
// 为什么独立：这是一个纯粹的安全决策——"检测到故障时要不要失能驱动、要不要禁止输出"。
// 原来它和 output_stopped_ / driver_ 混在控制器里，审阅这段安全逻辑时要在
// reportXxxFault() 和 syncProtocolTelemetry() 之间来回翻，容易漏看。
//
// 对外行为完全不变：
//   - 故障 + 策略允许 -> 失能驱动、置 output_stopped_（禁止 STEP 输出）
//   - 故障恢复且另一路也正常 -> 自动清除 output_stopped_
//   - 两路故障同时存在时，先恢复的那一路不会解除封锁
class FaultMonitor
{
public:
    explicit FaultMonitor(ClosedLoopController& owner);

    void setPolicy(bool stop_on_encoder_fault, bool stop_on_magnetic_fault);

    void reportEncoderFault(bool active);
    void reportMagneticFieldAlarm(bool active);

    bool encoderFaultActive() const { return encoder_fault_active_; }
    bool magneticFaultActive() const { return magnetic_fault_active_; }
    bool outputStopped() const { return output_stopped_; }

private:
    ClosedLoopController& owner_;

    bool stop_on_encoder_fault_ = true;
    bool stop_on_magnetic_fault_ = true;
    bool encoder_fault_active_ = false;
    bool magnetic_fault_active_ = false;
    // true = 因故障禁止一切 STEP 输出（外部跟随也不会发脉冲）
    bool output_stopped_ = false;
};

#endif
