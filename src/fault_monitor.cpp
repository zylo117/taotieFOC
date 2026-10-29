#include "fault_monitor.h"

#include "closed_loop_controller.h"
#include "stepper_driver.h"

FaultMonitor::FaultMonitor(ClosedLoopController& owner) : owner_(owner)
{
}

void FaultMonitor::setPolicy(bool stop_on_encoder_fault, bool stop_on_magnetic_fault)
{
    stop_on_encoder_fault_ = stop_on_encoder_fault;
    stop_on_magnetic_fault_ = stop_on_magnetic_fault;
}

void FaultMonitor::reportEncoderFault(bool active)
{
    encoder_fault_active_ = active;
    if (active && stop_on_encoder_fault_ && owner_.driver_ != nullptr)
    {
        owner_.driver_->setEnable(false);
        output_stopped_ = true;
    }
    else if (!active && !magnetic_fault_active_)
    {
        // 两路故障都消失了才解除封锁：先恢复的那一路不能单独放行
        output_stopped_ = false;
    }
}

void FaultMonitor::reportMagneticFieldAlarm(bool active)
{
    magnetic_fault_active_ = active;
    if (active && stop_on_magnetic_fault_ && owner_.driver_ != nullptr)
    {
        owner_.driver_->setEnable(false);
        output_stopped_ = true;
    }
    else if (!active && !encoder_fault_active_)
    {
        output_stopped_ = false;
    }
}
