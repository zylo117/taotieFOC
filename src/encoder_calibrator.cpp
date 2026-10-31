#include "encoder_calibrator.h"

#include "closed_loop_controller.h"

// ===========================================================================
// 磁编码器双程校准（拆分第 1 阶段：骨架）
//
// 这个文件目前只有构造与状态查询，四个主流程（start / update / finish /
// startMove）还在 closed_loop_controller.cpp 里跑。
//
// 拆分节奏（避免一次搬 400 行引出回归）：
//   阶段 1（本文件）：类壳 + 状态成员 + 访问器
//   阶段 2：搬 finish()（最短，依赖 saved_* 与 encoder_）
//   阶段 3：搬 startMove()（依赖 motion_* 与 driver_）
//   阶段 4：搬 update()（最长，四个状态机）
//   阶段 5：搬 start()，并把 controller 的 calibrateEncoder /
//           updateEncoderCalibration / finishEncoderCalibration /
//           startEncoderCalibrationMove 改成四行转发
//   阶段 6：删掉 controller 里 15 个 calibration_* 成员与 EncoderCalibrationStage 枚举
//
// 每阶段都可独立编译、独立刷机验证，出问题只回退一个提交。
// ===========================================================================

EncoderCalibrator::EncoderCalibrator(ClosedLoopController& owner) : owner_(owner)
{
}

bool EncoderCalibrator::isIdle() const
{
    return stage_ == STAGE_IDLE;
}
