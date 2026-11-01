#include "encoder_calibrator.h"

#include "closed_loop_controller.h"

// ===========================================================================
// 磁编码器双程校准（从 ClosedLoopController 逐步拆出）
//
// 拆分节奏（避免一次搬 400 行引出回归）：
//   阶段 1：类壳 + 状态成员 + 访问器                      已完成
//   阶段 2：finish()（最短，依赖 saved_* 与 encoder_）    已完成
//   阶段 3：startMove()（依赖 motion_* 与 getStepsPerRound）已完成
//   阶段 4：update()（最长，HOMING/SAMPLING 两个状态机） 已完成
//   阶段 5：start() + controller 的四个函数改成转发      已完成（本次，已接线）
//   阶段 6：删掉 controller 里其余 calibration_* 成员      已完成（本次）
//           （calibration_stage_ 与 calibration_saved_* 必须留下：
//            前者是与 motion_planner 的握手，后者是运动参数备份）
//
// 每阶段都可独立编译、独立刷机验证，出问题只回退一个提交。
// 从阶段 5 起本文件已是唯一生效的实现。controller 里只留两个确实有外部调用者
// 的转发：calibrateEncoder()（parameter_router 的 0x144）与
// updateEncoderCalibration()（rampUpdate 每拍），调用点一行都没改；
// finish() 与 startMove() 已成为本类私有方法，controller 侧的包装直接删了。
// ===========================================================================

namespace
{
    // controller 的 TU 里同名函数位于匿名命名空间，头文件不暴露；本文件自己实现一份，
    // 算法与 closed_loop_controller.cpp 的 normalize_signed_angle_error_deg /
    // normalize_signed_delta_deg(float) 完全一致（两者对 float 本来就是同一个 ±180 归一）。
    inline float wrap_deg_180(float deg)
    {
        while (deg > 180.0f)
        {
            deg -= 360.0f;
        }
        while (deg < -180.0f)
        {
            deg += 360.0f;
        }
        return deg;
    }

    inline float fast_absf(float value)
    {
        return (value < 0.0f) ? -value : value;
    }
}

EncoderCalibrator::EncoderCalibrator(ClosedLoopController& owner) : owner_(owner)
{
}

bool EncoderCalibrator::isIdle() const
{
    return owner_.calibration_stage_ == ClosedLoopController::CALIBRATION_IDLE;
}

uint8_t EncoderCalibrator::stage() const
{
    return static_cast<uint8_t>(owner_.calibration_stage_);
}

// 一小段校准位移走完时的握手：切 SAMPLING + 重置采样计数。
// 原本这段逻辑直接写在 motion_planner.cpp 的 updateTrapezoid 里（它当时能直接
// 写 controller 的 calibration_stage_ 与 5 个 calibration_sample_*）。
// 采样计数搬进本类后，planner 改为调这个方法，避免两个模块各持一份计数。
void EncoderCalibrator::beginSampling()
{
    owner_.calibration_stage_ = ClosedLoopController::CALIBRATION_SAMPLING;
    sample_count_ = 0U;
    sample_attempts_ = 0U;
    sample_sum_ = 0;
    sample_min_ = 0x7FFFFFFF;
    sample_max_ = (-0x7FFFFFFF - 1);
}

// ===========================================================================
// 启动一次双程校准（对应原 calibrateEncoder），由上位机写 0x144 触发。
//
// 做四件事，顺序不能换：
//   ① 门禁：阶段必须 IDLE；driver/encoder 就绪；编码器与磁场无故障。
//   ② clearCalibration 清掉旧表：否则 raw 读数会被旧表修正过，
//      新校准采到的是“已修正”的值，等于在旧表上再叠一层。
//   ③ 抢下输出通道：关掉外部脉冲跟随，保证角度模式能真正驱动电机。
//   ④ 备份运动参数 + 初始化校准状态 + 切角度模式回零。
// ===========================================================================
void EncoderCalibrator::start()
{
    ClosedLoopController& c = owner_;

    if (c.calibration_stage_ != ClosedLoopController::CALIBRATION_IDLE)
    {
        return;
    }
    if (c.driver_ == nullptr || c.encoder_ == nullptr
        || c.fault_monitor_.encoderFaultActive() || c.fault_monitor_.magneticFaultActive())
    {
        printf("[CALIBRATION] start rejected: driver/encoder unavailable or fault active\r\n");
        return;
    }

    c.encoder_->clearCalibration();  // 避免raw读成以前校准表的结果影响新校准

    // 【必须独占输出通道】start() 靠 setTargetAngleDeg(0) 把电机拉到零点，
    // 而 setTargetAngleDeg 的第一行就是“外部跟随开着就忽略”——
    // 如果用户当时在“外部脉冲运动” tab（0x149=1），这里会静默返回：
    //   closed_loop_angle_mode_enabled_ 没置位、motion_running_ 没置位 -> 电机不动，
    //   而 HOMING 阶段只是“等编码器回到 0°”，它自己不会驱动电机，
    //   于是 position_error 一直很大 -> stable_count 永远为 0 ->
    //   曾表现为只打一句 homing to zero 就卡死（当时连超时都不报）。
    // 所以这里先强制关掉外部跟随，之后再走角度模式。
    if (c.external_pulse_follower_.isEnabled())
    {
        c.external_pulse_follower_.setEnabled(false);
        c.external_pulse_follower_.setPrimePending(false);
        printf("[CALIBRATION] external pulse follow disabled for calibration\r\n");
    }

    c.step_pulse_width_ns_ = 200U;
    c.k_step_pulse_ticks = std::ceil(static_cast<float>(c.step_pulse_width_ns_)
                                     / static_cast<float>(stepper_common::target_tick_time));
    // 【保证角度模式有限速可用】本函数最终靠 setTargetAngleDeg(0) 驱动电机回零，
    // 而 MotionPlanner::updateAngleMode 里的 max_rps = angle_max_rpm_ / 60：
    // 如果上位机从未设过 0x12F（angle_max_rpm_ 为 0），max_rps 就是 0，
    // PID 输出会被 fast_clamp 全部钳成 0 -> 电机完全不动，
    // 表现为“homing to zero 之后没任何动作”。加速度同理。
    if (c.angle_max_rpm_ <= 0.0f)
    {
        c.angle_max_rpm_ = 1200.0f;
    }
    if (c.angle_accel_rpm_s_ <= 0.0f)
    {
        c.angle_accel_rpm_s_ = 6000.0f;
    }
    // 备份当前运动参数，finish() 时恢复（两边共用 controller 这一份）
    c.calibration_saved_start_rpm_ = c.motion_start_rpm_;
    c.calibration_saved_max_rpm_ = c.motion_max_rpm_;
    c.calibration_saved_accel_rpm_s_ = c.motion_accel_rpm_s_;
    c.calibration_saved_pulse_count_ = c.motion_pulse_count_;
    c.calibration_saved_pulse_width_ns_ = c.step_pulse_width_ns_;
    c.calibration_saved_motion_mode_ = c.motion_mode_;
    c.calibration_saved_compensation_enabled_ = c.closed_loop_compensation_enabled_;
    // 【校准期间必须纯开环】这里的置 false 只是给校准后留个干净初值，**它本身不够**：
    // 下面 setTargetAngleDeg(0) 归零会又把 closed_loop_compensation_enabled_ 置回 true
    //（既有行为），而校准中已禁止主机写 0x107，结果它全程都是 true。
    // 实测（仅靠这个置 false 时）：comp_rps_peak=0.5916 rps、emit bias windows=8091、
    // 每段多发 ≈1.4 微步、整圈进给 +2.85%。真正的保证在 rampUpdate 的补偿段门禁里：
    // calibration_stage_ != CALIBRATION_IDLE 时补偿段根本不运行。
    c.closed_loop_compensation_enabled_ = false;
    c.compensated_rps_ = 0.0f;
    // 校准专用状态（本类持有）。注意：和原实现一样不重置 home_wait_ticks_，
    // HOMING 的超时计数会带上一次残留（最多 1000 拍 = 50ms），行为保持一致。
    // 校验状态归零；顺带报一下“每段能不能整除”
    write_prev_valid_ = false;
    mono_bad_ = 0U;
    mono_worst_dev_ = 0;
    mono_worst_index_ = 0U;
    write_at_first_ = 0U;
    write_at_last_ = 0U;
    // 正/反向差值统计归零
    fb_sum_d_ = 0;
    fb_sum_i_ = 0;
    fb_sum_id_ = 0;
    fb_sum_ii_ = 0;
    fb_max_abs_d_ = 0;
    fb_max_abs_index_ = 0U;
    fb_pairs_ = 0U;
    // 脉冲对账基线：记下当前实发累计值（含归零前所有历史脉冲，取差即可）
    pulse_counter_base_ = c.step_pulses_emitted_;
    commanded_pulses_ = 0U;
    commanded_segments_ = 0U;
    // 逐段对账归零（显式清零，不依赖成员初始化的时机）
    seg_active_ = false;
    seg_pulse_base_ = 0U;
    seg_commanded_ = 0U;
    seg_ticks_ = 0U;
    seg_ticks_total_ = 0ULL;
    seg_index_ = 0U;
    seg_diff_min_ = 0;
    seg_diff_max_ = 0;
    seg_diff_max_at_ = 0U;
    seg_over_ = 0U;
    seg_under_ = 0U;
    seg_counted_diff_sum_ = 0;
    seg_emit_vs_count_sum_ = 0;
    // 发射偏差观测归零（在 controller 里，由 emitSteps 在校准 MOVING 阶段填报）
    c.calib_bias_windows_ = 0U;
    c.calib_bias_sum_ = 0;
    c.calib_comp_rps_peak_ = 0.0f;
    seg_diff_sum_ = 0U;
    for (uint32_t i = 0U; i < 7U; ++i)
    {
        seg_hist_[i] = 0U;
    }
    const uint32_t spr_now = c.getStepsPerRound();
    if ((spr_now % CALIBRATION_TABLE_SIZE) != 0U)
    {
        // 注意：段长交替（比如 12/13）**不累积错位** —— startMove 用的是边界公式
        // first_step = round(index * spr / N)，转子精确落在 round(i*spr/N)，
        // 误差上界 0.5 微步。报出来只是为了让“每段不完全等长”透明可见；
        // 想整除要把细分调到让 steps/round 成为整数倍（例如 128 细分→ 25600/1024=25）。
        printf("[CALIBRATION] steps/round=%lu %% 点数 != 0, segments alternate %lu/%lu microsteps\r\n",
               static_cast<unsigned long>(spr_now),
               static_cast<unsigned long>(spr_now / CALIBRATION_TABLE_SIZE),
               static_cast<unsigned long>(spr_now / CALIBRATION_TABLE_SIZE + 1U));
    }
    index_ = 0U;
    backward_pass_ = false;   // 从正向趟开始
    home_stable_count_ = 0U;
    home_previous_angle_ = c.encoder_->readFilteredAngle();
    c.calibration_stage_ = ClosedLoopController::CALIBRATION_HOMING;
    c.setTargetAngleDeg(0.0f);
    printf("[CALIBRATION] homing to zero\r\n");
}

// 结束校准并恢复运动参数（对应原 finishEncoderCalibration）。
// 备份值由本类 start() 写进 controller 的 calibration_saved_*，这里读回来恢复；
// 只存一份，不会出现两边不一致。
void EncoderCalibrator::finish(bool save_table)
{
    ClosedLoopController& c = owner_;

    // 与原实现一致：先置 SAVING（避免保存期间被当作“已空闲”），结束时置 IDLE。
    c.calibration_stage_ = ClosedLoopController::CALIBRATION_SAVING;
    c.closed_loop_angle_mode_enabled_ = false;
    c.stopMotion();

    bool saved = false;
    // 先把正/反向差值的分布报出来，再判表能不能用（两行都是诊断，不影响判定）
    reportForwardBackward();
    // 命令脉冲 vs 实发脉冲对账：
    //   diff ≈ 0            -> 脉冲数没问题，+3.0% 是转子在每段停止时被负载拽走的
    //                          机械爬行（方向无关、逐段累积）-> 治“段间松手”，不是改代码
    //   diff ≈ +390 微步    -> 真的多发了脉冲，出在发射/记账路径（例如 remaining_steps
    //                          只钳 num_steps、却不钳记账值这类不对称）-> 纯软件 bug
    {
        const uint32_t emitted = c.step_pulses_emitted_ - pulse_counter_base_;
        const int32_t diff = static_cast<int32_t>(emitted) - static_cast<int32_t>(commanded_pulses_);
        const double per_segment = (commanded_segments_ > 0U)
            ? static_cast<double>(diff) / static_cast<double>(commanded_segments_) : 0.0;
        printf("[CAL] pulses: commanded=%lu emitted=%lu diff=%ld segments=%u per_seg=%+.3f\r\n",
               static_cast<unsigned long>(commanded_pulses_),
               static_cast<unsigned long>(emitted),
               static_cast<long>(diff),
               static_cast<unsigned>(commanded_segments_),
               per_segment);
    }
    // 最后一段（反向趟回到 index 0 那一步）在这里结账，然后报分布
    closeSegment();
    reportSegments();
    // 发射偏差到底出在哪：
    //   windows=0                  -> 两个累加器未分叉，多发不是这里来的（转去查记账/结束条件）
    //   windows>0 且 rps_peak>0     -> 补偿确实在跑，“校准期间关闭环”没关住
    //   windows>0 且 rps_peak==0    -> 两个累加器在钳位/递减路径上分叉（纯记账 bug）
    printf("[CAL] emit bias: windows=%lu sum=%ld comp_rps_peak=%.4f rps\r\n",
           static_cast<unsigned long>(c.calib_bias_windows_),
           static_cast<long>(c.calib_bias_sum_),
           static_cast<double>(c.calib_comp_rps_peak_));
    // 【先校验再保存】表不可信就不写 Flash。这次一个坏点就能把整段角度搞乱，
    // 而建完表原本一道检查都没有，问题才能藏这么久。
    if (save_table && !tablePlausible())
    {
        printf("[CALIBRATION] table REJECTED, not saved (see [CAL] lines above)\r\n");
        save_table = false;
    }
    if (save_table)
    {
        saved = c.encoder_ != nullptr && c.encoder_->saveCalibration();
    }

    // 恢复校准前备份的运动参数
    c.motion_start_rpm_ = c.calibration_saved_start_rpm_;
    c.motion_max_rpm_ = c.calibration_saved_max_rpm_;
    c.motion_accel_rpm_s_ = c.calibration_saved_accel_rpm_s_;
    c.motion_pulse_count_ = c.calibration_saved_pulse_count_;
    c.motion_mode_ = c.calibration_saved_motion_mode_;
    c.step_pulse_width_ns_ = c.calibration_saved_pulse_width_ns_;

    // 【恢复校准前的补偿开关原状】这里是“恢复”，不是“打开”：
    // 校准前关着就仍然关着，绝不在用户没要求的时候把闭环弄成开的。
    // 之所以现在可以安全恢复：校准期间 parameter_router 已经硬挡了 0x107 的写入，
    // 所以备份值就等于上位机此刻的意图 —— 以前“不恢复”的理由（会造成
    // “界面已取消勾选、固件里补偿却开着”的不一致）已经不存在了。
    c.closed_loop_compensation_enabled_ = c.calibration_saved_compensation_enabled_;
    // 校准全程开环，PID 积分/微分里留的是校准前的陈旧状态；恢复补偿前一律清零，
    // 与 0x107 写入口的处理保持一致（模式切换必须清积分，这是本项目反复踩过的坑）。
    c.position_pid_.resetIntegral();
    c.position_pid_.resetDeriv();
    c.velocity_pid_.resetIntegral();
    c.velocity_pid_.resetDeriv();
    printf("[CAL] compensation restored=%u\r\n", c.closed_loop_compensation_enabled_ ? 1U : 0U);
    // 把固件里的真实状态回灌给上位机：rx.js 会用 0x0107 的回报帧同步“闭环使能”勾选框，
    // 所以即使校准期间用户点过勾选框（被固件挡掉），界面也会立即弹回真值。
    if (c.protocol_ != nullptr)
    {
        c.protocol_->setCustomParameter(TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE,
                                        c.closed_loop_compensation_enabled_ ? 1U : 0U);
    }

    c.calibration_stage_ = ClosedLoopController::CALIBRATION_IDLE;
    printf("[CALIBRATION] finished saved=%u\r\n", saved ? 1U : 0U);
}

// 把转子移动到第 index 个校准点（对应原 startEncoderCalibrationMove）。
// 每段只走 steps_per_round/CALIBRATION_TABLE_SIZE 步（64 细分、256 点下整 50 微步）。
bool EncoderCalibrator::startMove(uint16_t index)
{
    ClosedLoopController& c = owner_;

    // 开新段之前先给上一段结账：此时上一段后面已经跑完整个静止+采样过程，
    // 它那一窗的脉冲肯定已全部发完，取差不会把“还在 DMA 里”的步数算漏。
    closeSegment();

    // 【每 128 点才打一次进度】原来每段都打，带 %f 约 100 字节/段，
    // 1024 段就是 100KB，115200 下约 10 秒纯串口输出。
    if ((index % 128U) == 0U)
    {
        printf("[CALIBRATION] moving to index: %u from angle: %f\r\n", index, c.encoder_->readFilteredAngle());
    }

    // 用 controller 的 public 方法，避免依赖 closed_loop_controller.cpp 里的内部函数
    // getMicroStepsPerRound（它在那个 TU 里，头文件不暴露）。
    const uint32_t steps_per_round = c.getStepsPerRound();
    const uint64_t table_size = CALIBRATION_TABLE_SIZE;
    const uint32_t first_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t next_step = static_cast<uint32_t>(
        (static_cast<uint64_t>(index + 1U) * steps_per_round + table_size / 2U) / table_size);
    const uint32_t steps = next_step - first_step;
    if (steps == 0U)
    {
        printf("[CALIBRATION] unsupported microstep resolution\r\n");
        finish(false);
        return false;
    }

    c.motion_start_rpm_ = 60.0f;
    c.motion_max_rpm_ = 60.0f;
    // 【加速度必须足够大】每段只走一整格（256 点/64 细分下 50 微步），而 startMotion
    // 按梯形算减速点 decel_total_steps_ = vmax^2/(2a)：
    //   60 RPM/s    -> 6400 步 ≫ 50 步：第一拍就进 DECEL，一步不发
    //   60000 RPM/s -> 6 步：吃掉一段的相当比例
    // 拉到 600000 RPM/s -> decel 取整 0 -> steps_to_decel_ = 0 -> 全程 CRUISE。
    // 注意：这里的“瞬间满速”不是 +2.85% 的原因（mode=0 实验已否掉越窗说）。
    c.motion_accel_rpm_s_ = 600000.0f;
    c.motion_pulse_count_ = steps;
    // 命令量累计（两趟都计）：与 step_pulses_emitted_ 的增量对账用
    commanded_pulses_ += steps;
    if (commanded_segments_ < 0xFFFFU)
    {
        commanded_segments_++;
    }
    // 每段移动前清零等待计数器，让 SAMPLING 阶段先停 50ms 再采样
    home_wait_ticks_ = 0U;
    // 反向趟用 POSITION_REVERSE（startMotion 会据此把 motion_direction_ 设成 -1）
    c.motion_mode_ = backward_pass_
        ? ClosedLoopController::MOTION_MODE_POSITION_REVERSE
        : ClosedLoopController::MOTION_MODE_POSITION_FORWARD;
    c.step_pulse_width_ns_ = 200ULL;
    c.calibration_stage_ = ClosedLoopController::CALIBRATION_MOVING;
    c.motion_direction_ = backward_pass_ ? -1 : 1;
    c.startMotion();
    if (!c.motion_running_)
    {
        printf("[CALIBRATION] could not start move at index=%u\r\n", static_cast<unsigned>(index));
        finish(false);
        return false;
    }
    // 重新装填逐段对账（必须在 startMotion 之后：startMotion 会 stop() 掉残留序列，
    // 那不会发脉冲，但把基线定在它后面更干净）。
    seg_pulse_base_ = c.step_pulses_emitted_;
    seg_commanded_ = steps;
    seg_ticks_ = 0U;
    seg_active_ = true;
    return true;
}

// ===========================================================================
// 每拍推进校准状态机（对应原 updateEncoderCalibration），在 TMR4 控制循环里调用。
//
// 两个阶段：
//   HOMING    角度模式已把转子拉向 0 度；等“误差≤0.25° 且相邻拍变化≤0.20°”连续 100 拍
//             -> 退出角度模式、stopMotion、进 SAMPLING（index=0）
//             超时 100 万拍（约 50 秒）则 finish(false)，绝不清计数
//   SAMPLING 先停 1000 拍（50ms）让负载角稳定，再采 202 个样本、去掉最大最小后平均
//             -> 正向：只存数组；反向：与正向平均后写表
//             每采完一个点调 startMove() 走下一段，阶段由 updateTrapezoid 在走完后改回 SAMPLING
//
// 搬移时的成员映射（左：controller 原名，右：本类）：
//   calibration_stage_           -> c.calibration_stage_（保持单一真源，见头文件注释）
//   calibration_index_           -> index_
//   calibration_forward_raw_     -> forward_raw_      (kCalibrationPoints -> kPoints)
//   calibration_backward_pass_   -> backward_pass_
//   calibration_sample_count_/attempts_/sum_/min_/max_/anchor_ -> sample_count_/.../sample_anchor_
//   calibration_home_wait_ticks_/stable_count_/previous_angle_ -> home_wait_ticks_/home_stable_count_/home_previous_angle_
//   finishEncoderCalibration()   -> finish()      startEncoderCalibrationMove() -> startMove()
// ===========================================================================
void EncoderCalibrator::update(uint16_t raw)
{
    ClosedLoopController& c = owner_;

    // 逐段对账：数一下本段花了多少个控制拍（发射窗数的上界；窗口不够时会有
    // “本拍不发”的窗，所以这是上界而不是实窗数，足够区分分布了）。
    if (c.calibration_stage_ == ClosedLoopController::CALIBRATION_MOVING && seg_active_)
    {
        seg_ticks_++;
    }

    if (c.calibration_stage_ == ClosedLoopController::CALIBRATION_HOMING)
    {
        home_wait_ticks_++;
        // 【超时不能被清零】原来 else 分支里有一句 calibration_home_wait_ticks_ = 0，
        // 本意是“还在动就重新计时”，但代价是：只要电机一直没到零点（比如根本没被驱动），
        // 这个计数就永远涨不上去 -> 永远不超时 -> 整个校准卡在这里，
        // 日志只停在 [CALIBRATION] homing to zero，既不成功也不报错。
        // 现在只在“已经接近零点但还在微动”时才重置，否则让它自然累积到超时。
        if (home_wait_ticks_ >= 1000000U)
        {
            printf("[CALIBRATION] homing timeout (ticks=%lu)\r\n",
                   static_cast<unsigned long>(home_wait_ticks_));
            finish(false);
            return;
        }
        const float position = c.encoder_->readFilteredAngle();
        const float position_error = wrap_deg_180(-position);
        const float position_delta = wrap_deg_180(position - home_previous_angle_);
        home_previous_angle_ = position;
        // 【判据放宽】position_error 会被归一到 ±180，所以从 350° 方向归零时
        // 读到 359.99° 对应的误差是 +0.01°，原本就能通过。
        // 真正卡住的是 position_delta <= 0.02f：KTH7823 静止抖动实测 0.15° 左右，
        // 相邻两拍差值经常超过 0.02，稳定计数永远回到 0 —— 于是从 350° 归零会
        // “停在 359.99° 附近但永远不认为到零”，校准永远不开始。
        // 放宽到 0.20°（略高于噪声幅度），既不丢真实运动又能容忍抖动。
        if (fast_absf(position_error) <= 0.25f && fast_absf(position_delta) <= 0.20f)
        {
            if (home_stable_count_ < 100U)
            {
                home_stable_count_++;
            }
        }
        else
        {
            home_stable_count_ = 0U;
        }
        if (home_stable_count_ >= 100U)
        {
            c.closed_loop_angle_mode_enabled_ = false;
            c.stopMotion();
            beginSampling();
            home_wait_ticks_ = 0U;   // 重新用作“采样前等待”计数器
            printf("[CALIBRATION] zero reached; sampling index=0\r\n");
        }
        return;
    }

    if (c.calibration_stage_ != ClosedLoopController::CALIBRATION_SAMPLING)
    {
        return;
    }

    // 【每个位置先停一下再采样】运动刚停时转子还在找平衡（负载角），
    // 立刻采样会把“还没停稳”的角度记进表里，造成系统性偏差。
    // 复用 home_wait_ticks_ 当等待计数器（进 SAMPLING / 每段移动前已清零）。
    // 时长代价：每点约 (kSettleTicks + kSamplesNeeded) 个拍 = 80+15 = 95ms，
    // × 2047 个点 ≈ 3.3 分钟（原 50ms+10ms 约 2 分钟）。校准不是例行操作，慢一点可以接。
    if (home_wait_ticks_ < kSettleTicks)
    {
        home_wait_ticks_++;
        // 同时把可能存在的运动状态收尾，避免停顿期间残留速度
        c.current_step_speed_ = 0.0f;
        return;
    }

    sample_attempts_++;
    // 【按总线应答判有效性，不按值】原来这里写的是 raw != 0xFFFF，两个错：
    //   1) 校准点落在接缝上时，真角 359.9x° 的合法样本会被连续丢弃（实测同一节点 27 次），
    //      而去极值平均一旦只丢高侧，均值就被系统性拉低 —— 表在绕回处就会偏；
    //   2) 真正的 SPI 超时样本经过校准表映射后值已变，反而会被当有效值写进表。
    if (c.encoder_ != nullptr && c.encoder_->lastReadValid())
    {
        int32_t unwrapped = static_cast<int32_t>(raw);
        if (sample_count_ > 0U)
        {
            int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(sample_anchor_);
            if (delta > 32767)
            {
                delta -= 65536;
            }
            else if (delta < -32768)
            {
                delta += 65536;
            }
            unwrapped = static_cast<int32_t>(sample_anchor_) + delta;
        }
        else
        {
            sample_anchor_ = raw;
        }
        sample_sum_ += unwrapped;
        if (unwrapped < sample_min_)
        {
            sample_min_ = unwrapped;
        }
        if (unwrapped > sample_max_)
        {
            sample_max_ = unwrapped;
        }
        sample_count_++;
    } else
    {
        printf("[CALIBRATION] spi timeout at index=%u, attempt=%u\r\n",
            static_cast<unsigned>(index_),
            static_cast<unsigned>(sample_attempts_));
    }

    if (sample_count_ < kSamplesNeeded)
    {
        if (sample_attempts_ >= kMaxAttempts)
        {
            printf("[CALIBRATION] insufficient encoder samples at index=%u\r\n",
                   static_cast<unsigned>(index_));
            finish(false);
        }
        return;
    }

    // 去极值平均：除数必须等于“样本数 - 2”，写两处数字迟早会不一致。
    const int32_t trimmed_mean =
        (sample_sum_ - sample_min_ - sample_max_) / static_cast<int32_t>(kSamplesForMean);
    const uint16_t mean_raw = static_cast<uint16_t>(static_cast<uint32_t>(trimmed_mean) & 0xFFFFU);
    // 正转趟只存不写表（等反向趟取平均），反向趟平均后再写表。
    if (!backward_pass_)
    {
        if (index_ < kPoints)
        {
            forward_raw_[index_] = mean_raw;
        }
        if (index_ + 1U >= CALIBRATION_TABLE_SIZE)
        {
            // 正向到底：切反向，从倒数第二个点开始往回走。
            // 【最后一个点必须在这里就写表】反向趟从 1022 往回，不会再到 1023，
            // 如果只存数组不写表，这个位置永远是空的，
            // checkCalTableComplete() 会返回 false -> saveCalibration() 失败
            // -> finish 打出 finished saved=0。
            // 1023 是“接近 360 度”的边界点，它没有反向采样值，直接用正向值。
            if (index_ < kPoints)
            {
                noteWrite(index_, forward_raw_[index_]);
                c.encoder_->feedCalibrationSample(index_, forward_raw_[index_]);
            }
            backward_pass_ = true;
            index_ = (CALIBRATION_TABLE_SIZE > 1U) ? static_cast<uint16_t>(CALIBRATION_TABLE_SIZE - 2U) : 0U;
            printf("[CALIBRATION] forward pass done, reversing (index=%u)\r\n",
                   static_cast<unsigned>(index_));
            startMove(index_);
            return;
        }
        index_++;
        startMove(index_);
        return;
    }

    // 反向趟：与正向采样取平均后写表
    {
        const uint16_t forward_raw = (index_ < kPoints) ? forward_raw_[index_] : mean_raw;
        // 此刻手里同时拿着“正向那一趟”和“反向这一趟”的原始值，差值统计必须在这里做：
        // 平均写进表之后两者就再也分不开了（这也是 closure 分不清负载角与逐段超差的原因）。
        if (index_ < kPoints)
        {
            noteForwardBackward(index_, forward_raw, mean_raw);
        }
        // 【必须做环形平均，不能直接相加除以 2】raw 是 0~65535 的环绕角度：
        // 正向读 65530、反向读 26 时，(65530 + 26) / 2 = 32778 —— 得到一个“半圈外”的数，
        // 而正确答案是 10。表里凡是两趟跨过 0/65536 的那个索引都会中招（一圈只有一个，
        // 实测那张表是 index 1005）。
        // 级联后果：这个坏点恰好逃过 reverseLookup 的 CALIBRATION_WRAP(32768) 接缝判据
        //（|65474 - 32778| = 32696 < 32768，只差 72），于是被当成正常的相邻两点参与插值，
        // 把 180° 宽的 raw 区间压进 64 LSB（0.35°）的真实角区间 —— 这才是 0° 附近
        // 出现 6~7° 假误差的根源（与使能、与 PID、与电气干扰都无关）。
        // 注：同一个点内部的 202 个样本已经用了展开（见上面的 sample_anchor_），
        // takeRawAverage() 也是环形平均；只有“两趟之间”这一步漏了。
        int32_t pass_delta = static_cast<int32_t>(mean_raw) - static_cast<int32_t>(forward_raw);
        while (pass_delta > 32768)
        {
            pass_delta -= 65536;
        }
        while (pass_delta < -32768)
        {
            pass_delta += 65536;
        }
        const uint16_t averaged = static_cast<uint16_t>(
            (static_cast<int32_t>(forward_raw) + pass_delta / 2) & 0xFFFF);
        noteWrite(index_, averaged);
        c.encoder_->feedCalibrationSample(index_, averaged);
    }
    if (index_ == 0U)
    {
        finish(true);
        return;
    }
    index_--;
    startMove(index_);
}

// ===========================================================================
// 每写一格就检查与上一格的步进是否接近 64 LSB，并留下首尾两格算闭合。
//
// 两趟的写入方向相反（正向只在 1023 写一次，反向 1022→0），所以步进符号也会翻，
// 这里取绝对值比较“每格推进量的大小”。
// 拿旧那张 1024 点的坏表验算：1004→1005 的步进 = wrap(32778-65474) = -32696，
// 与理想格宽差 32632 ≫ 阈值 → 一抓一个准；而正常表的每格步进只偏离理想值几 LSB，
// 不会误报（阈值 = 4 倍理想格宽，随点数自动缩放）。
// ===========================================================================
void EncoderCalibrator::noteWrite(uint16_t index, uint16_t value)
{
    if (write_prev_valid_)
    {
        int32_t step = static_cast<int32_t>(value) - static_cast<int32_t>(write_prev_value_);
        while (step > 32768)
        {
            step -= 65536;
        }
        while (step < -32768)
        {
            step += 65536;
        }
        const int32_t magnitude = (step < 0) ? -step : step;
        const int32_t dev = magnitude - kIdealStep;
        const int32_t abs_dev = (dev < 0) ? -dev : dev;
        const int32_t abs_worst = (mono_worst_dev_ < 0) ? -mono_worst_dev_ : mono_worst_dev_;
        if (abs_dev > kStepToleranceLsb)
        {
            mono_bad_++;
        }
        if (abs_dev > abs_worst)
        {
            mono_worst_dev_ = dev;
            mono_worst_index_ = index;
        }
    }
    write_prev_valid_ = true;
    write_prev_value_ = value;
    if (index == 0U)
    {
        write_at_first_ = value;
    }
    if (index + 1U >= CALIBRATION_TABLE_SIZE)
    {
        write_at_last_ = value;
    }
}

// 反向趟里每写一个节点顺手记一笔：d = 反向原始值 - 正向原始值（环形折回 ±32768）。
// 不另外开数组：这一刻两份数据都在手上，平均完就分不开了。
void EncoderCalibrator::noteForwardBackward(uint16_t index, uint16_t fwd, uint16_t bwd)
{
    int32_t d = static_cast<int32_t>(bwd) - static_cast<int32_t>(fwd);
    while (d > 32768)
    {
        d -= 65536;
    }
    while (d < -32768)
    {
        d += 65536;
    }
    const int64_t i = static_cast<int64_t>(index);
    fb_sum_d_ += d;
    fb_sum_i_ += i;
    fb_sum_id_ += i * d;
    fb_sum_ii_ += i * i;
    fb_pairs_++;
    const int32_t abs_d = (d < 0) ? -d : d;
    const int32_t abs_worst = (fb_max_abs_d_ < 0) ? -fb_max_abs_d_ : fb_max_abs_d_;
    if (abs_d > abs_worst)
    {
        fb_max_abs_d_ = d;
        fb_max_abs_index_ = index;
    }
}

// ===========================================================================
// 报出 d(i) 的均值与斜率 —— 这两个数分别对应两件完全不同的事：
//   mean(d)/2 = 静态负载角（两趟符号相反，已被平均抵掉；对阶段 1 标定直接有用）
//   slope(d)  ≠ 0  则说明存在逐段累积、与方向相关的超差（平均抵不掉，
//              而且会把 closure 做成假数字）
// ===========================================================================
void EncoderCalibrator::reportForwardBackward() const
{
    const double lsb_deg = 360.0 / 65536.0;
    if (fb_pairs_ == 0U)
    {
        printf("[CAL] fwd-vs-bwd: no paired samples\r\n");
        return;
    }
    const double n = static_cast<double>(fb_pairs_);
    const double mean_d = static_cast<double>(fb_sum_d_) / n;
    const double den = n * static_cast<double>(fb_sum_ii_)
                     - static_cast<double>(fb_sum_i_) * static_cast<double>(fb_sum_i_);
    const double slope_per_node = (den != 0.0)
        ? (n * static_cast<double>(fb_sum_id_)
           - static_cast<double>(fb_sum_i_) * static_cast<double>(fb_sum_d_)) / den
        : 0.0;

    printf("[CAL] fwd-vs-bwd n=%u mean=%+.3fdeg (static load angle %+.3fdeg) "
           "slope=%+.3fdeg/turn max=%+.3fdeg@i=%u\r\n",
           static_cast<unsigned>(fb_pairs_),
           mean_d * lsb_deg,
           mean_d * 0.5 * lsb_deg,
           slope_per_node * static_cast<double>(CALIBRATION_TABLE_SIZE) * lsb_deg,
           static_cast<double>(fb_max_abs_d_) * lsb_deg,
           static_cast<unsigned>(fb_max_abs_index_));
    // closure 的端点不对称：v0 是两趟平均，v_last 只用了正向单程值（当初为了不留空洞）。
    // 所以 closure 里混进了“一个节点的方向差”，判读时记得扣除。
    printf("[CAL] endpoints: v0=%u(both passes avg) v_last=%u(FORWARD ONLY) "
           "-> closure includes one node's direction bias\r\n",
           static_cast<unsigned>(write_at_first_),
           static_cast<unsigned>(write_at_last_));
}

// ===========================================================================
// 表的合理性判定：闭合偏差 + 逐格步进单调性。不通过则 finish() 不写 Flash。
//
// 闭合为什么是硬错：命令 1023 段共 1023×12.5 微步（不足一圈），传感器应推进
// 1023×64 = 65472 LSB。多推/少推都说明“命令进给≠实际进给”。
// 实测那张表的闭合偏差是 +1396 LSB（+7.67°），正好对应“正转多走了约 7°才换向”。
// ===========================================================================
bool EncoderCalibrator::tablePlausible() const
{
    const int32_t nodeCount = static_cast<int32_t>(CALIBRATION_TABLE_SIZE);
    // 命令扫完整圈（N-1 段）时，传感器应推进 (N-1) * 理想格宽。
    // 实测旧那张 1024 点的表多推了 1396 LSB（+7.67°），正好对应“正转多走了约 7 度才换向”。
    // 现在这个差值已被 createFastCal 的比例归一化吸收（不会再产生悬崖），
    // 但必须报出来：它属于“命令进给与传感器对一圈的定义不一致”。
    const int32_t expect = kExpectLsb;

    int32_t closure = static_cast<int32_t>(write_at_last_) - static_cast<int32_t>(write_at_first_) - expect;
    // 未折叠的实际推进量（只用于打印）：折叠后的值有 ±65536 歧义（差几圈看不出来），
    // 把端点与未折叠差值一起打出来就能直接反推“实际/命令”的比例。
    const int32_t advance_lsb = static_cast<int32_t>(write_at_last_) - static_cast<int32_t>(write_at_first_);
    // 必须用 while 折返：跨接缝时这个值可能是 -129676 这种两圈外的数。
    while (closure > 32768)
    {
        closure -= 65536;
    }
    while (closure < -32768)
    {
        closure += 65536;
    }
    const int32_t abs_closure = (closure < 0) ? -closure : closure;
    const double lsb_deg = 360.0 / 65536.0;

    printf("[CAL] v0=%u v_last=%u advance=%ld LSB expect=%ld\r\n",
           static_cast<unsigned>(write_at_first_),
           static_cast<unsigned>(write_at_last_),
           static_cast<long>(advance_lsb),
           static_cast<long>(expect));
    printf("[CAL] closure=%+.3fdeg (%+ld LSB, limit %.1fdeg)  steps/round=%lu\r\n",
           static_cast<double>(closure) * lsb_deg,
           static_cast<long>(closure),
           static_cast<double>(kClosureToleranceLsb) * lsb_deg,
           static_cast<unsigned long>(owner_.getStepsPerRound()));
    printf("[CAL] monotonic: %u bad steps, worst |step-%ld|=%+ld LSB at index=%u (limit %ld)\r\n",
           static_cast<unsigned>(mono_bad_),
           static_cast<long>(kIdealStep),
           static_cast<long>(mono_worst_dev_),
           static_cast<unsigned>(mono_worst_index_),
           static_cast<long>(kStepToleranceLsb));

    // 【局部坏点硬拒；闭合偏差只报】
    // 上一版把闭合也设为硬拒，在当时是对的：旧建表算法（逐 raw 反查 + first-match）
    // 无法表示不闭合，头尾重叠就会出 352° 悬崖。
    // 现在 createFastCal 改成“展开 + 比例归一化后正向涂表”，任何小于半圈的漂移
    // 都不会再产生悬崖，于是闭合偏差回到它真正的位置：它是“命令进给与传感器角度
    // 对一圈的定义不一致”（本例 +1377 LSB = 2.1%），属于运动刻度的事，
    // 必须报但不能因此不让写表（否则在查清它之前根本没法校准）。
    bool ok = true;
    if (mono_bad_ != 0U)
    {
        printf("[CAL] REJECT: %u bad steps (worst |step-64|=%+ld LSB at index=%u, limit %ld)\r\n",
               static_cast<unsigned>(mono_bad_),
               static_cast<long>(mono_worst_dev_),
               static_cast<unsigned>(mono_worst_index_),
               static_cast<long>(kStepToleranceLsb));
        ok = false;
    }
    if (abs_closure > kClosureToleranceLsb)
    {
        // 归一化已经吃掉这部分比例，表仍可用；把这两个数字完整报出来，
        // 供下一步判定它到底来自“机械多走”还是“编码器刻度”。
        printf("[CAL] WARN: scale mismatch %+ld LSB/rev (%+.3f%% of a turn). "
               "Removed by normalization in the fast table; investigate it in the motion domain.\r\n",
               static_cast<long>(closure),
               static_cast<double>(closure) * 100.0 / 65472.0);
    }
    return ok;
}

// ===========================================================================
// 逐段对账：给刚结束的那一段算 diff = 实发脉冲 - 命令脉冲，并记进分布。
// 只在任务/中断上下文做十几次算术，不打印（printf 绝不进控制拍）。
// ===========================================================================
void EncoderCalibrator::closeSegment()
{
    if (!seg_active_)
    {
        return;
    }
    const uint32_t emitted_now = owner_.step_pulses_emitted_;
    const int32_t diff = static_cast<int32_t>(emitted_now - seg_pulse_base_)
                       - static_cast<int32_t>(seg_commanded_);

    if (seg_index_ == 0U)
    {
        seg_diff_min_ = diff;
        seg_diff_max_ = diff;
    }
    else
    {
        if (diff < seg_diff_min_)
        {
            seg_diff_min_ = diff;
        }
        if (diff > seg_diff_max_)
        {
            seg_diff_max_ = diff;
            seg_diff_max_at_ = seg_index_;
        }
    }
    if (diff > 0)
    {
        seg_over_++;
        seg_diff_sum_ += static_cast<uint32_t>(diff);
    }
    else if (diff < 0)
    {
        seg_under_++;
    }

    // 三分解：把“命令→记账→实发”中间那一环也记下来。motion_steps_emitted_ 在
    // startMotion() 里被清零，而本段结束后没人再动它（下一段才重清），所以此刻它
    // 就是本段的记账值。
    {
        const int32_t counted_now = static_cast<int32_t>(owner_.motion_steps_emitted_);
        const int32_t emitted_this = static_cast<int32_t>(emitted_now - seg_pulse_base_);
        seg_counted_diff_sum_ += counted_now - static_cast<int32_t>(seg_commanded_);
        seg_emit_vs_count_sum_ += emitted_this - counted_now;
    }

    // 直方图：≤-2 / -1 / 0 / +1 / +2 / +3..5 / ≥+6
    uint32_t bucket;
    if (diff <= -2)
    {
        bucket = 0U;
    }
    else if (diff == -1)
    {
        bucket = 1U;
    }
    else if (diff == 0)
    {
        bucket = 2U;
    }
    else if (diff == 1)
    {
        bucket = 3U;
    }
    else if (diff == 2)
    {
        bucket = 4U;
    }
    else if (diff <= 5)
    {
        bucket = 5U;
    }
    else
    {
        bucket = 6U;
    }
    seg_hist_[bucket]++;

    seg_ticks_total_ += seg_ticks_;
    seg_index_++;
    seg_active_ = false;
}

// 把分布报成两行（表质量与脉冲对账的判读都在这里）
void EncoderCalibrator::reportSegments() const
{
    if (seg_index_ == 0U)
    {
        printf("[CAL] segments: none closed\\r\n");
        return;
    }
    const double n = static_cast<double>(seg_index_);
    const double avg_ticks = static_cast<double>(seg_ticks_total_) / n;
    const double avg_over = (seg_over_ > 0U)
        ? static_cast<double>(seg_diff_sum_) / static_cast<double>(seg_over_) : 0.0;
    printf("[CAL] seg diff: n=%lu min=%ld max=%ld(max@%lu) over=%lu under=%lu avg_over=%+.2f\r\n",
           static_cast<unsigned long>(seg_index_),
           static_cast<long>(seg_diff_min_),
           static_cast<long>(seg_diff_max_),
           static_cast<unsigned long>(seg_diff_max_at_),
           static_cast<unsigned long>(seg_over_),
           static_cast<unsigned long>(seg_under_),
           avg_over);
    printf("[CAL] seg hist: le-2:%lu m1:%lu zero:%lu p1:%lu p2:%lu p3_5:%lu ge6:%lu | ticks/seg=%.1f\r\n",
           static_cast<unsigned long>(seg_hist_[0]),
           static_cast<unsigned long>(seg_hist_[1]),
           static_cast<unsigned long>(seg_hist_[2]),
           static_cast<unsigned long>(seg_hist_[3]),
           static_cast<unsigned long>(seg_hist_[4]),
           static_cast<unsigned long>(seg_hist_[5]),
           static_cast<unsigned long>(seg_hist_[6]),
           avg_ticks);
    // 命令→记账→实发 的三段分解：哪一段不为 0，问题就在哪一环
    printf("[CAL] seg chain: (counted-commanded)=%ld (emitted-counted)=%ld\r\n",
           static_cast<long>(seg_counted_diff_sum_),
           static_cast<long>(seg_emit_vs_count_sum_));
}
