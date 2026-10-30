#include "motion_planner.h"

#include <cmath>

#include "closed_loop_controller.h"
#include "step_output_channel.h"
#include "stepper_driver.h"

namespace
{
    // 与 closed_loop_controller.cpp 里的同名实现保持一致。
    // 不能直接复用那边匿名的版本（内部链接，跨 TU 不可见）。
    float fast_clamp(float v, float lo, float hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    float fast_abs(float v)
    {
        return v < 0.0f ? -v : v;
    }

    // 把角度差归一化到 [-180, 180]。与原实现的 while 版本语义一致，
    // 同时把 360 的整数倍先除一下，避免大角度下循环几百次。
    float normalize_signed_angle_error_deg(float error_deg)
    {
        if (error_deg > 1080.0f || error_deg < -1080.0f)
        {
            error_deg -= 360.0f * static_cast<float>(static_cast<int32_t>(error_deg / 360.0f));
        }
        while (error_deg > 180.0f)
        {
            error_deg -= 360.0f;
        }
        while (error_deg < -180.0f)
        {
            error_deg += 360.0f;
        }
        return error_deg;
    }
}

MotionPlanner::MotionPlanner(ClosedLoopController& owner) : owner_(owner)
{
}

void MotionPlanner::updateAngleMode(uint64_t delta_ramp_ns)
{
    ClosedLoopController& c = owner_;

    const float dt = static_cast<float>(delta_ramp_ns) / 1.0e9f;
    // 以真实机械角度误差做闭环控制，单位是度。微分步数只影响输出脉冲密度，
    // 不改变 PID 的误差定义，因此不同驱动或不同细分设置不会让同一组 PID 失效。
    const float desired_error_deg =
        normalize_signed_angle_error_deg(c.target_angle_deg_ - c.motion_position_deg_);  // 目标-实际
    c.closed_loop_angle_error_deg_ = desired_error_deg;

    if (dt > 0.0f)
    {
        const float position_error_rev = desired_error_deg / 360.0f;
        // 根据误差角算出位置环的速度修正量
        const float position_reference_rps = c.position_pid_.update(position_error_rev, dt);
        const float base_speed_rps = c.current_step_speed_ / static_cast<float>(c.getStepsPerRound());
        // 总期望速度 = 基础速度 + 位置环修正量
        const float total_ref_rps = base_speed_rps + position_reference_rps;
        // 总期望速度和实际速度的速度差
        const float velocity_error_rps = total_ref_rps - (c.encoder_speed_rpm_ / 60.0f);
        // 根据速度差算出速度环修正量
        const float velocity_correction_rps = c.velocity_pid_.update(velocity_error_rps, dt);

        const float max_rps = c.angle_max_rpm_ / 60.0f;
        const float unconstrained_rps =
            fast_clamp(position_reference_rps + velocity_correction_rps, -max_rps, max_rps);

        const float target_direction = desired_error_deg >= 0.0f ? 1.0f : -1.0f;
        const float directed_rps = unconstrained_rps * target_direction < 0.0f ? 0.0f : unconstrained_rps;
        const float accel_limit_rps2 = c.angle_accel_rpm_s_ / 60.0f;
        const float previous_rps = c.motion_speed_rpm_ / 60.0f;
        const float max_delta_rps = accel_limit_rps2 * dt;
        const float desired_rps =
            fast_clamp(directed_rps, previous_rps - max_delta_rps, previous_rps + max_delta_rps);

        // 校准归零阶段：接近目标 0.25° 以内就强制停，避免 PID 在零点附近来回抖，
        // 影响采样。顺带把两个环和累加器一起清干净，进入 SAMPLING 时状态是确定的。
        const float calibration_rps =
            (c.calibration_stage_ == ClosedLoopController::CALIBRATION_HOMING &&
             fast_abs(desired_error_deg) <= 0.25f)
                ? 0.0f
                : desired_rps;
        if (c.calibration_stage_ == ClosedLoopController::CALIBRATION_HOMING && calibration_rps == 0.0f)
        {
            c.motion_step_accumulator_ = 0.0f;
            c.compensated_motion_step_accumulator_ = 0.0f;
            c.compensated_rps_ = 0.0f;
            c.position_pid_.resetIntegral();
            c.position_pid_.resetDeriv();
            c.velocity_pid_.resetIntegral();
            c.velocity_pid_.resetDeriv();
        }

        const uint32_t micro_steps_per_round = c.getStepsPerRound();
        c.current_step_speed_ = calibration_rps * static_cast<float>(micro_steps_per_round);
        c.motion_speed_rpm_ = calibration_rps * 60.0f;
        if (calibration_rps != 0.0f)
        {
            c.motion_direction_ = calibration_rps >= 0.0f ? 1 : -1;
        }
    }
}

bool MotionPlanner::updateTrapezoid(uint64_t delta_ramp_ns, uint64_t now_ns)
{
    ClosedLoopController& c = owner_;

    // ---------- 往返摇摆运动：到达单程步数就换向 ----------
    const bool alternating_motion = c.motion_mode_ == ClosedLoopController::MOTION_MODE_ALTERNATING_FORWARD ||
                                    c.motion_mode_ == ClosedLoopController::MOTION_MODE_ALTERNATING_REVERSE;
    if (alternating_motion && !c.motion_leg_reversed_ && c.motion_leg_pulse_count_ > 0U &&
        c.motion_steps_emitted_ >= c.motion_leg_pulse_count_ &&
        c.motion_steps_emitted_ < c.motion_pulse_count_)
    {
        c.motion_direction_ = static_cast<int8_t>(-c.motion_direction_);
        c.motion_leg_reversed_ = true;
        c.motion_direction_change_pending_ = true;
        c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_ACCEL;
        c.current_step_speed_ = 0.0f;
        c.motion_speed_rpm_ = 0.0f;
        c.motion_step_accumulator_ = 0.0f;
        c.compensated_motion_step_accumulator_ = 0.0f;
        c.compensated_rps_ = 0.0f;
        // 【换向必须重算减速点】steps_to_decel_ 是在 startMotion 里按“全程”算的，
        // 换向后剩余行程是全新的另一程。不重算的话：如果第一程快跑完时 steps_to_decel_
        // 已经把剩余脉冲数吃进去，换向后 remaining_steps 仍然 <= steps_to_decel_
        // -> 第一拍就直接进 DECEL -> 速度从 0 往下减（已经是 0）
        // -> DDA 累加不到步 -> 不发脉冲 -> 反向那一程提前停住，
        //   剩下的脉冲看上去就变成“单向转”了。
        c.steps_to_decel_ = c.decel_total_steps_;
        // 【评分屏蔽】换向瞬间的误差是物理必然（命令已反转、机械还在减速），
        // 100ms（2000 拍 @ 20kHz）足够涵盖减速 + 反向加速过程。
        // 不屏蔽的话这个几十度的误差会把评分 RMS 抬到几十度，整定器完全失效。
        c.motion_turnaround_skip_ramps_ = 2000U;
        printf("[MOTION] turnaround: leg_steps=%lu total=%lu dir=%d\r\n",
               static_cast<unsigned long>(c.motion_leg_pulse_count_),
               static_cast<unsigned long>(c.motion_pulse_count_),
               c.motion_direction_);
    }

    // ---------- 全部脉冲发完 ----------
    if (c.motion_steps_emitted_ >= c.motion_pulse_count_)
    {
        if (c.calibration_stage_ == ClosedLoopController::CALIBRATION_MOVING)
        {
            // 校准运动走完：直接停，并切到采样阶段
            c.motion_running_ = false;
            c.motion_first_run_ = false;
            c.current_step_speed_ = 0.0f;
            c.motion_speed_rpm_ = 0.0f;
            c.motion_step_accumulator_ = 0.0f;
            c.compensated_motion_step_accumulator_ = 0.0f;
            c.compensated_rps_ = 0.0f;
            c.calibration_stage_ = ClosedLoopController::CALIBRATION_SAMPLING;
            c.calibration_sample_count_ = 0U;
            c.calibration_sample_attempts_ = 0U;
            c.calibration_sample_sum_ = 0;
            c.calibration_sample_min_ = 0x7FFFFFFF;
            c.calibration_sample_max_ = (-0x7FFFFFFF - 1);
            c.syncProtocolTelemetry();
            return true;
        }
        printf("motion_steps_emitted_: %lu, motion_pulse_count_: %lu\n",
               static_cast<unsigned long>(c.motion_steps_emitted_),
               static_cast<unsigned long>(c.motion_pulse_count_));
        // 关键：这里不能直接 stopMotion()。最后一个窗口的脉冲才刚进 TMR2/DMA 序列，
        // 立即关定时器会把尾巴上几十个脉冲连同对应角度一起砍掉（高速下约 0.36°），
        // 这就是上位机总是统计成“少了一点点角度”的原因。
        // 先挂起 DDA 并进入沉降阶段，等脉冲真正发完 + 轴静止，再由沉降块收尾。
        c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_SETTLE;
        c.current_step_speed_ = 0.0f;  // 运动结束强制清零，防止下次运动残留速度
        c.motion_speed_rpm_ = 0.0f;
        c.motion_step_accumulator_ = 0.0f;
        c.compensated_motion_step_accumulator_ = 0.0f;
        c.compensated_rps_ = 0.0f;
        c.motion_settle_start_ns_ = now_ns;
        c.motion_settle_stable_ramps_ = 0U;
        c.motion_settle_last_raw_ = c.encoder_filtered_raw_;
        c.motion_last_ramp_time_ns_ = now_ns;
        c.syncProtocolTelemetry();
        return true;
    }

    // ---------- 阶段推进：加速 / 匀速 / 减速 ----------
    // 剩余微步（往返模式下要算“当前这一程还剩多少”）
    const uint32_t remaining_total_steps = c.motion_pulse_count_ - c.motion_steps_emitted_;
    const uint32_t leg_steps_emitted = alternating_motion && c.motion_leg_reversed_
        ? c.motion_steps_emitted_ - c.motion_leg_pulse_count_
        : c.motion_steps_emitted_;
    const uint32_t remaining_steps = alternating_motion
        ? (c.motion_leg_reversed_ ? remaining_total_steps : c.motion_leg_pulse_count_ - leg_steps_emitted)
        : remaining_total_steps;
    const float dt_ramp_s = static_cast<float>(delta_ramp_ns) / 1.0e9f;

    if (c.motion_ramp_stage_ == ClosedLoopController::RAMP_STAGE_ACCEL)
    {
        c.current_step_speed_ += c.motion_accel_step_s2_ * dt_ramp_s;
        if (c.current_step_speed_ >= c.motion_max_step_s_)
        {
            c.current_step_speed_ = c.motion_max_step_s_;
            c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_CRUISE;
        }
        if (remaining_steps <= c.steps_to_decel_)
        {
            c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_DECEL;
        }
    }
    else if (c.motion_ramp_stage_ == ClosedLoopController::RAMP_STAGE_CRUISE)
    {
        if ((c.steps_to_decel_ != 0xFFFFFFFFU) && (remaining_steps <= c.steps_to_decel_))
        {
            c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_DECEL;
        }
    }
    else if (c.motion_ramp_stage_ == ClosedLoopController::RAMP_STAGE_DECEL)
    {
        c.current_step_speed_ -= c.motion_accel_step_s2_ * dt_ramp_s;
        if (c.current_step_speed_ < 0.0f)
        {
            c.current_step_speed_ = 0.0f;
        }
    }

    return false;
}

bool MotionPlanner::emitSteps(uint64_t delta_ramp_ns, uint64_t now_ns)
{
    ClosedLoopController& c = owner_;

    // 非角度模式：用 current_step_speed_ 反算实际转速（角度模式在步 A 里已经算过）
    if (!c.closed_loop_angle_mode_enabled_ && c.driver_ != nullptr)
    {
        const uint32_t micro_steps_per_round = c.getStepsPerRound();
        c.motion_speed_rpm_ = c.current_step_speed_ * 60.0f / static_cast<float>(micro_steps_per_round);
    }

    // ---------- DDA 微分累加器 ----------
    // 步进步数增量 = 瞬时速度(step/s) x 流逝时间(s)。
    // 即使任务被抢占延迟很久，delta_ramp_ns 记的是真实流逝时间，累加器会把该走的步数补上，
    // 调度抖动下也不会丢脉冲。
    const double dt_step_s = static_cast<double>(delta_ramp_ns) / 1.0e9;
    c.motion_step_accumulator_ += fast_abs(c.current_step_speed_) * dt_step_s;

    const uint32_t micro_steps_per_round = c.getStepsPerRound();
    const auto compensated_step_speed = c.compensated_rps_ * static_cast<float>(micro_steps_per_round);
    c.compensated_motion_step_accumulator_ +=
        fast_abs(c.current_step_speed_ + compensated_step_speed) * dt_step_s;

    // 角度模式：每拍最多走 1 步，避免大误差时一次吐出几十步
    if (c.closed_loop_angle_mode_enabled_ && c.motion_step_accumulator_ > 1.0)
    {
        c.motion_step_accumulator_ = 1.0;
        c.compensated_motion_step_accumulator_ = 1.0;
        c.compensated_rps_ = 0.0;
    }

    // ---------- 脉冲出口：铺 ARR 序列并启动 TMR2/DMA ----------
    // 和外部跟随走同一个 StepOutputChannel::emit：同一份缓冲区、同一套 spread/guard 逻辑。
    const bool alternating_motion = c.motion_mode_ == ClosedLoopController::MOTION_MODE_ALTERNATING_FORWARD ||
                                    c.motion_mode_ == ClosedLoopController::MOTION_MODE_ALTERNATING_REVERSE;

    if (!((c.compensated_motion_step_accumulator_ >= 1.0f) &&
          (c.closed_loop_angle_mode_enabled_ || c.motion_steps_emitted_ < c.motion_pulse_count_)))
    {
        return false;  // 本拍不需要发脉冲
    }

    auto num_steps = static_cast<uint32_t>(floor(c.compensated_motion_step_accumulator_));
    if (num_steps > c.max_step_per_ramp)
    {
        c.max_step_per_ramp = num_steps;
    }

    auto num_original_command_steps = static_cast<uint32_t>(floor(c.motion_step_accumulator_));
    if (!c.closed_loop_angle_mode_enabled_)
    {
        const uint32_t remaining_total_steps = c.motion_pulse_count_ - c.motion_steps_emitted_;
        const uint32_t leg_steps_emitted = alternating_motion && c.motion_leg_reversed_
            ? c.motion_steps_emitted_ - c.motion_leg_pulse_count_
            : c.motion_steps_emitted_;
        const uint32_t remaining_steps = alternating_motion
            ? (c.motion_leg_reversed_ ? remaining_total_steps : c.motion_leg_pulse_count_ - leg_steps_emitted)
            : remaining_total_steps;
        if (num_steps > remaining_steps)
        {
            num_steps = remaining_steps;
        }
    }
    if (num_steps > MAX_PULSE_ARR_LEN)
    {
        num_steps = MAX_PULSE_ARR_LEN;
    }

#ifdef USE_SOFT_PULSE
    // 软脉冲调试通道：直接 GPIO + 忙等，效率低，只在排查硬件时打开。
    for (uint32_t i = 0; i < num_steps; i++)
    {
        c.driver_->sendStepPulse(c.step_pulse_width_ns_);
    }
    c.motion_direction_change_pending_ = false;
#else
    if (!c.step_output_.emit(num_steps, c.motion_direction_ > 0, c.motion_direction_change_pending_,
                             delta_ramp_ns, c.k_step_pulse_ticks))
    {
        // 出口拒绍（窗口连头尾 guard 都放不下）：本窗不发，时间戳照样前移，
        // 累加器里的步数留到下一拍慢慢补，比硬塞一串超窗脉冲安全。
        c.motion_last_ramp_time_ns_ = now_ns;
        c.syncProtocolTelemetry();
        return true;
    }
    c.motion_direction_change_pending_ = false;
#endif

    if (!c.closed_loop_angle_mode_enabled_)
    {
        c.motion_steps_emitted_ += num_original_command_steps;
        c.motion_steps_emitted_signed += c.motion_direction_ * num_original_command_steps;
        // 本窗这批脉冲刚交给 DMA，要等下一拍确认真的发完，
        // 才并进 motion_steps_delivered_signed，用作跟随误差的命令位置。
        c.motion_dma_pending_signed = c.motion_direction_ * static_cast<int32_t>(num_original_command_steps);
    }
    c.motion_step_accumulator_ -= num_original_command_steps;
    c.compensated_motion_step_accumulator_ -= num_steps;

    return false;
}
