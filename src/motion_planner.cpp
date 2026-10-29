#include "motion_planner.h"

#include "closed_loop_controller.h"
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
