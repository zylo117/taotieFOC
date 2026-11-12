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

// ===========================================================================
// 角度模式（位置伺服）：把转子驱动到 target_angle_deg_
//
// 每 50us 执行一次，完整数据流：
//   误差(deg) = normalize(target - 当前位置)          // ±180
//      ↓ position_pid_（P 项 ×0.01，I/D 原值）
//   位置环输出 rps
//      ↓ + base_speed（上一拍输出，提供“记忆”/积分作用）
//   期望速度 rps
//      ↓ - 实测速度（encoder_speed_rpm_/60）
//   速度环（P 项同样 ×0.01）→ 修正量 rps
//      ↓ 相加后 fast_clamp(±angle_max_rpm_/60)
//   限幅后的 rps
//      ↓ 方向钳位（与误差反向时置 0，位置伺服不允许倒转）
//      ↓ 加速度限幅（angle_accel_rpm_s_）
//   compensated_rps_ → 下一步由 emitSteps 转成脉冲
//
// 两个已删除的隐藏环节（历史遗留，但它们是坑）：
//   1) kAnglePScale = 0.01：当年“PID 输出就是全部速度”，同样的 P 等效大 100 倍，
//      所以把 P 缩到 1/100；但同一个 kp 在规划运动补偿路径里不缩（p_scale=1），
//      于是一个数字在两条路径上等效增益差 100 倍，手感完全不一致。
//   2) base_speed_rps = current_step_speed_/spr：把“自己上一拍的输出”当基础速度，
//      等于给位置环偷加了一个积分器。它确实是“小 P 也能滚起来”的原因，但它积分的是
//      **速度反馈噪声**（单拍差分的残余噪声实测约 1.35 rps），稳态抖动≈σ·√(kp/2)：
//      kp 再小也抖，kp 调小又推不动；而且速度环 I 一开就发散（因为正反馈）。
// 现在的结构：位置环输出 v_ref（速度指令）→ 速度环只对 (v_ref − 实测) 做阻尼修正
// → 限幅/方向钳位/加速度钳位 → 微步/s。“尾巴收敛不到 0”该由位置环 ki 管（有 anti-windup、
// 模式切换会清），而不是靠自引用把噪声积起来。注意：改完之后原有增益需要重整定。
// ===========================================================================
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
        const float measured_motion_rps = c.encoder_speed_rpm_ / 60.0f;
        const float max_rps = c.angle_max_rpm_ / 60.0f;
        const float max_accel_rps2 = c.angle_accel_rpm_s_ / 60.0f;

        // ---------- 1) 梯形轮廓（速度前馈的来源）----------
        // 刚进入角度模式（或上次已停止）时从当前实际角度起步，避免轮廓凭空跳到位。
        if (!c.angle_profile_valid_)
        {
            c.angle_profile_pos_deg_ = c.motion_position_deg_;
            c.angle_profile_vel_rps_ = 0.0f;
            c.angle_profile_valid_ = true;
        }
        const float to_target_deg =
            normalize_signed_angle_error_deg(c.target_angle_deg_ - c.angle_profile_pos_deg_);
        const float profile_dir = (to_target_deg >= 0.0f) ? 1.0f : -1.0f;
        const float remaining_rev = fast_abs(to_target_deg) / 360.0f;
        // 平方根律：以恒定减速度刚好停在目标上所允许的最大速度。
        // 与“提前算减速点”等价，但不需额外状态，也不会因为目标跳变而算错。
        const float decel_cap_rps = (max_accel_rps2 > 0.001f)
            ? std::sqrt(2.0f * max_accel_rps2 * remaining_rev) : max_rps;
        float profile_desired_rps = profile_dir * ((max_rps < decel_cap_rps) ? max_rps : decel_cap_rps);
        // 轮廓速度本身也受加速度限制：前馈必须是连续的，不能一步跳到全速
        const float accel_cap = max_accel_rps2 * dt;
        profile_desired_rps = fast_clamp(profile_desired_rps,
                                         c.angle_profile_vel_rps_ - accel_cap,
                                         c.angle_profile_vel_rps_ + accel_cap);
        c.angle_profile_vel_rps_ = profile_desired_rps;
        // 度/s = rps × 360
        c.angle_profile_pos_deg_ += profile_desired_rps * 360.0f * dt;

        // ---------- 2) 前馈 + 跟踪 ----------
        // 位置环只看“轮廓位置 − 实际位置”（跟踪误差），不再看目标：
        // 走多快由前馈决定，跟得准不准才交给 PID。换目标时不再产生“误差是阶跃”的大 P 冲击。
        const float follow_err_deg =
            normalize_signed_angle_error_deg(c.angle_profile_pos_deg_ - c.motion_position_deg_);
        // 【两个环都必须限幅】与规划运动补偿段同一套规则：位置环最多给轮廓速度的 30%，
        // 速度环最多 20%。没有这一层时，网页上那个 30（历史口径，等效 0.3）会变成
        // 真·30 倍增益：v_ref 一步顶到限速帽，梯形轮廓的加减速被"补偿"整个吃掉 ——
        // 这正是"内部规划运动的加减速阶段消失、一上来就是最高转速"的直接原因。
        const float profile_abs_rps = fast_abs(profile_desired_rps);
        const float pos_limit = fast_clamp(profile_abs_rps * 0.30f, 0.25f, max_rps * 0.5f);
        c.position_pid_.setOutputLimit(pos_limit);
        // 【微分先行】D 对测量值求导（-measured_rate），不对误差求导：
        // 目标角改变时误差是阶跃，对误差微分会打一脚冲击（derivative kick）。
        float pos_corr = c.position_pid_.update(follow_err_deg / 360.0f, dt, 1.0f,
                                                measured_motion_rps, true);
        pos_corr = fast_clamp(pos_corr, -pos_limit, pos_limit);
        const float position_reference_rps = profile_desired_rps + pos_corr;

        // 速度环只对“指令速度 − 实测速度”做阻尼修正
        const float vel_limit = fast_clamp(profile_abs_rps * 0.20f, 0.15f, max_rps * 0.35f);
        c.velocity_pid_.setOutputLimit(vel_limit);
        const float velocity_error_rps = position_reference_rps - measured_motion_rps;
        const float measured_accel_rps2 =
            (measured_motion_rps - c.last_angle_mode_measured_rps_) / dt;
        c.last_angle_mode_measured_rps_ = measured_motion_rps;
        float velocity_correction_rps = c.velocity_pid_.update(velocity_error_rps, dt, 1.0f,
                                                               measured_accel_rps2, true);
        velocity_correction_rps = fast_clamp(velocity_correction_rps, -vel_limit, vel_limit);

        const float unconstrained_rps =
            fast_clamp(position_reference_rps + velocity_correction_rps, -max_rps, max_rps);

        // 【方向钳位：位置伺服不能反向输出】
        // 恢复原逻辑并说明为什么它是对的：
        //   PID 输出与误差方向相反时，说明 P 项已经被 D 项压到底了，
        //   此时正确的做法是“不再加速”，而不是“反向走”。
        //   允许负输出会让位置伺服真的倒转 -> 越转越远 -> 乱转。
        // （之前一度把它改成 unrestricted，那是错的，已回退。）
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
            // 归零已到位：轮廓也拉回当前位置并停下，否则下次进角度模式会从旧轮廓接着跑
            c.angle_profile_pos_deg_ = c.motion_position_deg_;
            c.angle_profile_vel_rps_ = 0.0f;
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
            const int8_t want_dir = (calibration_rps >= 0.0f) ? 1 : -1;
            if (want_dir != c.motion_direction_)
            {
                // 【角度模式换向必须挂 DIR 翻转标志】emitSteps 的 emit(direction_changed)
                // 靠它决定“先换 DIR 脚、等建立时间再发脉冲”。原来这里只改方向不发标志，
                // 首窗脉冲会在 DIR 还指着旧方向时发出去 —— 反向第一步变正向。
                c.motion_direction_ = want_dir;
                c.motion_direction_change_pending_ = true;
            }
        }
        // 【诊断快照 [VLOOP]】只存量、不打印（输出在 telemetry_reporter 的 kSlow 档）。
        // P/I/D 分开存，是为了看抖动到底由哪一项贡献（而不是先猜 kp）；
        // 这里用的是 c.velocity_pid_.kp（此刻真正在算的值），所以运行时改增益会直接露出来。
        {
            auto& d = c.vloop_;
            d.err_deg = follow_err_deg;
            d.ref_rps = position_reference_rps;
            d.meas_rps = measured_motion_rps;
            d.vel_err_rps = velocity_error_rps;
            d.pos_out_rps = position_reference_rps;
            d.prof_vel_rps = profile_desired_rps;
            d.vel_p = c.velocity_pid_.kp * velocity_error_rps;
            d.vel_i = c.velocity_pid_.ki * c.velocity_pid_.integral;
            d.vel_d = c.velocity_pid_.kd * c.velocity_pid_.filtered_derivative;
            d.out_rps = calibration_rps;
            d.step_speed_s = c.current_step_speed_;
            d.stamp++;
        }
    }
}

// ===========================================================================
// 梯形规划（位置/速度模式）：从 current_step_speed_ 出发，按
// ACCEL → CRUISE → DECEL → SETTLE 四阶段推进，返回 true 表示“本拍已处理，
// emitSteps 不用再跑”（换向/沉降这类需要独占拍的情况）。
//
// 阶段转换条件：
//   ACCEL   : speed < max_step_s -> 每拍 +accel*dt；到顶转 CRUISE
//   CRUISE  : remaining <= steps_to_decel_ -> 转 DECEL
//   DECEL   : speed > 0 -> 每拍 -accel*dt；到 0 且剩余 0 -> 转 SETTLE
//   SETTLE  : 等编码器静止（或 200ms 超时）-> stopMotion()
//
// 两处容易改错的地方（都已踩过坑）：
//   1) steps_to_decel_ 必须按“全程”算；换向后剩余行程是全新一程，必须重算，
//      否则第二程第一拍就进 DECEL，表现为反向提前停住。
//   2) 速度模式下运行时改 0x11B/0x11C/0x11E 时：提速回 ACCEL、降速必须每拍执行
//      （一拍只降 accel*dt = 64 步/s，只降一次等于没降）；换向要先刹到 0 再翻 DIR。
// ===========================================================================
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
        // 【换向必须清 PID 积分】同运行时换向分支：不清积分会让反向被旧积分抵消。
        c.position_pid_.resetIntegral();
        c.position_pid_.resetDeriv();
        c.velocity_pid_.resetIntegral();
        c.velocity_pid_.resetDeriv();
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

    // ---------- 快速停车请求（急刹冲击标定用）----------
    // 与 stopMotion() 的区别：stopMotion 是瞬间关脉冲（机械自由滑行），这里按
    // 0x11C 的加速度真正减速到 0，并在速度到 0 时就地收尾（不要求走完剩余脉冲）。
    // 这样整定器能在减速段继续采样，量到真正的"急刹冲击"峰值。
    if (c.quick_stop_requested_ && c.motion_running_ &&
        c.motion_ramp_stage_ != ClosedLoopController::RAMP_STAGE_SETTLE)
    {
        c.quick_stop_requested_ = false;
        c.motion_quick_stop_ = true;
        c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_DECEL;
        c.steps_to_decel_ = 0xFFFFFFFFU;   // 不靠"剩余步数"触发，只靠速度到 0
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
            // 切到采样阶段并重置采样计数。这些计数现在属于 EncoderCalibrator，
            // 不再直接写 controller 成员，改成显式调用握手方法（与校准器共用一份）。
            c.calibrator_.beginSampling();
            c.syncProtocolTelemetry();
            return true;
        }
        // 【已删 printf】这里原来每段运动收尾都无条件打 "motion_steps_emitted_/pulse_count"
        // （控制任务里约 8ms 停摆）。这两个数已经在 settle_report_ 的行程对账里，
        // 由 kSlow 档在真静止后统一打印。
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

    // ---------- 速度模式：运行时改目标转速 ----------
    // 背景：motion_max_step_s_ 只在 startMotion() 里按 motion_max_rpm_ 算一次，
    // 上位机后来改 0x11B 不会生效 —— 这就是“滞后角校准只能一段一段测、
    // 每点都要停机重启再加速”的根本原因。
    // 这里补上运行时变速：检测到目标变了就重算，并调整阶段让现有加减速逻辑自然过渡。
    // 只对速度模式（MOTION_MODE_VELOCITY_*）生效，位置模式不受影响。
    if (c.motion_mode_ == ClosedLoopController::MOTION_MODE_VELOCITY_FORWARD ||
        c.motion_mode_ == ClosedLoopController::MOTION_MODE_VELOCITY_REVERSE)
    {
        // 【运行时换向】上位机改 0x11E（正向<->反向）后，motion_direction_ 只在 startMotion
        // 里设过，不会自动跟 —— 这里检测模式与当前方向的差异，先减速到 0 再换向。
        // 直接反方向加速会让电机顿一下（电流冲击），先刹到 0 再反向才平滑。
        const bool want_forward = (c.motion_mode_ == ClosedLoopController::MOTION_MODE_VELOCITY_FORWARD);
        const bool is_forward = (c.motion_direction_ > 0);
        if (want_forward != is_forward)
        {
            const float dt_ramp_dir = static_cast<float>(delta_ramp_ns) / 1.0e9f;
            if (c.current_step_speed_ > 0.0f)
            {
                c.current_step_speed_ -= c.motion_accel_step_s2_ * dt_ramp_dir;
                if (c.current_step_speed_ < 0.0f)
                {
                    c.current_step_speed_ = 0.0f;
                }
            }
            if (c.current_step_speed_ <= 0.0f)
            {
                // 已经刹到 0，翻转方向；下一拍由 ACCEL 逻辑反向加速
                c.motion_direction_ = want_forward ? 1 : -1;
                c.motion_direction_change_pending_ = true;
                c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_ACCEL;
                // 【换向必须清 PID 积分与微分】否则：换向前 PID 在正向累积的积分项会继续推正向，
                // 而 motion_direction_ 已翻成 -1，两者相互抵消 -> 反向爬不起来。
                // 微分项同理：error 在换向瞬间突变 100°，(Δerror)/dt 直接到百万量级，
                // 虽然会被 clamp 削顶，但 filtered_derivative 里的残留会让换向后头几拍
                // 输出一个方向错误且巨大的力（实测峰值 177°）。
                c.position_pid_.resetIntegral();
                c.position_pid_.resetDeriv();
                c.velocity_pid_.resetIntegral();
                c.velocity_pid_.resetDeriv();
            }
            // 换向期间不做后续变速，避免一拍内既要刹车又要改目标
            c.motion_max_step_s_ = c.motion_max_rpm_ * static_cast<float>(c.getStepsPerRound()) / 60.0f;
            return false;
        }

        const uint32_t speed_spr = c.getStepsPerRound();
        const float new_max_step_s = c.motion_max_rpm_ * static_cast<float>(speed_spr) / 60.0f;

        // 【运行时改加速度】motion_accel_step_s2_ 同样只在 startMotion 算一次。
        // 阶段 2 的急刹测试需要把加速度临时放大 M 倍（0x11C = 原值 x M）
        // 来测“急启动 / 急刹”下的跟随误差，必须能在线改。
        // 注意：它同时影响提速和降速两侧的斜率，这是预期行为。
        const float new_accel_step_s2 = c.motion_accel_rpm_s_ * static_cast<float>(speed_spr) / 60.0f;
        if (new_accel_step_s2 > 0.0f && fast_abs(new_accel_step_s2 - c.motion_accel_step_s2_) > 1.0f)
        {
            c.motion_accel_step_s2_ = new_accel_step_s2;
        }
        if (new_max_step_s > 0.0f)
        {
            // 【目标变了】更新上限。提速靠下面的 ACCEL 分支自己追，不在这里处理。
            if (fast_abs(new_max_step_s - c.motion_max_step_s_) > 1.0f)
            {
                c.motion_max_step_s_ = new_max_step_s;
                if (c.current_step_speed_ < new_max_step_s &&
                    c.motion_ramp_stage_ != ClosedLoopController::RAMP_STAGE_ACCEL)
                {
                    c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_ACCEL;
                }
            }

            // 【降速必须每拍都做】不能只在“目标变化那一拍”降一次：
            // 250RPM -> 200RPM 差 10666 步/s，而一拍只降 accel x dt = 64 步/s，
            // 只执行一次等于完全没减速 —— 表现为“滞后角阶梯只能升不能降”。
            // ACCEL 分支每拍都在加，降速也必须每拍都在减，两者对称。
            if (c.current_step_speed_ > c.motion_max_step_s_ + 1.0f)
            {
                const float dt_slow = static_cast<float>(delta_ramp_ns) / 1.0e9f;
                c.current_step_speed_ -= c.motion_accel_step_s2_ * dt_slow;
                if (c.current_step_speed_ < c.motion_max_step_s_)
                {
                    c.current_step_speed_ = c.motion_max_step_s_;
                }
                c.motion_ramp_stage_ = ClosedLoopController::RAMP_STAGE_CRUISE;
            }
        }
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
        if (c.current_step_speed_ <= 0.0f)
        {
            if (c.motion_quick_stop_)
            {
                // 急刹完成：把"目标脉冲数"改成已发数，下一拍就走"全部发完 -> SETTLE"。
                // 注意必须走 else-if：否则紧接的爬行分支会立刻把速度又抬起来。
                c.current_step_speed_ = 0.0f;
                c.motion_quick_stop_ = false;
                c.motion_pulse_count_ = c.motion_steps_emitted_;
            }
            // 【死锁保护】速度已经减到 0，但还有脉冲没发完（剩余步数 > 0）。
            // 以前这里直接置 0 就不管了：状态机永远停在 DECEL，motion_running_ 一直是真，
            // 上位机等 0x11F 变 0 只能等到超时；电机也确实少走了那一截
            // （实测 2 圈运动停在 25220/25600，差 1.5%）。
            // 现在的处理：用一个小爬行速度把尾巴补完，让“全部发完 -> SETTLE”正常接管。
            else if (remaining_steps > 0U)
            {
                float creep = c.motion_start_step_s_ * 0.5f;
                if (creep < 400.0f)
                {
                    creep = 400.0f;          // 至少 400 微步/s，低于这个 emit 会长时间不足 1 步/拍
                }
                if (creep > c.motion_max_step_s_)
                {
                    creep = c.motion_max_step_s_;
                }
                c.current_step_speed_ = creep;
            }
            else
            {
                c.current_step_speed_ = 0.0f;
            }
        }
    }

    return false;
}

// ===========================================================================
// 把 accumulated 的步数（float，单位：步）变成实际脉冲序列。
// 返回 false = 本拍没有脉冲需要发（不会触发任何硬件动作）。
//
// 累加器语义：
//   motion_step_accumulator_            规划本身应该走的步数
//   compensated_motion_step_accumulator_ 叠加了 PID 补偿后的总步数
// 两者都按 speed × dt 累加，取 floor 得到整数步；取整后要把小数部分留在累加器里，
// 否则低速时每拍不足 1 步会被反复丢弃，掉速很严重。
//
// 角度模式的特殊处理：每拍最多走 kMaxStepsPerTick 步。
// 这个钳位同时是“防喷发”（大误差时一次发出几十步堵住 DMA）和“实际限速”：
// 8 步/拍 x 20kHz = 160000 步/s，按 12800 步/圈约 750 RPM；再往上的真限速
// 由 0x12F（角度最大速度）与 0x130（角度加速度）决定。
// ===========================================================================
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
    // 【用 float 不用 double】两个累加器已改成 float（见 header 注释）。
    // dt 也必须用 float，否则 float += double * double 会把整个表达式提升成 FP64 运算，
    // 白白回到软件模拟。
    // 【控制任务停顿保护：DDA 时间截断】
    // 正常窗口 dt≈50µs。当控制任务被慢档遥测/USB/SPI/Flash 抢走 CPU 时，dt 会
    // 突然变成几百µs 甚至 ms 级。DDA 若按真实 dt 全额补账，一窗要挤进的步数远超
    // 窗口能合法装下的量（如 96kpps 下 2ms 债务=192 步），输出退化为亚 µs 间隔
    // 爆发——TMC2209 输入级（最小周期 2µs）根本吃不进，转子丢同步后原地反抽，
    // 宏观就是"速度越高抖得越狠、450RPM 起必失步"（2026-11-12 实机定位）。
    // 截断到 150µs（3 窗）：转子此刻已注定掉步，让输出保持合法节拍、两个累加器
    // 用同一个截断后的 dt（记账与实际一致），闭环从真实位置继续，
    // 绝不再"卡一下猛地抽一下"。事件计数见 0x178。
    constexpr float kDtCapS = 150.0e-6f;
    float dt_step_s = static_cast<float>(delta_ramp_ns) * 1.0e-9f;
    if (dt_step_s > kDtCapS)
    {
        dt_step_s = kDtCapS;
        c.step_catchup_events_++;
    }
    c.motion_step_accumulator_ += fast_abs(c.current_step_speed_) * dt_step_s;

    const uint32_t micro_steps_per_round = c.getStepsPerRound();
    const auto compensated_step_speed = c.compensated_rps_ * static_cast<float>(micro_steps_per_round);
    // 修正量是“沿运动方向的附加速度”，基础速度是**脉冲频率大小**（>=0），相加后必须 >= 0：
    // 允许为负会让脉冲倒着发，而 DIR 脚已经固定，
    // 结果是脉冲照发、方向不变、转速却比规划快 —— 一个很难查的隐性超速。
    //
    // 【必须取绝对值：角度模式的基础速度是带符号的】
    // 规划运动的 current_step_speed_ 是大小（方向走 DIR 脚），但角度模式写进去的是带符号
    // 速度（见 updateAngleMode 末尾）。原来这里直接用带符号值：反向时 “基速(负)+补偿(0)”
    // 被钳成 0，补偿累加器永远涨不到 1，下面的发脉冲门（compensated_accumulator >= 1）
    // 永远不开 —— **反向的角度目标一个脉冲都不发**，只有 |速度| 超过约 750RPM 时才被
    // kMaxStepsPerTick 钳位误抬成 8 步/拍爆发泄漏（那次钳位也是错的，见下面独立夹取）。
    // 取绝对值后：方向只由 motion_direction_（DIR 脚）表达，脉冲频率恒为非负，
    // 规划运动路径 fast_abs 是恒等变换，行为不变。
    float total_step_speed = fast_abs(c.current_step_speed_) + compensated_step_speed;
    if (total_step_speed < 0.0f)
    {
        total_step_speed = 0.0f;
    }
    c.compensated_motion_step_accumulator_ += total_step_speed * dt_step_s;

    // 角度模式：每拍最多走 kMaxStepsPerTick 步。
    // 这个钳位同时是“防喷发”和“实际限速”：超出部分是被**丢弃**的（两个累加器都被直接
    // 压到上限），所以帽定得太低会让位置环在高误差段完全失去比例作用（v_ref 再大也出不去）。
    // 历史值：1 步/拍 = 20000 步/s ≈ 93 RPM（太低，1200RPM 设定根本达不到）；
    //            3 步/拍 = 60000 步/s ≈ 281 RPM —— 但 250 RPM 就已贴帽（2.67 步/拍），
    //            没有任何调节余量。
    // 现在 8 步/拍 = 160000 步/s ≈ 750 RPM（64 细分）。硬件上很宽：最小步距
    // 2×200ns=400ns → 一窗（50us）能装 125 步，MAX_PULSE_ARR_LEN=128 也没碰到。
    // 真正的限速回到它们该在的地方：0x12F（角度限速 RPM）与 0x130（加速度）。
    constexpr float kMaxStepsPerTick = 8.0f;
    if (c.closed_loop_angle_mode_enabled_)
    {
        // 【钳位必须各自独立夹，不能直接赋成 8】原实现两个累加器都写成 = kMaxStepsPerTick：
        // 补偿累加器本来不足 8 时，这一下等于“凭空抬到 8 步/拍”（160k 步/s）——
        // 反向时它是 0（见上面 fast_abs 修复的说明），这一钳位反而成了反向唯一
        // “泄漏脉冲”的通道，且一泄漏就是满速爆发。
        if (c.motion_step_accumulator_ > kMaxStepsPerTick)
        {
            c.motion_step_accumulator_ = kMaxStepsPerTick;
        }
        if (c.compensated_motion_step_accumulator_ > kMaxStepsPerTick)
        {
            c.compensated_motion_step_accumulator_ = kMaxStepsPerTick;
        }
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

    // ---------- 实发 vs 记账：两个累加器必须各记各的 ----------
    // 【本次修复的核心】旧实现把“记账值”钳到不超过“实发值”：
    //     if (num_original_command_steps > num_steps) num_original_command_steps = num_steps;
    // 它的本意是“没发出去的不能记成走过了”，但副作用更严重：只要补偿在**减速**
    // （PID 认为超前了），实发就小于规划，于是记账也跟着少记，规划累计永远到不了
    // pulse_count_ —— 表现为运动停在 25220/25600（差 1.5%）、状态机卡在 DECEL，
    // 上位机等 0x11F 变 0 只能等到超时。
    // 正确语义：
    //   记账 = "规划应该走到哪"（命令位置参考），只受剩余行程限制；
    //   实发 = "真正发了几个脉冲"，受剩余行程与出口容量限制。
    // 两者之差就是补偿量，跟随误差里自然看得出来。
    uint32_t remaining_steps = 0xFFFFFFFFU;
    if (!c.closed_loop_angle_mode_enabled_)
    {
        const uint32_t remaining_total_steps = c.motion_pulse_count_ - c.motion_steps_emitted_;
        const uint32_t leg_steps_emitted = alternating_motion && c.motion_leg_reversed_
            ? c.motion_steps_emitted_ - c.motion_leg_pulse_count_
            : c.motion_steps_emitted_;
        remaining_steps = alternating_motion
            ? (c.motion_leg_reversed_ ? remaining_total_steps : c.motion_leg_pulse_count_ - leg_steps_emitted)
            : remaining_total_steps;
    }

    // 实发步数：补偿累加器可能为负（补偿在刹车），此时本窗不发。
    uint32_t num_steps = 0U;
    if (c.compensated_motion_step_accumulator_ > 0.0f)
    {
        num_steps = static_cast<uint32_t>(std::floor(c.compensated_motion_step_accumulator_));
    }
    if (num_steps > c.max_step_per_ramp)
    {
        c.max_step_per_ramp = num_steps;
    }
    if (num_steps > remaining_steps)
    {
        num_steps = remaining_steps;
    }
    if (num_steps > MAX_PULSE_ARR_LEN)
    {
        num_steps = MAX_PULSE_ARR_LEN;
    }
    // 【合法速率夹取】本窗最多发 windowCapacity 步（2µs/步下限，TMC2209 输入级
    // 规格）。超出部分留在补偿累加器里逐窗偿还——旧行为是把债务挤进一窗爆发，
    // 驱动器吃不进的高频脉冲 = 转子丢同步 + 反抽（"450RPM 必失步"的机制）。
    // max_step_per_ramp 记的是"索要"步数（上面先记后夹），所以它仍然能看见债务。
    {
        const uint32_t cap = StepOutputChannel::windowCapacity(delta_ramp_ns);
        if (num_steps > cap)
        {
            num_steps = cap;
        }
    }

    // 规划（记账）步数：只受剩余行程限制，不受本窗实发限制。
    auto num_original_command_steps = static_cast<uint32_t>(std::floor(c.motion_step_accumulator_));
    if (num_original_command_steps > remaining_steps)
    {
        num_original_command_steps = remaining_steps;
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
        // 校准诊断用：只统计规划运动的脉冲，且用 num_steps（实发）而不是
        // num_original_command_steps（记账值）—— 两者不相等本身就是需要看见的信息。
        // 放在这个分支里而不是分支外，是因为归零那几步走角度模式：它们发的脉冲
        // 不属于任何“命令段长”，混进来会假报成“多发了脉冲”。
        c.step_pulses_emitted_ += num_steps;
        // 校准期间的发射偏差观测：只在校准 MOVING 阶段统计，其余时候一次比较就跳过。
        // 目的是分清：“实发≠记账”到底是补偿在跑（rps 峰值≠0），还是两个累加器的
        // 余数/钳位路径分叉（rps 峰值==0 但仍分叉）。
        if (c.calibration_stage_ == ClosedLoopController::CALIBRATION_MOVING)
        {
            if (num_steps != num_original_command_steps)
            {
                c.calib_bias_windows_++;
                c.calib_bias_sum_ += static_cast<int32_t>(num_steps)
                                   - static_cast<int32_t>(num_original_command_steps);
            }
            if (c.compensated_rps_ != 0.0f)
            {
                const float abs_rps = fast_abs(c.compensated_rps_);
                if (abs_rps > c.calib_comp_rps_peak_)
                {
                    c.calib_comp_rps_peak_ = abs_rps;
                }
            }
        }
        c.motion_steps_emitted_ += num_original_command_steps;
        c.motion_steps_emitted_signed += c.motion_direction_ * num_original_command_steps;
        // 本窗这批脉冲刚交给 DMA，要等下一拍确认真的发完，
        // 才并进 motion_steps_delivered_signed，用作跟随误差的命令位置。
        c.motion_dma_pending_signed = c.motion_direction_ * static_cast<int32_t>(num_original_command_steps);
    }
    c.motion_step_accumulator_ -= static_cast<float>(num_original_command_steps);
    c.compensated_motion_step_accumulator_ -= static_cast<float>(num_steps);
    // 累加器限幅：补偿长期单向时，被减的那一侧会漂出合理范围
    // （负太多意味着“欠发”的债越积越多，正太多意味着一次发不完）。
    // 这里不改变稳态行为，只防止极端参数把累加器推到浮点精度之外。
    if (c.compensated_motion_step_accumulator_ > 256.0f)
    {
        c.compensated_motion_step_accumulator_ = 256.0f;
    }
    else if (c.compensated_motion_step_accumulator_ < -256.0f)
    {
        c.compensated_motion_step_accumulator_ = -256.0f;
    }

    return false;
}

// ===========================================================================
// 速度模式连续硬件 PWM —— 设计动机见 motion_planner.h。
// TMR2 CH3 的 PWM 拓扑与窗口路径完全相同（PWM 模式 A + 低有效 + CCR=脉宽），
// 唯一区别：ARR 不再是 DMA 逐窗喂的序列，而是每拍更新一次的恒定周期
//（stepper_init_motion_timer 已开 PR 预装载，写入缓存到下个溢出才生效，不撕裂）。
// 脉冲间隔全局均匀：没有窗口边界死区，没有每窗 halt/重装的时序抖动。
// ===========================================================================
bool MotionPlanner::velocityPwmTick(uint64_t delta_ramp_ns)
{
    ClosedLoopController& c = owner_;
    if (!velocity_pwm_active_)
    {
        return false;
    }

    const uint32_t spr = c.getStepsPerRound();
    // 目标脉冲率 = 规划速度 + 闭环速度补偿（与 emitSteps 同语义，脉冲频率非负，
    // 方向只由 DIR/CH4 表达）。
    float pps = c.current_step_speed_ + c.compensated_rps_ * static_cast<float>(spr);
    if (pps < 0.0f)
    {
        pps = 0.0f;
    }

    // ---------- 频率量程（两档分频 + 迟滞） ----------
    // tier0：PSC=0，tick=4ns  → 3815 .. 500000 pps（16 位 PR / 驱动器 2µs 最小周期）
    // tier1：PSC=7，tick=32ns →  477 .. 496000 pps（64 细分下探到 2.2 RPM，消灭死区）
    // pps>8000 进 tier0、pps<3000 进 tier1，中间保持原档；换档要重装定时器，
    // 间隙约一个步周期，只在低速起步/蠕行穿越时发生一次。
    uint8_t tier = velocity_pwm_tier_;
    if (pps >= 8000.0f)
    {
        tier = 0U;
    }
    else if (pps < 3000.0f)
    {
        tier = 1U;
    }

    const uint32_t clock_hz = (tier == 0U) ? 250000000U : 31250000U;
    const uint32_t pr_min = (tier == 0U) ? kMinDriverStepPeriodTicks : 63U;  // 两档都 ≈2µs
    uint32_t pr_ticks = 0U;
    if (pps >= 1.0f)
    {
        const uint32_t want = static_cast<uint32_t>(static_cast<float>(clock_hz) / pps + 0.5f);
        if (want <= 65535U)
        {
            pr_ticks = (want < pr_min) ? pr_min : want;
        }
        // pr_ticks==0：连粗分频档的下限（477pps≈2.2RPM@64细分）都够不着，不发
    }

    if (pr_ticks == 0U)
    {
        if (velocity_pwm_running_)
        {
            stepper_common::stepper_stop_motion_timer();
            velocity_pwm_running_ = false;
        }
    }
    else
    {
        if (!velocity_pwm_running_ || tier != velocity_pwm_tier_)
        {
            stepper_common::stepper_init_motion_timer((tier == 0U) ? 0U : 7U,
                                                      stepper_common::k_step_pulse_ticks,
                                                      c.motion_direction_ > 0);
            velocity_pwm_running_ = true;
            velocity_pwm_tier_ = tier;
        }
        tmr_period_value_set(TMR2, pr_ticks);
        c.step_period_us_ = static_cast<uint16_t>(
            (pr_ticks * ((tier == 0U) ? 4U : 32U) + 500U) / 1000U);
    }

    // ---------- 记账：按"实际发出的速率"入账，FollowError 不造假 ----------
    // 上一版按目标 pps 入账，被上限钳位时账面跑得比脉冲快，0x121 在 2343RPM
    // 之上"不可控暴涨"是假账。现在发多少记多少：FE 回归真实同步误差本义。
    // （0x14C 指令速度维持目标语义不变。）
    constexpr float kDtCapS = 150.0e-6f;  // 任务停顿保护，与 emitSteps 同值同理由
    float dt = static_cast<float>(delta_ramp_ns) * 1.0e-9f;
    if (dt > kDtCapS)
    {
        dt = kDtCapS;
        c.step_catchup_events_++;
    }
    c.motion_speed_rpm_ = c.current_step_speed_ * 60.0f / static_cast<float>(spr);
    const float emitted_pps =
        (pr_ticks != 0U) ? (static_cast<float>(clock_hz) / static_cast<float>(pr_ticks)) : 0.0f;
    c.motion_step_accumulator_ += emitted_pps * dt;
    uint32_t n = 0U;
    if (c.motion_step_accumulator_ >= 1.0f)
    {
        n = static_cast<uint32_t>(c.motion_step_accumulator_);
        c.motion_step_accumulator_ -= static_cast<float>(n);
    }
    c.step_pulses_emitted_ += n;
    c.motion_steps_emitted_ += n;
    c.motion_steps_emitted_signed +=
        static_cast<int32_t>(c.motion_direction_) * static_cast<int32_t>(n);
    c.motion_dma_pending_signed = c.motion_direction_ * static_cast<int32_t>(n);
    return true;
}

void MotionPlanner::velocityPwmHalt()
{
    if (velocity_pwm_running_)
    {
        stepper_common::stepper_stop_motion_timer();
        velocity_pwm_running_ = false;
    }
    velocity_pwm_active_ = false;
}
