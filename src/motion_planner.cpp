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
// 两个“看起来奇怪但必须保留”的设计：
//   1) base_speed 用上一拍输出：没有它，小 P 根本推不动转子（实测极慢）；
//      代价是它是正反馈，所以速度环 Ki 必须为 0，否则持续发散。
//   2) P 项 ×0.01：角度模式 PID 输出就是全部速度，而运动模式里 PID 只做小修正，
//      同样的 P 在角度模式等效放大很多，不缩就大幅摆动。
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
        const float position_error_rev = desired_error_deg / 360.0f;
        // 【唯一样例：角度模式的 P 项 ×1/100】
        // 网页与固件用的是同一套 PID（没有任何口径倍率），但角度模式下
        // P 的等效增益比运动模式大得多（运动模式 PID 只做小修正），所以这里把 P 缩到 1/100。
        // 只缩 P：缩 I 会让稳态误差永远消不掉，缩 D 会丢阻尼而过冲。
        constexpr float kAnglePScale = 0.01f;
        // 根据误差角算出位置环的速度修正量
        const float position_reference_rps = c.position_pid_.update(position_error_rev, dt, kAnglePScale);
        // 【恢复 base_speed：角度模式需要“记忆”】
        // 一度改成 0（纯位置伺服）后实测：
        //   - 极慢：误差 4 度 -> P=9 -> 只输出 0.1rps，几乎不动
        //   - 抖动：PID 输出直接就是全部速度，速度环的小噪声被完整放大
        // 原来的写法把上一拍输出作为基础速度，相当于给位置环一个积分器，
        // 小 P 也能逐渐把速度“滚”起来，这是这个结构能用的关键。
        //
        // 代价（已知且可接受）：它是个正反馈，所以速度环不能靠 I 项去消除稳态误差，
        // 否则会持续发散（实测 I=1 就发散）。速度环只当阻尼用。
        const float base_speed_rps = c.current_step_speed_ / static_cast<float>(c.getStepsPerRound());
        // 总期望速度 = 基础速度 + 位置环修正量
        const float total_ref_rps = base_speed_rps + position_reference_rps;
        // 总期望速度和实际速度的速度差
        const float velocity_error_rps = total_ref_rps - (c.encoder_speed_rpm_ / 60.0f);
        // 根据速度差算出速度环修正量（同样只缩 P 项）
        const float velocity_correction_rps = c.velocity_pid_.update(velocity_error_rps, dt, kAnglePScale);

        const float max_rps = c.angle_max_rpm_ / 60.0f;
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

    // ---------- 速度模式：运行时改目标转速 ----------
    // 背景：motion_max_step_s_ 只在 startMotion() 里按 motion_max_rpm_ 算一次，
    // 上位机后来改 0x11B 不会生效 —— 这就是“负载角校准只能一段一段测、
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
            // 只执行一次等于完全没减速 —— 表现为“负载角阶梯只能升不能降”。
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
        if (c.current_step_speed_ < 0.0f)
        {
            c.current_step_speed_ = 0.0f;
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
// 这个钳位同时是“防喷发”（大误差时一次吐几十步堵住 DMA）和“实际限速”：
// 3 步/拍 x 20kHz = 60000 步/s，按 12800 步/圈约 281 RPM，
// 界面上的“闭环最大速度”超过这个值也达不到。
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
    const float dt_step_s = static_cast<float>(delta_ramp_ns) * 1.0e-9f;
    c.motion_step_accumulator_ += fast_abs(c.current_step_speed_) * dt_step_s;

    const uint32_t micro_steps_per_round = c.getStepsPerRound();
    const auto compensated_step_speed = c.compensated_rps_ * static_cast<float>(micro_steps_per_round);
    c.compensated_motion_step_accumulator_ +=
        fast_abs(c.current_step_speed_ + compensated_step_speed) * dt_step_s;

    // 角度模式：每拍最多走 kMaxStepsPerTick 步。
    // 原来钳到 1 步/拍等于硬限速 20000 步/s：按 12800 步/圈就是 93 RPM，
    // 所以界面上设“闭环最大速度 1200RPM”根本达不到。
    // 现在提到 3 步/拍（60000 步/s ≈ 280 RPM），仍在 50us 窗口内。
    constexpr float kMaxStepsPerTick = 3.0f;
    if (c.closed_loop_angle_mode_enabled_ && c.motion_step_accumulator_ > kMaxStepsPerTick)
    {
        c.motion_step_accumulator_ = kMaxStepsPerTick;
        c.compensated_motion_step_accumulator_ = kMaxStepsPerTick;
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

    auto num_steps = static_cast<uint32_t>(std::floor(c.compensated_motion_step_accumulator_));
    if (num_steps > c.max_step_per_ramp)
    {
        c.max_step_per_ramp = num_steps;
    }

    auto num_original_command_steps = static_cast<uint32_t>(std::floor(c.motion_step_accumulator_));
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

    // 【记账不能超过实发】上面的 remaining / MAX_PULSE_ARR_LEN 钳位只作用于 num_steps。
    // 如果不把 num_original_command_steps 也压到不超过实发，本窗根本没发那么多脉冲，
    // 记账却照样加上去 —— 段结束判据（看记账值）会提前满足，最后几微步压根没发出去，
    // 于是每段行程被悄悄少掉一点（实测 (counted-commanded)=+21 / (emitted-counted)=-21，
    // 净 0 只是两头偏差互相抵消，不是“没问题”）；同时命令位置会比真正发出去的
    // 脉冲领先，跟随误差因此带一个假分量。
    // 注意这里只“下调”记账值：当补偿让实发多于原始命令时，仍按原始命令记账，
    // 这样命令位置不会被 PID 自己的输出推着走（那是另一个问题的根源，保持原语义）。
    if (num_original_command_steps > num_steps)
    {
        num_original_command_steps = num_steps;
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
    c.motion_step_accumulator_ -= num_original_command_steps;
    c.compensated_motion_step_accumulator_ -= num_steps;

    return false;
}
