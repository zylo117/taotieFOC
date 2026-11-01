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
    // 校准专用状态（本类持有）。注意：和原实现一样不重置 home_wait_ticks_，
    // HOMING 的超时计数会带上一次残留（最多 1000 拍 = 50ms），行为保持一致。
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

    // 【不恢复补偿开关】校准是开环操作；恢复成校准前的 true 会造成
    // “界面已取消勾选、固件里补偿却开着”的不一致。由上位机下次启动显式下发 0x107。
    c.closed_loop_compensation_enabled_ = false;

    c.calibration_stage_ = ClosedLoopController::CALIBRATION_IDLE;
    printf("[CALIBRATION] finished saved=%u\r\n", saved ? 1U : 0U);
}

// 把转子移动到第 index 个校准点（对应原 startEncoderCalibrationMove）。
// 每段只走 steps_per_round/CALIBRATION_TABLE_SIZE 步（64 细分下 12~13 步）。
bool EncoderCalibrator::startMove(uint16_t index)
{
    ClosedLoopController& c = owner_;

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
    // 【加速度必须足够大】每段只走 12~13 步，而 startMotion 按梯形算减速点
    // decel_total_steps_ = vmax^2/(2a)：
    //   60 RPM/s    -> 6400 步 >> 13 步：第一拍就进 DECEL，一步不发
    //   60000 RPM/s -> 6 步：吃掉半程（实测整圈只走 200 度）
    // 拉到 600000 RPM/s -> decel = 0.64 取整 0 -> steps_to_decel_ = 0 -> 全程 CRUISE。
    c.motion_accel_rpm_s_ = 600000.0f;
    c.motion_pulse_count_ = steps;
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
    // 复用 home_wait_ticks_ 当等待计数器（进 SAMPLING / 每段移动前已清零）：
    // 等 1000 拍 = 50ms。不能等太久：1024 个点 × 50ms 已经是 51 秒，
    // 200ms 会变成 3.4 分钟，完全没法用。
    if (home_wait_ticks_ < 1000U)
    {
        home_wait_ticks_++;
        // 同时把可能存在的运动状态收尾，避免停顿期间残留速度
        c.current_step_speed_ = 0.0f;
        return;
    }

    sample_attempts_++;
    if (raw != 0xFFFFU)
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
        printf("[CALIBRATION] invalid encoder sample: %d at index=%u, attempt=%u\r\n",
            raw,
            static_cast<unsigned>(index_),
            static_cast<unsigned>(sample_attempts_));
    }

    if (sample_count_ < 202U)
    {
        if (sample_attempts_ >= 1000U)
        {
            printf("[CALIBRATION] insufficient encoder samples at index=%u\r\n",
                   static_cast<unsigned>(index_));
            finish(false);
        }
        return;
    }

    const int32_t trimmed_mean = (sample_sum_ - sample_min_ - sample_max_) / 200;
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
