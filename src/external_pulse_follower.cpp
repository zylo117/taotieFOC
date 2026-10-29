#include "external_pulse_follower.h"

#include <cmath>

#include "closed_loop_controller.h"
#include "step_output_channel.h"
#include "stepper_driver.h"

namespace
{
    float absF(float v)
    {
        return v < 0.0f ? -v : v;
    }
}

ExternalPulseFollower::ExternalPulseFollower(ClosedLoopController& owner) : owner_(owner)
{
}

void ExternalPulseFollower::accumulateCommand(int32_t net_steps, int32_t abs_steps, bool prime)
{
    if (!enabled_)
    {
        return;
    }

    if (prime)
    {
        // 使能后的第一拍：把使能之前积压的历史脉冲扔掉，
        // 否则一上位就先把刚才调试用的几千个脉冲补发出去，直接飞车。
        prime_pending_ = false;
        return;
    }

    cmd_steps_ += net_steps;

    // 窗内换向自检：一个 50us 窗口最终只能落成一个 DMA 序列、一个 DIR，
    // 所以「先正走后反走」的一串脉冲只能按净和发出去。终点位置仍然正确，
    // 但中间那段往返没有执行 —— 有背隙/弹性传动的机构会按换向次数累积偏差。
    const int32_t net_abs = net_steps >= 0 ? net_steps : -net_steps;
    if (abs_steps != net_abs)
    {
        ++window_reversals_;
    }
}

void ExternalPulseFollower::update(uint64_t delta_ramp_ns)
{
    ClosedLoopController& c = owner_;

    if (c.driver_ == nullptr || delta_ramp_ns == 0U || delta_ramp_ns > 1000000ULL)
    {
        // 首拍（motion_last_ramp_time_ns_ 还是 0）或调度卡顿超过 1ms 时不补发：
        // 积压会留在 cmd_steps_ 里由后面几拍慢慢追，比一次吐上百步安全得多。
        return;
    }

    // 故障保护：编码器/磁场报警时驱动已被 reportXxxFault() 失能，
    // 这里再发脉冲只会空转 TMR2（且积压白白变成「丢步」），直接不输出。
    if (c.output_stopped_)
    {
        c.motion_speed_rpm_ = 0.0f;
        return;
    }

    const uint32_t micro_steps_per_round = c.getStepsPerRound();
    const float dt_s = static_cast<float>(delta_ramp_ns) / 1.0e9f;

    // 脉宽可能在跟随时被上位机改（0x123），每窗按最新值刷一遍。
    // DDA 路径是在 startMotion() 里算的，跟随路径没人管，漏了就是一个陈旧的 min_step_ticks。
    c.k_step_pulse_ticks = std::ceil(static_cast<float>(c.step_pulse_width_ns_) /
                                     static_cast<float>(stepper_common::target_tick_time));

    // ---- 1. 本窗要 1:1 转发的步数：积压、限速、序列长度三者取小 ----
    const int32_t backlog = cmd_steps_ - emitted_steps_;
    const int32_t backlog_abs = backlog >= 0 ? backlog : -backlog;

    // 限速：0 表示用 angle_max_rpm_ 换算出的驱动能力上限（默认 1200RPM x 12800 = 256k step/s）。
    // 宁可比外部慢一点，也不要一次性灌满让电机失步 —— 失步是真正找不回来的。
    uint32_t rate_step_s = max_step_s_;
    if (rate_step_s == 0U)
    {
        const float rpm_cap = c.angle_max_rpm_ > 0.0f ? c.angle_max_rpm_ : 1200.0f;
        const float step_s_cap = rpm_cap * static_cast<float>(micro_steps_per_round) / 60.0f;
        rate_step_s = static_cast<uint32_t>(step_s_cap > 1.0f ? step_s_cap : 1.0f);
    }

    float budget_steps = static_cast<float>(rate_step_s) * dt_s;
    if (budget_steps < 1.0f)
    {
        // 低速时也至少允许 1 步，否则小数部分会被限速反复截断，永远凑不满一整步
        budget_steps = 1.0f;
    }
    if (budget_steps > static_cast<float>(backlog_abs))
    {
        budget_steps = static_cast<float>(backlog_abs);
    }

    // 限速值不是整步/窗时（如 256k step/s 就是 12.8 步/窗），
    // 直接截断会每窗少 0.8 步 = 长期恒定落后 6%，必须靠余量补回来。
    float base_acc = budget_steps + base_residue_;
    uint32_t num_base = static_cast<uint32_t>(base_acc);
    if (num_base > static_cast<uint32_t>(backlog_abs))
    {
        num_base = static_cast<uint32_t>(backlog_abs);
    }
    base_residue_ = base_acc - static_cast<float>(num_base);
    if (base_residue_ > 1.0f)
    {
        base_residue_ = 1.0f;
    }
    else if (base_residue_ < 0.0f)
    {
        base_residue_ = 0.0f;
    }

    const int8_t base_sign = backlog >= 0 ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    const float signed_base = static_cast<float>(num_base) * static_cast<float>(base_sign);

    // ---- 2. 闭环补偿（0x107=1 才生效）：位置环 -> 速度环 -> 本窗修正步数 ----
    // 开环时这一段整体跳过，signed_base 就是全部输出，实现严格 1:1 转发。
    float pid_steps = 0.0f;
    if (c.closed_loop_compensation_enabled_ && c.encoder_ != nullptr)
    {
        // 位置环输入必须是「命令 - 实际」：motion_follow_error_deg_ 定义是「实际 - 命令」，要取反
        const float position_ref_rps = c.position_pid_.update(-c.motion_follow_error_deg_ / 360.0f, dt_s);
        const float base_rps = signed_base / static_cast<float>(micro_steps_per_round) / dt_s;
        const float measured_rps = c.encoder_speed_rpm_ / 60.0f;
        const float velocity_correction_rps =
            c.velocity_pid_.update(base_rps + position_ref_rps - measured_rps, dt_s);
        pid_steps = velocity_correction_rps * static_cast<float>(micro_steps_per_round) * dt_s;
        // PID 没调好时别让它一个窗口把整段序列吃光（真正超速由下面的限速和丢步计数兜）
        const float pid_cap = static_cast<float>(MAX_PULSE_ARR_LEN) * 0.5f;
        if (pid_steps > pid_cap)
        {
            pid_steps = pid_cap;
        }
        else if (pid_steps < -pid_cap)
        {
            pid_steps = -pid_cap;
        }
        // 让 0x121 等遥测继续看得到补偿量（开环时是 0）
        c.compensated_rps_ = velocity_correction_rps;
    }
    else
    {
        c.compensated_rps_ = 0.0f;
    }

    // ---- 3. 合成本窗的脉冲数 ----
    float signed_steps = signed_base + pid_steps;
    if (signed_steps > static_cast<float>(MAX_PULSE_ARR_LEN))
    {
        signed_steps = static_cast<float>(MAX_PULSE_ARR_LEN);
    }
    else if (signed_steps < -static_cast<float>(MAX_PULSE_ARR_LEN))
    {
        signed_steps = -static_cast<float>(MAX_PULSE_ARR_LEN);
    }
    const uint32_t num_steps = static_cast<uint32_t>(absF(signed_steps));

    if (num_steps == 0U)
    {
        // 没积压、补偿也是 0：保持使能，安静等下一拍（这不等于停止跟随）
        backlog_steps_ = static_cast<uint32_t>(backlog_abs);
        c.motion_speed_rpm_ = 0.0f;
        return;
    }

    const int8_t direction = signed_steps >= 0.0f ? 1 : -1;
    // 外部 DIR 什么时候翻我们不知道，只能逐窗比：换向那一窗必须在序列头尾插 guard，
    // 否则贴着发的脉冲会被驱动器认错方向（甚至整步丢掉）。
    if (c.motion_direction_ != direction)
    {
        c.motion_direction_change_pending_ = true;
    }
    c.motion_direction_ = direction;
    c.motion_speed_rpm_ = static_cast<float>(num_steps) / dt_s * 60.0f /
                          static_cast<float>(micro_steps_per_round);

    // ---- 4. 交给统一脉冲出口：铺 ARR 序列 + 启动 TMR2/DMA ----
    // 和规划模式用的是同一份 emit（同一个缓冲区、同一套 spread/guard 逻辑）。
    if (c.step_output_.emit(num_steps, direction > 0, c.motion_direction_change_pending_,
                            delta_ramp_ns, c.k_step_pulse_ticks))
    {
        c.motion_direction_change_pending_ = false;
    }

    // ---- 5. 记账：命令位置走 delivered，不凭空领先一个窗口 ----
    // 注意只把 num_base 计入「已转发」，PID 那部分不算 —— 否则位置环多发的步数
    // 会被积压逻辑当成「发多了」反过来退回，两个环互相推着振荡。
    emitted_steps_ += static_cast<int32_t>(num_base) * static_cast<int32_t>(base_sign);
    const int32_t emitted = direction * static_cast<int32_t>(num_steps);
    c.motion_steps_emitted_signed += emitted;
    c.motion_dma_pending_signed += emitted;

    const int32_t new_backlog = cmd_steps_ - emitted_steps_;
    const uint32_t new_backlog_abs =
        static_cast<uint32_t>(new_backlog >= 0 ? new_backlog : -new_backlog);
    backlog_steps_ = new_backlog_abs;
    if (new_backlog_abs > peak_backlog_steps_)
    {
        peak_backlog_steps_ = new_backlog_abs;
    }
    // 告警口径：积压超过一圈 = 电机已经跟不上输入频率了。只把超出部分累加，
    // 所以「正常几拍的延迟、下一窗就补回来」不会触发；这个数持续增长才是真的超速。
    if (new_backlog_abs > micro_steps_per_round)
    {
        dropped_steps_ += new_backlog_abs - micro_steps_per_round;
    }
}

void ExternalPulseFollower::resetCounters()
{
    cmd_steps_ = 0;
    emitted_steps_ = 0;
    base_residue_ = 0.0f;
    backlog_steps_ = 0U;
    peak_backlog_steps_ = 0U;
    dropped_steps_ = 0U;
    window_reversals_ = 0U;
}
