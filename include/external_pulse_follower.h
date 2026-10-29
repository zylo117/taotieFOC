#ifndef EXTERNAL_PULSE_FOLLOWER_H
#define EXTERNAL_PULSE_FOLLOWER_H

#include <cstdint>

class ClosedLoopController;

// 外部 PUL/DIR 脉冲跟随（把 PB5/PB4 上收到的脉冲变成电机命令）。
//
// 语义是「位置指令跟随」而不是严格透明转发：
//   1 个外部脉冲 = 1 个驱动器微步，累加成命令位置；
//   本窗把「命令 - 已发」的积压按限速重发。开环时严格 1:1，闭环时再叠 PID 修正。
//
// 和 ParameterRouter 一样是控制器的 friend：这里的积压记账必须和控制器里的
// motion_steps_emitted_signed / motion_dma_pending_signed / 方向 / 速度共用同一份，
// 拆成 getter/setter 只会引入两套状态不同步的风险。
class ExternalPulseFollower
{
public:
    explicit ExternalPulseFollower(ClosedLoopController& owner);

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool on) { enabled_ = on; }
    bool primePending() const { return prime_pending_; }
    void setPrimePending(bool pending) { prime_pending_ = pending; }
    void setMaxStepRate(uint32_t steps_per_second) { max_step_s_ = steps_per_second; }
    uint32_t maxStepRate() const { return max_step_s_; }

    int32_t cmdSteps() const { return cmd_steps_; }
    int32_t emittedSteps() const { return emitted_steps_; }
    uint32_t backlogSteps() const { return backlog_steps_; }
    uint32_t peakBacklogSteps() const { return peak_backlog_steps_; }
    uint32_t droppedSteps() const { return dropped_steps_; }
    uint32_t windowReversals() const { return window_reversals_; }

    // 本窗的脉冲绝对值和 != |净和|，说明窗内 DIR 翻过（轨迹会被净和压平）
    void noteWindowReversal() { ++window_reversals_; }

    // 把本窗采到的脉冲并入命令位置。
    //   net_steps  ：本窗脉冲的有符号净和（DIR 低 = +1，高 = -1）
    //   abs_steps  ：本窗脉冲的绝对值和，用来判断窗内有没有换向
    //   prime      ：使能后的第一拍为 true，直接丢弃。使能前积压的历史脉冲
    //                一上位就补发出去会直接飞车。
    void accumulateCommand(int32_t net_steps, int32_t abs_steps, bool prime);

    // 本窗把积压按限速重发（开环 1:1，闭环叠 PID 修正）
    void update(uint64_t delta_ns);

    void resetCounters();

    // 限速硬上限：128 步/窗 x 20kHz = 2.56 Mstep/s，再高会超出 50us 窗口
    static constexpr uint32_t kMaxStepRateLimit = 2560000UL;

private:
    ClosedLoopController& owner_;

    volatile bool enabled_ = false;
    volatile bool prime_pending_ = false;
    volatile int32_t cmd_steps_ = 0;
    // 已 1:1 转发出去（有符号，不含 PID 修正）。
    // PID 修正量不能算进来，否则位置环多发的步数会被积压逻辑当成「发多了」反向退回，
    // 位置环和积压环互相推着振荡。
    volatile int32_t emitted_steps_ = 0;
    // 限速值不是整步/窗时（256k = 12.8 步/窗），截断会长期落后 6%，靠这个余量补
    volatile float base_residue_ = 0.0f;
    volatile uint32_t max_step_s_ = 0U;
    volatile uint32_t backlog_steps_ = 0U;
    volatile uint32_t peak_backlog_steps_ = 0U;
    volatile uint32_t dropped_steps_ = 0U;
    volatile uint32_t window_reversals_ = 0U;
};

#endif
