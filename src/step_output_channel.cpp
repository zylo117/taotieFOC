#include "step_output_channel.h"

#include "step_sequencer.h"
#include "stepper_driver.h"

namespace
{
    // 重新装载 ARR 序列之前必须先把定时器和 DMA 全停下来：
    // 否则 DMA 会在计数器还在跑的时候改写 ARR，抢到的第一个周期长度随机，
    // 表现为“偶发多一步/少一步”，很难复现。
    void haltTimerAndDma()
    {
        tmr_counter_enable(TMR2, FALSE);
        tmr_output_enable(TMR2, FALSE);
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
    }

    // 用 DMA1_CH2 把 ARR 序列逐个搬进 TMR2->pr（周期寄存器）。
    // 每次溢出搬一个 32 位字，搬完即止（loop_mode = FALSE）。
    void loadArrSequence(uint32_t* seq, uint32_t seq_count)
    {
        dma_init_type dma_conf;
        dma_default_para_init(&dma_conf);
        dma_conf.direction            = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_conf.buffer_size          = static_cast<uint16_t>(seq_count);
        dma_conf.peripheral_inc_enable = FALSE;
        dma_conf.memory_inc_enable     = TRUE;
        dma_conf.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_conf.memory_data_width     = DMA_MEMORY_DATA_WIDTH_WORD;
        dma_conf.loop_mode_enable      = FALSE;
        dma_conf.priority              = DMA_PRIORITY_HIGH;

        dma_conf.peripheral_base_addr = reinterpret_cast<uint32_t>(&TMR2->pr);
        dma_conf.memory_base_addr     = reinterpret_cast<uint32_t>(seq);
        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
        dma_init(DMA1_CHANNEL2, &dma_conf);

        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
    }
}

void StepOutputChannel::startSequence(uint32_t* seq, uint32_t seq_count, bool direction)
{
    haltTimerAndDma();
    loadArrSequence(seq, seq_count);
    stepper_common::stepper_init_motion_timer(stepper_common::k_psc,
                                             stepper_common::k_step_pulse_ticks,
                                             direction);
}

uint32_t StepOutputChannel::windowCapacity(uint64_t window_ns)
{
    // 与 emit 的 guard 预算公式保持一致（少算安全，多算会超窗被下窗截断）。
    const uint64_t guard_ns =
        2ULL * stepper_common::guard_tick * stepper_common::target_tick_time + 5ULL;
    if (window_ns <= guard_ns)
    {
        return 0U;
    }
    const uint64_t budget_ticks =
        ((window_ns - guard_ns) / stepper_common::target_tick_time) *
        static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE) / 1000ULL;
    const uint64_t cap = budget_ticks / kMinDriverStepPeriodTicks;
    return (cap > MAX_PULSE_ARR_LEN) ? MAX_PULSE_ARR_LEN : static_cast<uint32_t>(cap);
}

bool StepOutputChannel::emit(uint32_t num_steps, bool direction, bool direction_changed,
                             uint64_t window_ns, uint32_t step_pulse_ticks)
{
    if (num_steps == 0U || num_steps > MAX_PULSE_ARR_LEN)
    {
        return false;
    }

    const uint32_t seq_count = num_steps + 2U;  // 头尾各一个换向 guard

    // 头尾 guard 占掉的固定时间：两端各一个 guard_tick，再加 5ns 余量吸收取整。
    const uint64_t guard_ns =
        2ULL * stepper_common::guard_tick * stepper_common::target_tick_time + 5ULL;
    if (window_ns <= guard_ns)
    {
        // 窗口连 guard 都放不下（首拍或调度严重卡顿），这一窗不发。
        // 调用方会把这段时间留给下一拍，比硬塞一串超窗脉冲安全。
        return false;
    }

    const uint64_t remaining_ticks = (window_ns - guard_ns) / stepper_common::target_tick_time;
    // 步距下限抬到驱动器合法最小周期（2µs）：min_step_ticks 同时是"平滑失败时
    // 恒速突发的周期"，原来的 2x脉宽=400ns 只保证硬件不吞高电平，却远超 TMC2209
    // 输入级能力（≥1µs 高/低）——债务窗一旦走到突发分支就是驱动器根本没吃进的
    // 废脉冲。调用方已按 windowCapacity 夹过步数，这里再保底一次。
    const uint32_t min_step_ticks =
        (step_pulse_ticks * 2U > kMinDriverStepPeriodTicks)
            ? (step_pulse_ticks * 2U)
            : kMinDriverStepPeriodTicks;

    uint32_t step_ticks = min_step_ticks;
    bool spread_enabled = false;

#if STEP_SEQ_MODE != 0
    // 高速兜底：步频超过 1/(2*脉宽) = 2.5 Mstep/s 时，连 min_step_ticks 都塞不下，
    // 平滑没有意义且必定超窗（下一窗会砍掉没发完的脉冲 → 真丢步），退回恒速突发。
    // 另：只铺满窗口预算的 STEP_SEQ_SPREAD_PERMILLE，剩下的余量留给控制任务
    // 调度抖动和启动序列的软件开销，不让每个窗口都擦着边界结束。
    const uint64_t spread_budget_ticks =
        (remaining_ticks * static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000ULL;
    const uint64_t spread_need_ticks = static_cast<uint64_t>(num_steps) * min_step_ticks;
    if (spread_budget_ticks >= spread_need_ticks)
    {
        spread_enabled = true;
        step_ticks = static_cast<uint32_t>(spread_budget_ticks / num_steps);
    }
#endif

    // 【TMR2 是 16 位定时器：PR 写进 >0xFFFF 的值会按 mod 65536 截断】
    // 控制任务停摆 20ms 后窗口预算暴涨，小步数窗口的 step_ticks 轻松超过 65535
    // （250 步/s 的爬行速度单步周期就 ~62500 tick，已经贴边）——截断后脉冲
    // 周期变成完全无关的随机值，表现是"卡一下之后猛地抽一下"。
    // 封顶到 0xFFFF：宁可低速段被压到最快可表示周期（仍是单调可预期的），
    // 也不要 mod 截断出来的乱序周期。
    if (step_ticks > 0xFFFFU)
    {
        step_ticks = 0xFFFFU;
    }

    // 预算窗口转 uint32（任务阻塞极端情况下 remaining_ticks 可能很大，封顶防溢出）。
    const uint64_t budget_ticks_wide = spread_enabled
        ? (remaining_ticks * static_cast<uint64_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000ULL
        : static_cast<uint64_t>(num_steps) * min_step_ticks;
    const uint32_t budget_ticks = (budget_ticks_wide > 0x3FFFFFFFULL)
        ? 0x3FFFFFFFU
        : static_cast<uint32_t>(budget_ticks_wide);

    step_sequencer::fill(seq_buffer_, seq_count, step_ticks, min_step_ticks, budget_ticks);
    step_sequencer::addDirectionGuard(seq_buffer_, seq_count, stepper_common::guard_tick,
                                      direction_changed);
    startSequence(seq_buffer_, seq_count, direction);
    return true;
}

void StepOutputChannel::stop()
{
    stepper_common::stepper_stop_motion_timer();
}
