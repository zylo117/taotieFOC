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
    //
    // 【关键性能优化】原来每次调用都跑 dma_default_para_init + dma_init + dma_flexible_config，
    // 等于每窗（50us）重写几十个 DMA 寄存器 —— 2 万次/秒。
    // 而方向/数据宽度/优先级/触发源从来不变，seq_buffer_ 也是同一个固定数组，
    // 所以只需要第一次全量配置，之后每次只改“传输数量”和“内存地址”两个字段。
    // 实测收益：规划运动（每窗都发）CPU 从 100% 降下来，外部脉冲（间歇发）也明显改善。
    bool loadArrSequence(uint32_t* seq, uint32_t seq_count)
    {
        static bool dma_configured = false;
        if (!dma_configured)
        {
            dma_init_type dma_conf;
            dma_default_para_init(&dma_conf);
            dma_conf.direction            = DMA_DIR_MEMORY_TO_PERIPHERAL;
            dma_conf.peripheral_inc_enable = FALSE;
            dma_conf.memory_inc_enable     = TRUE;
            dma_conf.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
            dma_conf.memory_data_width     = DMA_MEMORY_DATA_WIDTH_WORD;
            dma_conf.loop_mode_enable      = FALSE;
            dma_conf.priority              = DMA_PRIORITY_HIGH;
            dma_conf.peripheral_base_addr = reinterpret_cast<uint32_t>(&TMR2->pr);
            dma_conf.memory_base_addr     = reinterpret_cast<uint32_t>(seq);
            dma_conf.buffer_size          = static_cast<uint16_t>(seq_count);
            dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW);
            dma_init(DMA1_CHANNEL2, &dma_conf);
            dma_configured = true;
        }
        else
        {
            // 只更新传输数量和内存地址（两个寄存器写），其余字段保持上次的值
            DMA1_CHANNEL2->dtcnt = static_cast<uint16_t>(seq_count);
            DMA1_CHANNEL2->maddr = reinterpret_cast<uint32_t>(seq);
        }

        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
        return true;
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

bool StepOutputChannel::emit(uint32_t num_steps, bool direction, bool direction_changed,
                             uint64_t window_ns, uint32_t step_pulse_ticks)
{
    if (num_steps == 0U || num_steps > MAX_PULSE_ARR_LEN)
    {
        return false;
    }

    const uint32_t seq_count = num_steps + 2U;  // 头尾各一个换向 guard

    // 【性能】整个预算计算用 32 位完成。
    // 原因：Cortex-M4 只有 32 位硬件除法（UDIV，2~12 周期），64 位除法会变成软件库
    // __aeabi_uldivmod（约 300 周期/次），乘法 __aeabi_lmul 也要几十周期。
    // 原实现每窗跑 3 次 64 位除法 + 2 次 64 位乘法 ≈ 3.9us，而整个控制周期才 50us。
    //
    // 为什么能降到 32 位：window_ns 正常是 50000（50us），即使调度卡顿到 10ms
    // （10,000,000）也远小于 2^32，remaining_ticks x PERMILLE 最大约 1.1e9 也不溢出。
    // 超过 10ms 的卡顿本来就要丢帧，钳位是安全且必要的。
    constexpr uint64_t kMaxWindowNs = 10000000ULL;  // 10ms 上限（200 个控制周期）
    const uint32_t window_ns_32 = (window_ns > kMaxWindowNs)
        ? static_cast<uint32_t>(kMaxWindowNs)
        : static_cast<uint32_t>(window_ns);

    // 头尾 guard 占掉的固定时间：两端各一个 guard_tick，再加 5ns 余量吸收取整。
    const uint32_t guard_ns = static_cast<uint32_t>(
        2U * static_cast<uint32_t>(stepper_common::guard_tick) *
        static_cast<uint32_t>(stepper_common::target_tick_time) + 5U);
    if (window_ns_32 <= guard_ns)
    {
        // 窗口连 guard 都放不下（首拍或调度严重卡顿），这一窗不发。
        // 调用方会把这段时间留给下一拍，比硬塞一串超窗脉冲安全。
        return false;
    }

    const uint32_t tick_ns = static_cast<uint32_t>(stepper_common::target_tick_time);
    const uint32_t remaining_ticks = (window_ns_32 - guard_ns) / tick_ns;
    const uint32_t min_step_ticks = step_pulse_ticks * 2U;  // 高电平 + 低电平各一个脉宽

    uint32_t step_ticks = min_step_ticks;
    bool spread_enabled = false;
    // 预算窗口（两种模式共用，不再重复算一遍 64 位乘除）
    uint32_t budget_ticks = num_steps * min_step_ticks;

#if STEP_SEQ_MODE != 0
    // 高速兜底：步频超过 1/(2*脉宽) = 2.5 Mstep/s 时，连 min_step_ticks 都塞不下，
    // 平滑没有意义且必定超窗（下一窗会砍掉没发完的脉冲 → 真丢步），退回恒速突发。
    // 另：只铺满窗口预算的 STEP_SEQ_SPREAD_PERMILLE，剩下的余量留给控制任务
    // 调度抖动和启动序列的软件开销，不让每个窗口都擦着边界结束。
    const uint32_t spread_budget_ticks =
        (remaining_ticks * static_cast<uint32_t>(STEP_SEQ_SPREAD_PERMILLE)) / 1000U;
    const uint32_t spread_need_ticks = num_steps * min_step_ticks;
    if (spread_budget_ticks >= spread_need_ticks)
    {
        spread_enabled = true;
        step_ticks = spread_budget_ticks / num_steps;
        budget_ticks = spread_budget_ticks;
    }
#endif

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
