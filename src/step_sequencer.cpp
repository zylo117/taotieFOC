#include "step_sequencer.h"

#include <cmath>

#include "stepper_driver.h"

// ===========================================================================
// STEP 时序模式的详细设计与调参方法写在 step_sequencer.h 的宏定义旁边，
// 这里只保留一句：模式 1 和 2 都保证“整窗总时长严格等于窗口预算”，
// 所以窗口边界不会产生位置漂移。
// ===========================================================================
// 编译期自证：pio run 的日志里会打印这一段，可以直接确认烧进去的是哪一档。
#if STEP_SEQ_MODE == 0
#pragma message("[STEP_SEQ] mode 0: constant-speed burst (original)")
#elif STEP_SEQ_MODE == 1
#pragma message("[STEP_SEQ] mode 1: smooth spread")
#else
#pragma message("[STEP_SEQ] mode 2: smooth spread + harmonic injection")
#endif

namespace
{
    constexpr float k_two_pi = 6.28318530718f;

#if STEP_SEQ_MODE == 2
    // 谐波注入的相位（rad），跨 ramp 窗口连续累加：
    // 如果每个窗口都从 0 相位重启，就等于 20kHz 的锯齿调频，会产生一大堆无用边带。
    float g_step_seq_phase_rad = 0.0f;
#endif
}

namespace step_sequencer
{
    void fill(uint32_t* seq, uint32_t seq_count, uint32_t base_ticks,
              uint32_t min_ticks, uint32_t budget_ticks)
    {
#if STEP_SEQ_MODE == 0
        // ---- 档位 0：恒速突发，整窗用最小步距一口气打完，剩余时间全留给 guard ----
        const uint32_t base = (base_ticks < min_ticks) ? min_ticks : base_ticks;
        for (uint32_t i = 1; i < seq_count - 1; i++)  // 头尾是换向 guard，不是脉冲
        {
            seq[i] = base;
        }
#else
        // ---- 档位 1/2：把本窗 N 步铺满整个预算窗口 ----
        // 逐步用“剩余预算 / 剩余步数”求均值，而不是一次性算 budget/N：
        //   1) 整数除法余数被后面的步自动吃掉，整窗总时长误差 < 1 tick(4ns)，永不累积；
        //   2) 谐波调制的摆动会被后面的步“吸回均值”，总时长严格等于预算，
        //      所以窗口边界不会产生位置漂移（上一版 /(1+depth) 缩放就是死在这里）。
        uint32_t budget = budget_ticks;
        uint32_t steps_left = seq_count - 2U;  // 头尾是换向 guard

#if STEP_SEQ_MODE == 2
        float phase = g_step_seq_phase_rad;
#endif

        for (uint32_t i = 1; i < seq_count - 1; i++)
        {
            uint32_t ticks;
            if (steps_left <= 1U)
            {
                // 最后一步把预算里剩下的（含除法余数）全部吃掉，保证与窗口严格对齐
                ticks = budget;
            }
            else
            {
                const uint32_t avg = budget / steps_left;
                ticks = avg;

#if STEP_SEQ_MODE == 2
                ticks = static_cast<uint32_t>(
                    static_cast<float>(avg) * (1.0f + STEP_SEQ_HARMONIC_DEPTH * sinf(phase)) + 0.5f);
                // 相位按“真实流逝时间”推进（用调制后的 ticks），物理上才是正确的调频；
                // 注入频率只由 f_h 决定，与步频无关，不存在自激问题。
                phase += k_two_pi * STEP_SEQ_HARMONIC_FREQ_HZ *
                         (static_cast<float>(ticks) *
                          static_cast<float>(stepper_common::target_tick_time) * 1.0e-9f);
                if (phase >= k_two_pi)
                {
                    phase -= k_two_pi;
                }
#endif

                // 给后面每一步至少留 min_ticks，否则总线必然超窗（下一个窗口会砍掉没发完的脉冲）
                const uint32_t reserve = (steps_left - 1U) * min_ticks;
                const uint32_t max_this = (budget > reserve) ? (budget - reserve) : min_ticks;
                if (ticks > max_this)
                {
                    ticks = max_this;
                }
                if (ticks < min_ticks)
                {
                    ticks = min_ticks;
                }
            }

            seq[i] = ticks;
            budget -= ticks;
            steps_left--;
        }

#if STEP_SEQ_MODE == 2
        g_step_seq_phase_rad = phase;
#endif
#endif
    }

    void addDirectionGuard(uint32_t* seq, uint32_t seq_count, uint32_t guard_tick,
                           bool switch_direction)
    {
        // 如果开局和上一次方向相同则不必加额外换向等待，否则等一个guard_tick
        // 但是末端一定要加，避免这一局最后一步脉冲结束不到guard_tick就进入下一局开局换向
        // 注意换向后要外面自己维护方向这个状态
        if (!switch_direction)
        {
            seq[0] = 1;  // 这1tick至关紧要，初始化的ARR如果是0就永久卡住了
        }
        else
        {
            seq[0] = guard_tick;
        }
        seq[seq_count - 1U] = guard_tick;
    }

    void resetHarmonicPhase()
    {
#if STEP_SEQ_MODE == 2
        g_step_seq_phase_rad = 0.0f;
#endif
    }
}
