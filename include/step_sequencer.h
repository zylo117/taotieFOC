#ifndef STEP_SEQUENCER_H
#define STEP_SEQUENCER_H

#include <cstdint>

// ===== STEP 时序模式（三档，platformio.ini 里用 -DSTEP_SEQ_MODE 选）=====
// 0 = 恒速突发：窗口开头用 min_ticks 一口气打完，简单、永不丢步，但位置是硬台阶
// 1 = 平滑铺满：把 N 步均匀铺到整窗，台阶被摊平，位置/速度连续（默认）
// 2 = 平滑 + 谐波注入：在 1 的基础上对步频做正弦调频，把窄带能量摊到边带
// 定义放在头文件里，是因为 StepOutputChannel 决定“本窗要不要平滑铺开”时也要看它们；
// 全部用 #ifndef 包住，所以 platformio.ini 的 -D 依然能覆盖。
#ifndef STEP_SEQ_MODE
#if defined(STEP_SEQ_HARMONIC_ENABLE) && (STEP_SEQ_HARMONIC_ENABLE)
#define STEP_SEQ_MODE 2  // 兼容旧的开关写法
#else
#define STEP_SEQ_MODE 1
#endif
#endif

#ifndef STEP_SEQ_HARMONIC_FREQ_HZ
#define STEP_SEQ_HARMONIC_FREQ_HZ 180.0f
#endif

#ifndef STEP_SEQ_HARMONIC_DEPTH
#define STEP_SEQ_HARMONIC_DEPTH 0.05f
#endif

// 平滑铺满的预算比例（千分比）。不能给 1000：每个窗口都擦着边界结束的话，
// 控制任务偶尔晚几微秒，下一窗就会把没发完的尾巴砍掉（真丢步）。
#ifndef STEP_SEQ_SPREAD_PERMILLE
#define STEP_SEQ_SPREAD_PERMILLE 900
#endif

// ARR 序列生成：把「本窗要发 N 步」翻译成 TMR2 每个周期该等多少 tick。
//
// 这里是纯算法 + 一个跨窗口连续的谐波相位，不依赖控制器的任何成员。
// 外部跟随、梯形规划、角度模式三条路径全都走这一份，保证脉冲形状完全一致。
//
// 三档模式由 STEP_SEQ_MODE 选择（见 platformio.ini）：
//   0 = 恒速突发：窗口开头用 min_ticks 一口气打完，简单、永不丢步，但位置是硬台阶
//   1 = 平滑铺满：把 N 步均匀铺到整窗，台阶被摊平，位置/速度连续（默认）
//   2 = 平滑 + 谐波注入：在 1 的基础上对步频做正弦调频，把窄带能量摊到边带
namespace step_sequencer
{
    // 填充一个 ramp 窗口内的 ARR（每步间隔）序列。
    //   base_ticks  ：恒速模式下每步的固定间隔（tick，1 tick = 4ns）
    //   min_ticks   ：允许的最小步距，必须 > 脉宽 tick，否则高电平被截断直接丢步
    //   budget_ticks：本窗口可用的总 tick 预算，平滑模式下用它守恒
    // 头尾两格留给换向 guard，不写脉冲。
    void fill(uint32_t* seq, uint32_t seq_count, uint32_t base_ticks,
              uint32_t min_ticks, uint32_t budget_ticks);

    // 头尾插换向 guard。switch_direction=false 时头部只放 1 tick
    // （不能放 0：初始 ARR 为 0 会让定时器永久卡住）。
    void addDirectionGuard(uint32_t* seq, uint32_t seq_count, uint32_t guard_tick,
                           bool switch_direction);

    // 下一局运动从 0 相位重新开始注入（只在 STEP_SEQ_MODE == 2 时有实际效果）
    void resetHarmonicPhase();
}

#endif
