#ifndef STEP_OUTPUT_CHANNEL_H
#define STEP_OUTPUT_CHANNEL_H

#include <cstdint>

// 一个 ramp 窗口内 ARR 序列能装的最大步数（头尾各留 1 个换向 guard，数组长度为 N+2）。
// 物理上限由最小步距决定：最小步距 = 2 x 脉宽(200ns) = 400ns，
// 50us / 400ns = 125 步，所以 128 已经贴到上限；真发满 128 步会略超窗
// （128 x 400ns = 51.2us），超出部分靠外部跟随的 dropped 计数暴露。
#define MAX_PULSE_ARR_LEN 128U

// TMR2 + DMA1_CH2 的脉冲出口。
//
// 规划和外部跟随在逻辑上互斥（跟随时 startMotion 直接忽略），但它们共用同一套硬件，
// 所以也共用这一份 ARR 序列缓冲和启动流程 —— 以前是两套几乎逐字重复的代码，
// 改动时很容易只改一边，导致两条路径的脉冲形状悄悄跑偏。
//
// 这一层只负责“把 N 步铺进窗口并启动定时器”，不关心方向从哪来、发多少步是谁算的；
// 步数记账（emitted / delivered / pending）仍然留在使用方，因为两边的记账语义不同。
// TMC2209 STEP 输入合法最小周期 2µs（高/低各 ≥1µs）。4ns/tick → 500 tick。
// 这是"窗口在合法节拍下最多能塞几步"的依据，也是 emit 内步距的下限：
// 债务窗（控制任务被抢之后的补账窗）宁可欠发留到下窗，也绝不 351ns/2.8Mpps 爆发。
constexpr uint32_t kMinDriverStepPeriodTicks = 500U;

class StepOutputChannel
{
public:
    // 本窗把 num_steps 步发出去。direction = true 表示正转。
    // direction_changed 为 true 时序列头部插换向 guard（外部 DIR 随时可能翻转，
    // 换向那一窗不插 guard 会让驱动器认错方向甚至整步丢掉）。
    // 返回 false 表示本窗没有真正发出：步数为 0、超过 MAX_PULSE_ARR_LEN、
    // 或窗口预算连头尾 guard 都放不下。
    bool emit(uint32_t num_steps, bool direction, bool direction_changed,
              uint64_t window_ns, uint32_t step_pulse_ticks);

    // 窗口在驱动合法节拍（≥2µs/步）下的最大可发步数。调用方用它先夹实发步数，
    // 余下的债务留在累加器逐窗偿还——比"超合法速率爆发 + 下窗 halt 截断静默丢步"
    // 体面得多（450RPM 失步风暴的根源就在这两个环节的叠加）。
    static uint32_t windowCapacity(uint64_t window_ns);

    // 立刻停 TMR2 + DMA 并把计数器清零（stopMotion 用）。
    void stop();

private:
    // 从停止定时器到重新装载 DMA，再到启动计数器；必须成对调用，不能拆开单独用。
    static void startSequence(uint32_t* seq, uint32_t seq_count, bool direction);

    // 唯一的 ARR 序列缓冲。以前规划和外部跟随各有一份 520 字节的数组，
    // 两条路径的填充/启动代码也几乎逐字重复，改一边忘一边就会悄悄跑偏。
    uint32_t seq_buffer_[MAX_PULSE_ARR_LEN + 2] = {};
};

#endif
