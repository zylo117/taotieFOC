#ifndef PULSE_CAPTURE_H
#define PULSE_CAPTURE_H

#include <cstdint>

#include "at32f403a_407.h"

/*
 * PUL/DIR 指令脉冲采集：TMR3 输入捕获 + DMA 环形缓冲
 * ---------------------------------------------------------------------------
 * 目标：把上位机（Klipper/Marlin）发来的 STEP(PUL)/DIR 指令按“单个脉冲”粒度完整
 *       采集下来，并且每个脉冲都和它到来那一瞬间的 DIR 电平绑定，解决 DIR 在
 *       rampUpdate() 里被软件采样时早已翻转造成的“方向反”。
 *
 * 数据通路（全硬件，没有每脉冲中断）：
 *   PUL 上升沿 -> TMR3 捕获 -> DMA 请求 -> DMA 通道A 把捕获寄存器(CCR)搬进时间戳环
 *                                      -> DMA 通道B 同时把 GPIO 输入寄存器搬进 DIR 快照环
 *   两个通道由同一个捕获请求触发，所以两条环严格一一对应；
 *   软件只在 20kHz 的 rampUpdate() 里比较 DMA 写指针和本地读指针，批量消费。
 *
 * 硬件接线（默认，PULSE_CAPTURE_USE_TMR3 = 1，PULSE_CAPTURE_PUL_ON_CH1 = 0）：
 *   PB5 -> TMR3_CH2  PUL 输入，上升沿捕获，ICF=0 无滤波
 *   PB4 -> TMR3_CH1  DIR 输入，本身不产生任何捕获事件，只在 PUL 边沿被采样
 *   需要 gpio_pin_remap_config(TMR3_MUX_10)（PB4 默认是 NJTRST，还要关 JTAG-DP）
 *
 * 为什么 DIR 不进 CCR：
 *   定时器里“TI1 触发捕获（IC2 间接映射到 TI1）”抓进 CCR 的是计数器 CNT，而不是
 *   另一个引脚的电平，硬件上并没有“把 DIR 电平锁进 CCR2”这种模式。这里用
 *   “同一个 PUL 捕获请求触发的 DMA 读 GPIO 输入寄存器”做逐脉冲快照：延迟只有
 *   几个 HCLK（250MHz 下几十 ns），远小于 100ns 的最窄脉宽，既不是软件轮询，
 *   也不是 DIR 外部中断。
 *
 * 时间基准：
 *   TMR3 PSC=0，ARR=0xFFFF 自由运行；APB1 预分频=2 时 AT32 定时器硬件自动倍频，
 *   计数频率 = system_core_clock，1 tick = 1/system_core_clock 秒。
 *   16 位时间戳约 262us 回绕一次，代码内禁止写死 240M/250M 这类字面值。
 *
 * 定时器分工（PULSE_CAPTURE_USE_TMR3 = 1 时）：
 *   TMR2 CH3/CH4 (PB10/PB11) = 内部 STEP/DIR 的 DDA 输出（ARR 由 DMA 改写）
 *   TMR3 CH1/CH2 (PB4/PB5)   = 外部 PUL/DIR 捕获，CNT 自由运行、绝不清零
 *   TMR5                     = 磁编码器 2us 采样时基
 *   三者互不干扰，采集与内部 DDA 输出可以同时工作。
 *
 * 注意：把 PULSE_CAPTURE_USE_TMR3 改成 0 会退回“TMR2 同时做输出与捕获”的旧方案，
 *       那时 DDA 输出改写 ARR/清零 CNT 会破坏自由运行的时间戳基准，两者不能同时工作。
 */

// 调试开关：置 1 后额外统计总捕获脉冲数/最大积压深度，方便和信号发生器对拍是否丢脉冲
#ifndef PULSE_CAPTURE_DEBUG
#define PULSE_CAPTURE_DEBUG 0
#endif

// 定时器选择：1 = 用 TMR3 捕获（PB4/PB5，推荐，与 TMR2 的 STEP/DIR 输出彻底解耦）
//             0 = 用 TMR2 捕获（PA15/PB3，会和内部 DDA 输出抢同一个定时器）
#ifndef PULSE_CAPTURE_USE_TMR3
#define PULSE_CAPTURE_USE_TMR3 1
#endif

// 接线选择（只是交换同一个定时器上 CH1/CH2 的角色）：
//   0 = PUL 接 CH2（TMR3: PB5），DIR 接 CH1（TMR3: PB4）（默认）
//   1 = PUL 接 CH1（TMR3: PB4），DIR 接 CH2（TMR3: PB5）
#ifndef PULSE_CAPTURE_PUL_ON_CH1
#define PULSE_CAPTURE_PUL_ON_CH1 0
#endif

// 时间戳 DMA：两个 DMA 通道挂同一个捕获请求。若某批芯片不允许一个请求被两个通道
// 同时使用，把它设为 0 即可退化为“只有 DIR 快照”——脉冲增量/累计完全不受影响，
// 只是 tmr_tick 恒为 0。
#ifndef PULSE_CAPTURE_TICK_DMA
#define PULSE_CAPTURE_TICK_DMA 1
#endif

// 环形缓冲区深度，必须是 2 的幂（用位掩码取模）
#define PULSE_CAPTURE_RING_DEPTH 512U

// 200 步/圈 x 256 细分 = 51200 脉冲/圈
#define PULSE_CAPTURE_PULSES_PER_REV 51200.0f

// 环形缓冲区里的一条记录（软件消费时按这个结构体解释两条 DMA 环上的同一号元素）
typedef struct
{
    uint16_t tmr_tick; // TMR 捕获时间戳，时间基准 = system_core_clock
    uint8_t dir_snap;  // 该脉冲到来瞬间的 DIR 电平快照，0/1
} PulseSnap_t;

// 缓冲区溢出故障标记：DMA 写指针绕圈追上读指针（丢脉冲），置位后由上层报错/停机。
// 只能在 pulse_capture_clear_overflow_flag() 里清除。
extern volatile bool pulse_buf_overflow_flag;

// 初始化捕获定时器的双通道、GPIO 重映射、DMA 环形缓冲与缓冲区指针。
void pulse_capture_init(void);

// 在 rampUpdate() 里调用：消费本轮所有未读脉冲，返回有符号脉冲增量
// （dir_snap=0 记 +1，dir_snap=1 记 -1），并累加到内部 int64_t 累计指令脉冲。
int32_t pulse_capture_read_delta(void);

// 上电至今的累计有符号指令脉冲数（角度 = total * 360.0f / PULSE_CAPTURE_PULSES_PER_REV）
int64_t pulse_capture_get_total(void);

// 清溢出故障标记（重新对齐读写指针后由上层确认用）
void pulse_capture_clear_overflow_flag(void);

#if PULSE_CAPTURE_DEBUG
// 调试：累计已被消费的脉冲总数，用于和信号发生器的输出个数对拍
uint32_t pulse_capture_get_captured_count(void);
// 调试：历史最大积压深度（离溢出还有多远），越接近环深说明 rampUpdate 越吃力
uint16_t pulse_capture_get_max_pending(void);
#endif

// tick 差值 -> 纳秒。不要写字面频率，统一用 system_core_clock。
static inline float pulse_capture_ticks_to_ns(uint16_t ticks)
{
    return static_cast<float>(ticks) * 1.0e9f / static_cast<float>(system_core_clock);
}

#endif
