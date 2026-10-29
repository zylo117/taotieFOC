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

// 调试开关：置 1 后额外统计最大积压深度，方便和信号发生器对拍是否丢脉冲
#ifndef PULSE_CAPTURE_DEBUG
#define PULSE_CAPTURE_DEBUG 0
#endif

// 自检打印开关：置 1 后 telemetry 任务每秒打印一行 [PUL] 摘要（采集数 / 累计角度 /
// PUL·DIR 实时电平 / DMA 积压 / 溢出），用来在接线上官司时判断“到底收到了没有”。
// 关掉它可以完全不影响时序。
#ifndef PULSE_CAPTURE_SELFCHECK_LOG
#define PULSE_CAPTURE_SELFCHECK_LOG 1
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

// 注意：这里不再假设固定细分。1 个外部 PUL 脉冲 = 1 个驱动器微步，
// 一圈脉冲数 = 200 步/圈 x 驱动器实际微步数，运行期由
// ClosedLoopController::getStepsPerRound() 给出（默认 200 x 64 = 12800）。
// （旧代码写死的 51200 = 200x256 只是打印口径，会让 [PUL] 的 deg 恒为真实值的 1/4。）

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

// 与 pulse_capture_read_delta() 完全等价，但额外输出本窗所有脉冲的绝对值和。
// abs_sum != |返回值| 说明这一窗里 DIR 至少翻转过一次（窗内换向）：输出层一个
// DMA 序列只能有一个方向、一次 guard，所以"先正走后反走"的轨迹会被净和压平。
// 终点位置仍然正确，只是中间那段往返没有执行 —— 有背隙/弹性传动的机构会
// 按换向次数累积偏差。只读统计用，不影响任何输出行为。
int32_t pulse_capture_read_delta_ex(int32_t* abs_sum);

// 上电至今的累计有符号指令脉冲数（角度 = total * 360.0f / 每圈步数）
int64_t pulse_capture_get_total(void);

// 清溢出故障标记（重新对齐读写指针后由上层确认用）
void pulse_capture_clear_overflow_flag(void);

// 上电至今已被消费的捕获事件总数（不分方向），用来和信号发生器的发送脉冲数对拍。
// 这个计数始终有效，不需要打开 PULSE_CAPTURE_DEBUG。
uint32_t pulse_capture_get_captured_count(void);

// 只读 PUL 引脚当前电平（GPIO IDT），极轻量，可以在高频循环里连续采样做占空统计。
// 返回 1 = MCU 认为高，0 = MCU 认为低。
uint8_t pulse_capture_get_pul_level(void);

// 注：PUL/DIR 的内部上下拉是编译期开关 PULSE_CAPTURE_INPUT_PULL_DOWN（见 pulse_capture.cpp），
// 默认下拉。它只有在引脚配成 GPIO_MODE_INPUT 时才真正生效——AT32 库的 gpio_init()
// 只在输入模式分支写上下拉，写 MUX 时 gpio_pull 会被整句丢弃（踩过这个坑）。

// 运行期电气 / 采集自检（最好在“确定没有脉冲”的时候读才有判读意义）：
//   pul_level  : PUL 引脚当前电平。UART 空闲时 TXD 恒为高，接好了就应该读到 1；
//                读到 0 说明驱动端的高电平没被 MCU 认成高电平（VIH 裕量不足 / 没共地）。
//   dir_level  : DIR 引脚当前电平，接 GND 应为 0，接 3V3 应为 1（悬空会乱跳）。
//   dma_pending: DMA 写指针与软件读指针之间的积压深度，持续为 0 才正常。
// 说明：引脚工作在定时器输入捕获时，输入数据寄存器 IDT 依然反映真实 pad 电平。
void pulse_capture_get_probe(uint8_t* pul_level, uint8_t* dir_level, uint16_t* dma_pending);

#if PULSE_CAPTURE_DEBUG
// 调试：历史最大积压深度（离溢出还有多远），越接近环深说明 rampUpdate 越吃力
uint16_t pulse_capture_get_max_pending(void);
#endif

// tick 差值 -> 纳秒。不要写字面频率，统一用 system_core_clock。
static inline float pulse_capture_ticks_to_ns(uint16_t ticks)
{
    return static_cast<float>(ticks) * 1.0e9f / static_cast<float>(system_core_clock);
}

#endif
