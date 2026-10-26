#include "pulse_capture.h"

#include "at32f403a_407.h"

// ===========================================================================
// 定时器 / 通道 / 引脚选择
// ---------------------------------------------------------------------------
// PULSE_CAPTURE_USE_TMR3 = 1（默认，已量产接线）：
//   PB5 -> TMR3_CH2 = PUL，PB4 -> TMR3_CH1 = DIR
//   TMR3_MUX_10 同时把 CH3/CH4 映射到 PB0/PB1：本模块不使能这两通道，
//   PB1（编码器 MGH）仍是普通 GPIO 输入，读写不受影响。
//   PB4 默认是 NJTRST，必须先关 JTAG-DP（保留 SW-DP）才能当复用输入。
//   TMR2 完全留给内部 STEP/DIR 的 DDA 输出（CH3/CH4 = PB10/PB11）。
// PULSE_CAPTURE_USE_TMR3 = 0（旧方案）：
//   PB3 -> TMR2_CH2 = PUL，PA15 -> TMR2_CH1 = DIR，此时采集与 DDA 输出共用 TMR2。
// ===========================================================================
#if PULSE_CAPTURE_USE_TMR3
#define PULSE_CAPTURE_TMR            TMR3
#define PULSE_CAPTURE_CRM_CLOCK      CRM_TMR3_PERIPH_CLOCK
#define PULSE_CAPTURE_REMAP          TMR3_MUX_10 // ch1(pb4) / ch2(pb5) / ch3(pb0) / ch4(pb1)
#define PULSE_CAPTURE_CH1_PORT       GPIOB
#define PULSE_CAPTURE_CH1_PIN        GPIO_PINS_4
#define PULSE_CAPTURE_CH2_PORT       GPIOB
#define PULSE_CAPTURE_CH2_PIN        GPIO_PINS_5
#define PULSE_CAPTURE_FLEX_CH1       DMA_FLEXIBLE_TMR3_CH1
#define PULSE_CAPTURE_FLEX_CH2       DMA_FLEXIBLE_TMR3_CH2
#else
#define PULSE_CAPTURE_TMR            TMR2
#define PULSE_CAPTURE_CRM_CLOCK      CRM_TMR2_PERIPH_CLOCK
#define PULSE_CAPTURE_REMAP          TMR2_MUX_11 // ch1_ext(pa15) / ch2(pb3) / ch3(pb10) / ch4(pb11)
#define PULSE_CAPTURE_CH1_PORT       GPIOA
#define PULSE_CAPTURE_CH1_PIN        GPIO_PINS_15
#define PULSE_CAPTURE_CH2_PORT       GPIOB
#define PULSE_CAPTURE_CH2_PIN        GPIO_PINS_3
#define PULSE_CAPTURE_FLEX_CH1       DMA_FLEXIBLE_TMR2_CH1
#define PULSE_CAPTURE_FLEX_CH2       DMA_FLEXIBLE_TMR2_CH2
#endif

#if PULSE_CAPTURE_PUL_ON_CH1
#define PULSE_CAPTURE_CH           TMR_SELECT_CHANNEL_1
#define PULSE_CAPTURE_CCR          (&PULSE_CAPTURE_TMR->c1dt)
#define PULSE_CAPTURE_DMA_REQUEST  TMR_C1_DMA_REQUEST
#define PULSE_CAPTURE_FLEX_REQUEST PULSE_CAPTURE_FLEX_CH1
#define PULSE_CAPTURE_PUL_PORT     PULSE_CAPTURE_CH1_PORT
#define PULSE_CAPTURE_PUL_PIN      PULSE_CAPTURE_CH1_PIN
#define PULSE_CAPTURE_DIR_CH       TMR_SELECT_CHANNEL_2
#define PULSE_CAPTURE_DIR_PORT     PULSE_CAPTURE_CH2_PORT
#define PULSE_CAPTURE_DIR_PIN      PULSE_CAPTURE_CH2_PIN
#else
#define PULSE_CAPTURE_CH           TMR_SELECT_CHANNEL_2
#define PULSE_CAPTURE_CCR          (&PULSE_CAPTURE_TMR->c2dt)
#define PULSE_CAPTURE_DMA_REQUEST  TMR_C2_DMA_REQUEST
#define PULSE_CAPTURE_FLEX_REQUEST PULSE_CAPTURE_FLEX_CH2
#define PULSE_CAPTURE_PUL_PORT     PULSE_CAPTURE_CH2_PORT
#define PULSE_CAPTURE_PUL_PIN      PULSE_CAPTURE_CH2_PIN
#define PULSE_CAPTURE_DIR_CH       TMR_SELECT_CHANNEL_1
#define PULSE_CAPTURE_DIR_PORT     PULSE_CAPTURE_CH1_PORT
#define PULSE_CAPTURE_DIR_PIN      PULSE_CAPTURE_CH1_PIN
#endif

// DIR 快照掩码：DMA 搬的是整个 GPIO 输入寄存器，消费时按位取出 DIR 那一根脚
#define PULSE_CAPTURE_DIR_MASK     ((uint16_t)PULSE_CAPTURE_DIR_PIN)

namespace
{
    // 时间戳环用 DMA1_CH3，DIR 快照环用 DMA1_CH4：两者共用同一个捕获请求。
    // CH1 = ADC 相电流，CH2 = TMR2 overflow（内部 STEP 的 ARR 序列），这里不复用。
    constexpr uint16_t kRingDepth = PULSE_CAPTURE_RING_DEPTH;
    constexpr uint16_t kRingMask = kRingDepth - 1U;

    static_assert((kRingDepth & kRingMask) == 0U, "PULSE_CAPTURE_RING_DEPTH 必须是 2 的幂");

#if PULSE_CAPTURE_TICK_DMA
    volatile uint16_t s_tick_ring[kRingDepth]; // DMA 搬运：捕获寄存器 CCR（时间戳）
#endif
    volatile uint16_t s_dir_ring[kRingDepth];  // DMA 搬运：GPIO 输入寄存器（DIR 快照原始值）

    bool s_initialized = false;
    uint16_t s_rd_index = 0U;              // 软件读指针，只有 rampUpdate 上下文推进
    int64_t s_cmd_pulse_total = 0;         // 累计有符号指令脉冲
    bool s_tick_ring_valid = true;         // 时间戳环是否和 DIR 环同步推进

#if PULSE_CAPTURE_DEBUG
    uint32_t s_captured_count = 0U;        // 已消费脉冲总数
    uint16_t s_max_pending = 0U;           // 历史最大积压深度
#endif

    // DMA 写指针：循环模式下 CNT 从环深递减到 1 后回载环深，wr = 环深 - 剩余计数
    inline uint16_t ringWriteIndex(dma_channel_type* channel)
    {
        return static_cast<uint16_t>((kRingDepth - dma_data_number_get(channel)) & kRingMask);
    }
} // namespace

volatile bool pulse_buf_overflow_flag = false;

void pulse_capture_init(void)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(PULSE_CAPTURE_CRM_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    // 1. 关 JTAG-DP 只保留 SW-DP：PB4(NJTRST) 才能当定时器复用输入
    //    （USE_TMR3=0 的旧方案里 PA15/PB3 也是靠这一句释放的）
    gpio_pin_remap_config(SWJTAG_MUX_010, TRUE);

    // 2. 捕获定时器重映射：TMR3_MUX_10 -> CH1=PB4(DIR)、CH2=PB5(PUL)
    //    CH3/CH4 同时被映射到 PB0/PB1，但不使能就不影响 PB1(编码器 MGH) 的 GPIO 读取
    gpio_pin_remap_config(PULSE_CAPTURE_REMAP, TRUE);

    // 3. 两路输入脚切到定时器复用输入。PUL/DIR 由上位机推挽驱动，
    //    若实际是开漏/光耦且空闲时悬空，需要外部上拉或把 PUL 改成 GPIO_PULL_DOWN。
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_pins = PULSE_CAPTURE_PUL_PIN;
    gpio_init(PULSE_CAPTURE_PUL_PORT, &gpio_init_struct);
    gpio_init_struct.gpio_pins = PULSE_CAPTURE_DIR_PIN;
    gpio_init(PULSE_CAPTURE_DIR_PORT, &gpio_init_struct);

    // 4. 捕获定时器：PSC=0、ARR=0xFFFF、向上自由运行。
    //    计数时钟 = system_core_clock（APB1 预分频=2，AT32 定时器硬件自动倍频回主频），
    //    所以 1 tick = 1/system_core_clock 秒，约 4ns @250MHz，16 位约 262us 回绕。
    tmr_counter_enable(PULSE_CAPTURE_TMR, FALSE);
    tmr_base_init(PULSE_CAPTURE_TMR, 0xFFFFU, 0U);
    tmr_cnt_dir_set(PULSE_CAPTURE_TMR, TMR_COUNT_UP);
    tmr_clock_source_div_set(PULSE_CAPTURE_TMR, TMR_CLOCK_DIV1); // 滤波采样时钟，ICF=0 时不起作用
    tmr_period_buffer_enable(PULSE_CAPTURE_TMR, FALSE);           // ARR 固定，不需要预装载

    // 5. 输入捕获：只抓上升沿，ICF=0（无滤波）、输入分频=1。
    //    脉宽最窄 100ns、脉冲最高 2.56MHz，任何滤波都会吞掉窄脉冲，这里必须全关。
    tmr_input_config_type input_config;
    tmr_input_default_para_init(&input_config);
    input_config.input_polarity_select = TMR_INPUT_RISING_EDGE;
    input_config.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT; // 本通道引脚直连，不做间接映射
    input_config.input_filter_value = 0U;

    input_config.input_channel_select = PULSE_CAPTURE_CH;
    tmr_input_channel_init(PULSE_CAPTURE_TMR, &input_config, TMR_CHANNEL_INPUT_DIV_1);

    // DIR 通道只做引脚功能声明：即使它自己产生捕获，也不会开中断/DMA，不产生任何事件处理
    input_config.input_channel_select = PULSE_CAPTURE_DIR_CH;
    tmr_input_channel_init(PULSE_CAPTURE_TMR, &input_config, TMR_CHANNEL_INPUT_DIV_1);

    tmr_channel_enable(PULSE_CAPTURE_TMR, PULSE_CAPTURE_CH, TRUE);
    tmr_channel_enable(PULSE_CAPTURE_TMR, PULSE_CAPTURE_DIR_CH, TRUE);

    // 6. DMA 请求源固定为“通道捕获事件”（不是 overflow）
    tmr_channel_dma_select(PULSE_CAPTURE_TMR, TMR_DMA_REQUEST_BY_CHANNEL);

    // 7. 时间戳环：捕获寄存器 CCR -> s_tick_ring，半字搬运，循环模式
#if PULSE_CAPTURE_TICK_DMA
    dma_init_type dma_config;
    dma_default_para_init(&dma_config);
    dma_config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_config.buffer_size = kRingDepth;
    dma_config.peripheral_base_addr = reinterpret_cast<uint32_t>(PULSE_CAPTURE_CCR);
    dma_config.memory_base_addr = reinterpret_cast<uint32_t>(const_cast<uint16_t*>(s_tick_ring));
    dma_config.peripheral_inc_enable = FALSE;
    dma_config.memory_inc_enable = TRUE;
    dma_config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_config.loop_mode_enable = TRUE;
    dma_config.priority = DMA_PRIORITY_HIGH;
    dma_flexible_config(DMA1, FLEX_CHANNEL3, PULSE_CAPTURE_FLEX_REQUEST);
    dma_init(DMA1_CHANNEL3, &dma_config);
    dma_flag_clear(DMA1_FDT3_FLAG | DMA1_HDT3_FLAG | DMA1_DTERR3_FLAG);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);
#endif

    // 8. DIR 快照环：GPIO 输入寄存器 -> s_dir_ring，由同一个捕获请求在同一时刻触发。
    //    这里搬的是原始引脚快照（整个端口的 IDT），消费时再用掩码取出 DIR 位。
    dma_init_type dir_dma_config;
    dma_default_para_init(&dir_dma_config);
    dir_dma_config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dir_dma_config.buffer_size = kRingDepth;
    dir_dma_config.peripheral_base_addr =
        reinterpret_cast<uint32_t>(&PULSE_CAPTURE_DIR_PORT->idt);
    dir_dma_config.memory_base_addr = reinterpret_cast<uint32_t>(const_cast<uint16_t*>(s_dir_ring));
    dir_dma_config.peripheral_inc_enable = FALSE;
    dir_dma_config.memory_inc_enable = TRUE;
    dir_dma_config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dir_dma_config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dir_dma_config.loop_mode_enable = TRUE;
    dir_dma_config.priority = DMA_PRIORITY_VERY_HIGH; // DIR 快照是核心数据，优先抢占总线
    dma_flexible_config(DMA1, FLEX_CHANNEL4, PULSE_CAPTURE_FLEX_REQUEST);
    dma_init(DMA1_CHANNEL4, &dir_dma_config);
    dma_flag_clear(DMA1_FDT4_FLAG | DMA1_HDT4_FLAG | DMA1_DTERR4_FLAG);
    dma_channel_enable(DMA1_CHANNEL4, TRUE);

    // 9. DMA 通道全部就绪后再打开捕获请求，避免第一个请求落到未初始化的通道上
    tmr_dma_request_enable(PULSE_CAPTURE_TMR, PULSE_CAPTURE_DMA_REQUEST, TRUE);

    // 10. 清状态后启动计数器，之后 CNT 自由运行，任何地方都不再清零
    s_rd_index = 0U;
    s_cmd_pulse_total = 0;
    s_tick_ring_valid = true;
    pulse_buf_overflow_flag = false;
#if PULSE_CAPTURE_DEBUG
    s_captured_count = 0U;
    s_max_pending = 0U;
#endif

    tmr_counter_value_set(PULSE_CAPTURE_TMR, 0U);
    tmr_counter_enable(PULSE_CAPTURE_TMR, TRUE);

    s_initialized = true;
}

int32_t pulse_capture_read_delta(void)
{
    if (!s_initialized)
    {
        return 0;
    }

    // DMA 写的是 SRAM，读指针之前先保证看到 DMA 已落盘的数据
    __DMB();

    // 循环模式下每次绕圈都会置位全满标志：上次消费之后 DMA 已经写满一整圈以上，
    // 意味着 rd 处那一条已经被覆盖（只有调度卡死/断点才会出现），必须报丢脉冲。
    const bool ring_wrapped = dma_flag_get(DMA1_FDT4_FLAG) != RESET;
    if (ring_wrapped)
    {
        dma_flag_clear(DMA1_FDT4_FLAG);
    }
    // 传输错误（总线被抢/地址非法）会让通道静默停摆，同样当作丢脉冲报错
    const bool dma_bus_error = dma_flag_get(DMA1_DTERR4_FLAG) != RESET;
    if (dma_bus_error)
    {
        dma_flag_clear(DMA1_DTERR4_FLAG);
    }

    const uint16_t wr_index = ringWriteIndex(DMA1_CHANNEL4);
    const uint16_t pending = static_cast<uint16_t>((wr_index - s_rd_index) & kRingMask);

#if PULSE_CAPTURE_DEBUG
    if (pending > s_max_pending)
    {
        s_max_pending = pending;
    }
#endif

#if PULSE_CAPTURE_TICK_DMA
    // 两个通道由同一个捕获请求触发，正常情况下写指针一致；
    // 只允许时间戳通道落后 1 个（DMA 仲裁正在搬运第二个通道）。
    if (s_tick_ring_valid)
    {
        const uint16_t tick_gap = static_cast<uint16_t>(
            (ringWriteIndex(DMA1_CHANNEL3) - wr_index) & kRingMask);
        if (tick_gap != 0U && tick_gap != kRingMask)
        {
            s_tick_ring_valid = false; // 时间戳环失效，退化为只用 DIR 快照
        }
    }
#endif

    if (ring_wrapped || dma_bus_error)
    {
        // 写指针已经绕圈追上读指针：环形缓冲区里的旧数据被覆盖，丢弃并重新对齐，
        // 置位全局故障标记交给上层报错（丢脉冲必须让上层知道）。
        pulse_buf_overflow_flag = true;
        s_rd_index = wr_index;
        return 0;
    }

    if (pending == 0U)
    {
        return 0;
    }

    int32_t delta = 0;
    for (uint16_t count = 0U; count < pending; ++count)
    {
        PulseSnap_t snap;
#if PULSE_CAPTURE_TICK_DMA
        snap.tmr_tick = s_tick_ring_valid ? s_tick_ring[s_rd_index] : 0U;
#else
        snap.tmr_tick = 0U;
#endif
        snap.dir_snap = (s_dir_ring[s_rd_index] & PULSE_CAPTURE_DIR_MASK) != 0U ? 1U : 0U;

        // DIR 低电平 = 正方向 +1，高电平 = 反方向 -1
        delta += (snap.dir_snap == 0U) ? 1 : -1;

        s_rd_index = static_cast<uint16_t>((s_rd_index + 1U) & kRingMask);
    }

    s_cmd_pulse_total += delta;

#if PULSE_CAPTURE_DEBUG
    s_captured_count += pending;
#endif

    return delta;
}

int64_t pulse_capture_get_total(void)
{
    return s_cmd_pulse_total;
}

void pulse_capture_clear_overflow_flag(void)
{
    pulse_buf_overflow_flag = false;
}

#if PULSE_CAPTURE_DEBUG
uint32_t pulse_capture_get_captured_count(void)
{
    return s_captured_count;
}

uint16_t pulse_capture_get_max_pending(void)
{
    return s_max_pending;
}
#endif
