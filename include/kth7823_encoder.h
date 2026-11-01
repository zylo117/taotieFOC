#ifndef KTH7823_ENCODER_H
#define KTH7823_ENCODER_H
#include "angle_encoder.h"

// 圆周角度矢量滑动平均滤波 / 整形增量累加（相对角度）+ 基准角度滑动平均滤波（快，无浮点）
// 二选一，前者准，后者快超多（虽然看cpu占用率仅降低1.5%），前者跑高速会卡mcu
// #define USE_POLAR_COORDINATES_WRAP

class Kth7823Encoder : public AngleEncoder
{
public:
    Kth7823Encoder();
    bool init() override;
    uint16_t readRaw() override;
    uint16_t readFilteredRaw() override;
    float readFilteredAngle() override;
    bool updateFilteredSample() override;

    // ===== 【高頻独立采样 + 拍内取平均】=====
    // 背景：实测内部更新间隔约 13us（77kHz）。原来在 TMR4 中断里“喷发 4 次”（14.4us），
    // 只能拿到约 1 个独立样本，降噪几乎为 0。
    //
    // 新方案：
    //   sampleRawFast()      —— 由独立定时器（TMR5，13us）调用，只做 SPI 读 + 存环形缓冲，
    //                           不做非线性校准查表（省 1.56us/次）
    //   takeRawAverage()     —— 由 TMR4（50us）调用，把这段时间累积的所有样本环绕安全地平均，
    //                           返回一个值给 updateFilteredSample()；不碰 SPI，只要 ~0.5us
    //   updateFilteredSample(uint16_t raw) —— 只做滤波，不再读 SPI
    //
    // 效果：50us 内 3.8 个独立样本 → 降噪上限 √3.8 ≈ 1.95 倍（原方案 1.05 倍），
    //       CPU 占用相当，但 TMR4 中断时长从 14.4us 降到 ~0.5us。
    static constexpr uint32_t kRawRingSize = 8U;  // 8 x 13us = 104us > 50us 控制周期
    void sampleRawFast();
    uint16_t takeRawAverage();
    bool updateFilteredSample(uint16_t raw);
    // 环形缓冲溢出次数（说明控制环取数太慢，正常应恒为 0）
    uint32_t rawRingOverflowCount() const { return raw_ring_overflow_; }

    // ===========================================================================
    // 【角度跳变锁存】纯测量，不参与任何控制行为。
    //
    // 为什么要它：0° 锁定时的反常负向跳动是偶发的（几分钟几次），上位机轮询
    // 只能拿到“跳完之后”的结果，拿不到跳变那一拍的内部状态。而中断里不能
    // printf（300 字节 @115200 约 26ms，会掉 500 个控制拍），所以只锁存现场，
    // 由 Telemetry_task 在任务上下文把快照吐出来。
    //
    // 一次快照就能分清三个候选：
    //   avg_raw 本身就跳  -> 传感器/SPI（跳变来自采样，与滤波无关）
    //   avg_raw 不跳、in_corrected 跳 -> 校准表查表结果跳
    //   两者都不跳、只有 out 跳    -> 滤波器自己的跨拍状态（base/sum/span）造的
    // ===========================================================================
    struct JumpSnapshot
    {
        uint32_t seq;          // 第几个样本（= read_count_）
        uint32_t events;       // 累计锁存过多少次（看发生频率；只增不清）
        uint16_t avg_raw;      // 环形平均后、查表前（未校准）
        uint16_t consumed;     // 本次平均实际用了几个 TMR5 样本
        // 【真正关键的字段】SPI 刚刚返回的原始字，还没过校准表。
        // avg_raw/consumed 在“TMR5 多样本平均”被停用后永远是 0（main.cpp:343-344），
        // 而 in 已经是查表后的值 —— 没有 raw_word 就分不清：
        //   是传感器真的返回了那个怪值（电气/时序/磁场），还是
        //   传感器返回正常、被校准表映射成了那个怪值（表/接缝）。
        uint16_t raw_word;     // getCorrectedRaw() 之前的 SPI 原始字
        uint16_t reason;       // 1=输出跳变越线；2=原始值与查表值环形差过大
        uint16_t in_corrected; // 查表后送入滤波的本拍样本
        uint16_t base;         // angle_base_
        int32_t  delta;        // 本样本相对 base 的环绕差值
        int32_t  sum_delta;    // 窗口内差值和
        int32_t  evicted;      // 本拍被淘汰出窗口的旧差值
        int32_t  span;         // window_span（延迟补偿的输入）
        uint16_t win, cnt, idx;
        uint16_t prev_out;     // 上一拍输出
        uint16_t out;          // 本拍输出
        int32_t  jump;         // 带符号跳变（LSB，负=向后）
        uint32_t all_ones, all_zeros, ring_overflow;
    };

    // 有快照则拷走并清标志（返回 false = 没有新事件）。
    // 返回 false 时 *out 不变；并发写坏的话用 seq 复查一次，宁可这轮不报。
    bool takeJumpSnapshot(JumpSnapshot* out);

    // 未经过非线性校准表的滤波原始值。
    // 用途：控制器的“行程累计”必须用它，不能用校准后的值——
    // 校准表修正的是 INL（每点 ±0.35°），但修正量的积分不闭合，
    // 若用校准后的值累加行程，50 圈会累积出 9° 的比例误差（实测）。
    bool magneticFieldHigh() const override;
    bool magneticFieldLow() const override;
    uint16_t lastRawFrame() const;
    uint16_t lastTxFrame() const;
    uint32_t readCount() const;
    uint32_t allOnesCount() const;
    uint32_t allZerosCount() const;
    uint8_t misoLevel() const;
    void setZero(float zero_angle) override;
    void setFilterWindowSize(uint16_t window_size);
    void configureFilter(const EncoderFilterConfig& config);
    EncoderFilterConfig filterConfig() const;
    void requestFilterWindow(uint16_t window_size) override;

private:
    float zero_angle_;

#ifdef USE_POLAR_COORDINATES_WRAP
    //==== 【极坐标XY滑动平均（圆周角度正确滤波）】====
    // 将角度转为单位圆坐标: X = cosθ , Y = sinθ
    // 对X、Y分别做环形滑动平均，再atan2(Xavg,Yavg)还原角度，解决0<->65535跨圈跳变问题
    float win_s_[EncoderFilterConfig::kMaxWindow];  // Y = sinθ 窗口缓存
    float win_c_[EncoderFilterConfig::kMaxWindow];  // X = cosθ 窗口缓存
    float sum_s_;                                        // Y坐标累加和
    float sum_c_;                                        // X坐标累加和
#else
    int32_t  delta_win_[EncoderFilterConfig::kMaxWindow];
    int32_t  sum_delta_;
    uint16_t angle_base_;       //基准角度 0‑65535
#endif

    volatile uint16_t filter_window_size_;
    uint16_t filter_index_;
    uint16_t filter_count_;
    // 任务侧只是登记要切的窗口，采样中断自己应用（见 applyPendingFilterWindow）。
    // pending_filter_window_ 用 0xFFFF 表示“没有待处理的请求”，这样才能和窗口 0（关闭滤波）区分开。
    volatile uint16_t pending_filter_window_;
    void applyPendingFilterWindow();
    EncoderFilterConfig filter_config_;
    volatile uint16_t last_frame_raw_;
    volatile uint16_t filtered_raw_;
    // 亚 LSB 的滤波输出：整数 LSB 会凭空多出 ±0.5 LSB（±0.0027°）的量化抖动，
    // 对外给角度时用这个浮点值，控制环和上位机读数都不再被量化台阶卡住。
    volatile float filtered_angle_deg_;
    volatile uint16_t last_tx_frame_;
    volatile uint32_t read_count_;
    volatile uint32_t all_ones_count_;
    volatile uint32_t all_zeros_count_;

    // 高频采样环形缓冲（TMR5 写、TMR4 读）
    volatile uint16_t raw_ring_[kRawRingSize];
    volatile uint32_t raw_ring_head_ = 0U;      // 累计写入数（取模即下标）
    volatile uint32_t raw_ring_tail_ = 0U;      // 累计读取数
    volatile uint32_t raw_ring_overflow_ = 0U;

    // ===== 跳变锁存（见 JumpSnapshot 注释）=====
    // 门限 90 LSB = 0.5°。为什么不是更宽：建表修成单调连续之后，残留的 0° 附近
    // 异常只有约 -2°（早期那个 7~8° 是双值悬崖，已消除），正好落在旧的 2.0°
    // 门限以下——一直在跳过不记录，看起来像“没事件”。
    // 也不能更窄：其它角度上的正常抖动实测最大 0.2°，取 0.5° 给它 2.5 倍余量。
    static constexpr int32_t kJumpLatchLsb = 90;
    void checkJumpLatch(uint16_t out_raw, uint16_t in_corrected,
                        int32_t delta, int32_t evicted, int32_t span);
    JumpSnapshot jump_;
    volatile bool jump_latched_ = false;
    volatile uint32_t jump_events_ = 0U;
    uint16_t jump_prev_out_ = 0U;
    bool jump_prev_valid_ = false;
    uint16_t last_avg_raw_ = 0U;    // takeRawAverage 里的未校准平均值
    uint16_t last_avg_count_ = 0U;  // 本次平均用了几个样本
    // SPI 刚返回、还没查表的原始字（readRaw 和 sampleRawFast 都会更新）
    volatile uint16_t last_spi_word_ = 0U;
    // 原始字与查表结果的环形差超过它就算“表把值改飞了”。
    // 正常 INL 修正量约 ±0.35° = ±64 LSB，取 512 LSB（2.8°）留 8 倍余量。
    static constexpr int32_t kTableSpreadLsb = 512;
};
#endif
