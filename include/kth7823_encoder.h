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
};
#endif
