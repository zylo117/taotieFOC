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
    float readFilteredAngle() override;
    bool updateFilteredSample() override;
    bool magneticFieldHigh() const override;
    bool magneticFieldLow() const override;
    uint16_t lastRawFrame() const;
    float lastFrameTheta() const;
    float lastFrameAngle() const;
    uint16_t lastTxFrame() const;
    uint32_t readCount() const;
    uint32_t allOnesCount() const;
    uint32_t allZerosCount() const;
    uint8_t misoLevel() const;
    void setZero(float zero_angle) override;
    bool calibrate(const EncoderCalibrationConfig& config, EncoderCalibrationResult* result) override;
    void setFilterWindowSize(uint8_t window_size);
    void configureFilter(const EncoderFilterConfig& config);
    EncoderFilterConfig filterConfig() const;

private:
    float zero_angle_;

#ifdef USE_POLAR_COORDINATES_WRAP
    //==== 【极坐标XY滑动平均（圆周角度正确滤波）】====
    // 将角度转为单位圆坐标: X = cosθ , Y = sinθ
    // 对X、Y分别做环形滑动平均，再atan2(Xavg,Yavg)还原角度，解决0<->65535跨圈跳变问题
    float win_s_[EncoderFilterConfig::kDefaultWindow];  // Y = sinθ 窗口缓存
    float win_c_[EncoderFilterConfig::kDefaultWindow];  // X = cosθ 窗口缓存
    float sum_s_;                                        // Y坐标累加和
    float sum_c_;                                        // X坐标累加和
#else
    int32_t  delta_win_[EncoderFilterConfig::kDefaultWindow];
    int32_t  sum_delta_;
    uint16_t angle_base_;       //基准角度 0‑65535
#endif

    uint8_t filter_window_size_;
    uint8_t filter_index_;
    uint8_t filter_count_;
    EncoderFilterConfig filter_config_;
    volatile uint16_t last_frame_raw_;
    volatile float last_frame_theta_;   //单次原始raw对应的弧度
    volatile float filtered_theta_;
    volatile uint16_t last_tx_frame_;
    volatile uint32_t read_count_;
    volatile uint32_t all_ones_count_;
    volatile uint32_t all_zeros_count_;
};
#endif
