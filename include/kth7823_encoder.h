#ifndef KTH7823_ENCODER_H
#define KTH7823_ENCODER_H

#include "angle_encoder.h"

class Kth7823Encoder : public AngleEncoder
{
public:
    Kth7823Encoder();
    bool init() override;
    uint16_t readRawAngle() override;
    uint16_t readFilteredRawAngle() override;
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
    void setZero(uint16_t zero_angle) override;
    bool calibrate(const EncoderCalibrationConfig& config, EncoderCalibrationResult* result) override;
    void setFilterWindowSize(uint8_t window_size);
    void configureFilter(const EncoderFilterConfig& config);
    EncoderFilterConfig filterConfig() const;

private:
    uint16_t zero_angle_;
    volatile uint16_t filtered_raw_angle_;
    uint16_t filter_window_[EncoderFilterConfig::kDefaultWindow];
    uint8_t filter_window_size_;
    uint8_t filter_index_;
    uint8_t filter_count_;
    uint32_t filter_sum_;
    EncoderFilterConfig filter_config_;
    volatile uint16_t last_raw_frame_;
    volatile float last_frame_theta_;
    volatile uint16_t last_tx_frame_;
    volatile uint32_t read_count_;
    volatile uint32_t all_ones_count_;
    volatile uint32_t all_zeros_count_;
};

#endif
