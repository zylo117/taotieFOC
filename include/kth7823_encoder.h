#ifndef KTH7823_ENCODER_H
#define KTH7823_ENCODER_H

#include "angle_encoder.h"

class Kth7823Encoder : public AngleEncoder
{
public:
    Kth7823Encoder();
    bool init() override;
    uint16_t readRawAngle() override;
    bool magneticFieldHigh() const override;
    bool magneticFieldLow() const override;
    uint16_t lastRawFrame() const;
    uint16_t lastTxFrame() const;
    uint32_t readCount() const;
    uint32_t allOnesCount() const;
    uint32_t allZerosCount() const;
    uint8_t misoLevel() const;
    void setZero(uint16_t zero_angle) override;
    bool calibrate(const EncoderCalibrationConfig& config, EncoderCalibrationResult* result) override;

private:
    uint16_t zero_angle_;
    volatile uint16_t last_raw_frame_;
    volatile uint16_t last_tx_frame_;
    volatile uint32_t read_count_;
    volatile uint32_t all_ones_count_;
    volatile uint32_t all_zeros_count_;
};

#endif
