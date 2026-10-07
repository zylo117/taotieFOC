#ifndef PID_AUTO_TUNER_H
#define PID_AUTO_TUNER_H

#include <cstdint>

class PidAutoTuner
{
public:
    PidAutoTuner();

    void beginTrial();
    void addSample(float follow_error_deg, bool latter_half);
    void finishTrial();
    uint32_t sampleCount() const;
    uint32_t scoreMilliDegrees() const;

private:
    bool collecting_;
    uint32_t sample_count_;
    float squared_error_sum_;
    float peak_error_deg_;
    uint32_t first_half_sample_count_;
    uint32_t latter_half_sample_count_;
    float first_half_squared_error_sum_;
    float latter_half_squared_error_sum_;
};

#endif