#include "pid_auto_tuner.h"

#include <cmath>

PidAutoTuner::PidAutoTuner()
        : collecting_(false), sample_count_(0U), squared_error_sum_(0.0f), peak_error_deg_(0.0f),
            first_half_sample_count_(0U), latter_half_sample_count_(0U),
            first_half_squared_error_sum_(0.0f), latter_half_squared_error_sum_(0.0f)
{
}

void PidAutoTuner::beginTrial()
{
    sample_count_ = 0U;
    squared_error_sum_ = 0.0f;
    peak_error_deg_ = 0.0f;
    first_half_sample_count_ = 0U;
    latter_half_sample_count_ = 0U;
    first_half_squared_error_sum_ = 0.0f;
    latter_half_squared_error_sum_ = 0.0f;
    collecting_ = true;
}

void PidAutoTuner::addSample(float follow_error_deg, bool latter_half)
{
    if (!collecting_ || !std::isfinite(follow_error_deg) || sample_count_ == UINT32_MAX)
    {
        return;
    }

    const float abs_error = std::fabs(follow_error_deg);
    const float squared_error = follow_error_deg * follow_error_deg;
    squared_error_sum_ += squared_error;
    if (latter_half)
    {
        latter_half_squared_error_sum_ += squared_error;
        latter_half_sample_count_++;
    }
    else
    {
        first_half_squared_error_sum_ += squared_error;
        first_half_sample_count_++;
    }
    if (abs_error > peak_error_deg_)
    {
        peak_error_deg_ = abs_error;
    }
    sample_count_++;
}

void PidAutoTuner::finishTrial()
{
    collecting_ = false;
}

uint32_t PidAutoTuner::sampleCount() const
{
    return sample_count_;
}

uint32_t PidAutoTuner::scoreMilliDegrees() const
{
    if (sample_count_ < 4U)
    {
        return UINT32_MAX;
    }

    const float rms_error = std::sqrt(squared_error_sum_ / static_cast<float>(sample_count_));
    const float first_half_rms = first_half_sample_count_ > 0U
        ? std::sqrt(first_half_squared_error_sum_ / static_cast<float>(first_half_sample_count_))
        : 0.0f;
    const float latter_half_rms = latter_half_sample_count_ > 0U
        ? std::sqrt(latter_half_squared_error_sum_ / static_cast<float>(latter_half_sample_count_))
        : 0.0f;
    const float growth_penalty = latter_half_rms > first_half_rms ? latter_half_rms - first_half_rms : 0.0f;
    const float score_mdeg = (rms_error + peak_error_deg_ * 0.35f + growth_penalty) * 1000.0f;
    if (!std::isfinite(score_mdeg) || score_mdeg >= static_cast<float>(UINT32_MAX))
    {
        return UINT32_MAX - 1U;
    }
    return static_cast<uint32_t>(score_mdeg);
}