#include "load_angle.h"

#include "at32f403a_407.h"

#include <cmath>

extern "C"
{
#include "flash.h"
}

namespace
{
    uint16_t load_angle_flash_words[LoadAngle::MAX_POINTS * 3U];
}

LoadAngle::LoadAngle()
    : point_count_(0U), received_count_(0U), fitted_(false),
    power_coefficient_(0.0f), intercept_deg_(0.0f), exponent_(0.5f)
{
    for (uint16_t i = 0U; i < MAX_POINTS; ++i)
    {
        points_[i].speed_rpm = 0U;
        points_[i].load_angle_mdeg = 0;
        received_[i] = false;
    }
}

bool LoadAngle::beginUpload(uint16_t count)
{
    if (count < 2U || count > MAX_POINTS)
    {
        return false;
    }
    point_count_ = count;
    received_count_ = 0U;
    fitted_ = false;
    power_coefficient_ = 0.0f;
    intercept_deg_ = 0.0f;
    exponent_ = 0.5f;
    for (uint16_t i = 0U; i < point_count_; ++i)
    {
        received_[i] = false;
    }
    return true;
}

bool LoadAngle::setPoint(uint16_t index, uint16_t speed_rpm, int32_t load_angle_mdeg)
{
    if (index >= point_count_)
    {
        return false;
    }
    points_[index].speed_rpm = speed_rpm;
    points_[index].load_angle_mdeg = load_angle_mdeg;
    if (!received_[index])
    {
        received_[index] = true;
        received_count_++;
    }
    fitted_ = false;
    return true;
}

bool LoadAngle::fit()
{
    if (point_count_ < 2U || received_count_ != point_count_)
    {
        fitted_ = false;
        return false;
    }

    const double count = static_cast<double>(point_count_);
    double best_error = HUGE_VAL;
    double best_coefficient = 0.0;
    double best_intercept = 0.0;
    double best_exponent = 0.5;
    double lower_exponent = 0.05;
    double upper_exponent = 0.95;
    double exponent_step = 0.05;

    // For each exponent, solve the best coefficient and offset with linear least squares.
    for (uint8_t refinement = 0U; refinement < 5U; ++refinement)
    {
        for (double exponent = lower_exponent; exponent <= upper_exponent + 1.0e-9; exponent += exponent_step)
        {
            double sum_power = 0.0;
            double sum_angle = 0.0;
            double sum_power_squared = 0.0;
            double sum_power_angle = 0.0;
            for (uint16_t i = 0U; i < point_count_; ++i)
            {
                const double speed = static_cast<double>(points_[i].speed_rpm);
                const double power = speed > 0.0 ? std::pow(speed, exponent) : 0.0;
                const double angle = static_cast<double>(points_[i].load_angle_mdeg) / 1000.0;
                sum_power += power;
                sum_angle += angle;
                sum_power_squared += power * power;
                sum_power_angle += power * angle;
            }

            const double denominator = count * sum_power_squared - sum_power * sum_power;
            if (denominator <= 1.0e-12)
            {
                continue;
            }
            const double coefficient = (count * sum_power_angle - sum_power * sum_angle) / denominator;
            const double intercept = (sum_angle - coefficient * sum_power) / count;
            double squared_error = 0.0;
            for (uint16_t i = 0U; i < point_count_; ++i)
            {
                const double speed = static_cast<double>(points_[i].speed_rpm);
                const double power = speed > 0.0 ? std::pow(speed, exponent) : 0.0;
                const double angle = static_cast<double>(points_[i].load_angle_mdeg) / 1000.0;
                const double error = angle - (intercept + coefficient * power);
                squared_error += error * error;
            }

            if (squared_error < best_error)
            {
                best_error = squared_error;
                best_coefficient = coefficient;
                best_intercept = intercept;
                best_exponent = exponent;
            }
        }

        lower_exponent = best_exponent - exponent_step;
        upper_exponent = best_exponent + exponent_step;
        if (lower_exponent < 0.05) lower_exponent = 0.05;
        if (upper_exponent > 0.95) upper_exponent = 0.95;
        exponent_step *= 0.1;
    }

    if (best_error == HUGE_VAL)
    {
        fitted_ = false;
        return false;
    }
    power_coefficient_ = static_cast<float>(best_coefficient);
    intercept_deg_ = static_cast<float>(best_intercept);
    exponent_ = static_cast<float>(best_exponent);
    fitted_ = true;
    return true;
}

bool LoadAngle::saveToFlash()
{
    if (!fitted_ || point_count_ < 2U)
    {
        return false;
    }

    uint16_t* words = load_angle_flash_words;
    uint16_t word_count = 0U;
    uint16_t checksum = 0U;
    for (uint16_t i = 0U; i < point_count_; ++i)
    {
        const uint32_t angle_bits = static_cast<uint32_t>(points_[i].load_angle_mdeg);
        words[word_count++] = points_[i].speed_rpm;
        words[word_count++] = static_cast<uint16_t>(angle_bits & 0xFFFFU);
        words[word_count++] = static_cast<uint16_t>(angle_bits >> 16U);
        checksum = static_cast<uint16_t>(checksum + words[word_count - 3U] +
                                         words[word_count - 2U] + words[word_count - 1U]);
    }

    uint16_t metadata[8] = {
        static_cast<uint16_t>(FLASH_MAGIC & 0xFFFFU),
        static_cast<uint16_t>(FLASH_MAGIC >> 16U),
        point_count_,
        checksum,
        0U,
        0U,
        0U,
        0U
    };
    if (flash_write(FLASH_DATA_BASE, words, word_count) != SUCCESS)
    {
        return false;
    }

    metadata[4] = 1U;
    return flash_write(FLASH_META_BASE, metadata, 8U) == SUCCESS;
}

bool LoadAngle::loadFromFlash()
{
    uint16_t metadata[8];
    flash_read(FLASH_META_BASE, metadata, 8U);
    const uint32_t magic = static_cast<uint32_t>(metadata[0]) |
                           (static_cast<uint32_t>(metadata[1]) << 16U);
    const uint16_t count = metadata[2];
    if (magic != FLASH_MAGIC || metadata[4] != 1U || count < 2U || count > MAX_POINTS)
    {
        point_count_ = 0U;
        fitted_ = false;
        return false;
    }

    const uint16_t word_count = static_cast<uint16_t>(count * 3U);
    uint16_t* words = load_angle_flash_words;
    flash_read(FLASH_DATA_BASE, words, word_count);
    uint16_t checksum = 0U;
    for (uint16_t i = 0U; i < word_count; ++i)
    {
        checksum = static_cast<uint16_t>(checksum + words[i]);
    }
    if (checksum != metadata[3])
    {
        point_count_ = 0U;
        fitted_ = false;
        return false;
    }

    point_count_ = count;
    received_count_ = count;
    for (uint16_t i = 0U; i < count; ++i)
    {
        const uint16_t offset = static_cast<uint16_t>(i * 3U);
        const uint32_t angle_bits = static_cast<uint32_t>(words[offset + 1U]) |
                                    (static_cast<uint32_t>(words[offset + 2U]) << 16U);
        points_[i].speed_rpm = words[offset];
        points_[i].load_angle_mdeg = static_cast<int32_t>(angle_bits);
        received_[i] = true;
    }
    return fit();
}

bool LoadAngle::getPoint(uint16_t index, uint16_t* speed_rpm, int32_t* load_angle_mdeg) const
{
    if (index >= point_count_ || speed_rpm == nullptr || load_angle_mdeg == nullptr)
    {
        return false;
    }
    *speed_rpm = points_[index].speed_rpm;
    *load_angle_mdeg = points_[index].load_angle_mdeg;
    return true;
}

uint16_t LoadAngle::pointCount() const
{
    return point_count_;
}

bool LoadAngle::isFitted() const
{
    return fitted_;
}

float LoadAngle::powerCoefficient() const
{
    return power_coefficient_;
}

float LoadAngle::interceptDeg() const
{
    return intercept_deg_;
}

float LoadAngle::exponent() const
{
    return exponent_;
}

int32_t LoadAngle::getLoadAngleMilliDegrees(uint16_t speed_rpm) const
{
    if (!fitted_)
    {
        return 0;
    }
    const float speed_power = speed_rpm > 0U ? powf(static_cast<float>(speed_rpm), exponent_) : 0.0f;
    const float estimate_mdeg = (intercept_deg_ + power_coefficient_ * speed_power) * 1000.0f;
    if (estimate_mdeg > 2147483000.0f)
    {
        return 2147483000;
    }
    if (estimate_mdeg < -2147483000.0f)
    {
        return -2147483000;
    }
    return static_cast<int32_t>(estimate_mdeg);
}