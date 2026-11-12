#include "lag_angle.h"

#include "at32f403a_407.h"

#include <cmath>

extern "C"
{
#include "flash.h"
}

namespace
{
    uint16_t lag_angle_flash_words[LagAngle::MAX_POINTS * 3U];

    // float 与 32 位原始值的位级转换（存拟合参数用，不做数值换算）
    inline uint32_t f2u(float f)
    {
        union { float f; uint32_t u; } v;
        v.f = f;
        return v.u;
    }
    inline float u2f(uint32_t u)
    {
        union { uint32_t u; float f; } v;
        v.u = u;
        return v.f;
    }
}

LagAngle::LagAngle()
    : point_count_(0U), received_count_(0U), fitted_(false),
    power_coefficient_(0.0f), intercept_deg_(0.0f), exponent_(0.5f)
{
    for (uint16_t i = 0U; i < MAX_POINTS; ++i)
    {
        points_[i].speed_rpm = 0U;
        points_[i].lag_angle_mdeg = 0;
        received_[i] = false;
    }
}

bool LagAngle::beginUpload(uint16_t count)
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

bool LagAngle::setPoint(uint16_t index, uint16_t speed_rpm, int32_t lag_angle_mdeg)
{
    if (index >= point_count_)
    {
        return false;
    }
    points_[index].speed_rpm = speed_rpm;
    points_[index].lag_angle_mdeg = lag_angle_mdeg;
    if (!received_[index])
    {
        received_[index] = true;
        received_count_++;
    }
    fitted_ = false;
    return true;
}

bool LagAngle::fit()
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
                const double angle = static_cast<double>(points_[i].lag_angle_mdeg) / 1000.0;
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
                const double angle = static_cast<double>(points_[i].lag_angle_mdeg) / 1000.0;
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

bool LagAngle::saveToFlash()
{
    if (!fitted_ || point_count_ < 2U)
    {
        return false;
    }

    uint16_t* words = lag_angle_flash_words;
    uint16_t word_count = 0U;
    uint16_t checksum = 0U;
    for (uint16_t i = 0U; i < point_count_; ++i)
    {
        const uint32_t angle_bits = static_cast<uint32_t>(points_[i].lag_angle_mdeg);
        words[word_count++] = points_[i].speed_rpm;
        words[word_count++] = static_cast<uint16_t>(angle_bits & 0xFFFFU);
        words[word_count++] = static_cast<uint16_t>(angle_bits >> 16U);
        checksum = static_cast<uint16_t>(checksum + words[word_count - 3U] +
                                         words[word_count - 2U] + words[word_count - 1U]);
    }

    uint16_t metadata[16] = {
        static_cast<uint16_t>(FLASH_MAGIC & 0xFFFFU),
        static_cast<uint16_t>(FLASH_MAGIC >> 16U),
        point_count_,
        checksum,
        1U,       // [4] 数据有效标志
        1U,       // [5] 拟合参数有效标志（有则开机直接用，不再重拟合）
        0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U
    };
    // [6..11] 三个 float 各拆两个半字（lo,hi）；[12] 参数校验和。
    const uint32_t c_bits = f2u(power_coefficient_);
    const uint32_t i_bits = f2u(intercept_deg_);
    const uint32_t e_bits = f2u(exponent_);
    metadata[6]  = static_cast<uint16_t>(c_bits & 0xFFFFU);
    metadata[7]  = static_cast<uint16_t>(c_bits >> 16U);
    metadata[8]  = static_cast<uint16_t>(i_bits & 0xFFFFU);
    metadata[9]  = static_cast<uint16_t>(i_bits >> 16U);
    metadata[10] = static_cast<uint16_t>(e_bits & 0xFFFFU);
    metadata[11] = static_cast<uint16_t>(e_bits >> 16U);
    uint16_t fit_ck = 0U;
    for (int k = 6; k <= 11; ++k)
    {
        fit_ck = static_cast<uint16_t>(fit_ck + metadata[k]);
    }
    metadata[12] = fit_ck;

    if (flash_write(FLASH_DATA_BASE, words, word_count) != SUCCESS)
    {
        return false;
    }

    return flash_write(FLASH_META_BASE, metadata, 16U) == SUCCESS;
}

bool LagAngle::loadFromFlash()
{
    uint16_t metadata[16];
    flash_read(FLASH_META_BASE, metadata, 16U);
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
    uint16_t* words = lag_angle_flash_words;
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
        points_[i].lag_angle_mdeg = static_cast<int32_t>(angle_bits);
        received_[i] = true;
    }

    // 【开机不再重拟合】元数据里带了校验通过的拟合参数就直接采用，省掉每次上电
    // 几百毫秒的 fit()。旧记录（无参数标志 / 校验不过）落回 fit()，向后兼容：
    // 那批 flash 数据本就没有可用参数，只能现算。
    if (metadata[5] == 1U)
    {
        uint16_t fit_ck = 0U;
        for (int k = 6; k <= 11; ++k)
        {
            fit_ck = static_cast<uint16_t>(fit_ck + metadata[k]);
        }
        if (fit_ck == metadata[12])
        {
            const uint32_t c_bits = static_cast<uint32_t>(metadata[6]) |
                                    (static_cast<uint32_t>(metadata[7]) << 16U);
            const uint32_t i_bits = static_cast<uint32_t>(metadata[8]) |
                                    (static_cast<uint32_t>(metadata[9]) << 16U);
            const uint32_t e_bits = static_cast<uint32_t>(metadata[10]) |
                                    (static_cast<uint32_t>(metadata[11]) << 16U);
            power_coefficient_ = u2f(c_bits);
            intercept_deg_ = u2f(i_bits);
            exponent_ = u2f(e_bits);
            fitted_ = true;
            return true;
        }
    }
    return fit();
}

bool LagAngle::getPoint(uint16_t index, uint16_t* speed_rpm, int32_t* lag_angle_mdeg) const
{
    if (index >= point_count_ || speed_rpm == nullptr || lag_angle_mdeg == nullptr)
    {
        return false;
    }
    *speed_rpm = points_[index].speed_rpm;
    *lag_angle_mdeg = points_[index].lag_angle_mdeg;
    return true;
}

uint16_t LagAngle::pointCount() const
{
    return point_count_;
}

bool LagAngle::isFitted() const
{
    return fitted_;
}

float LagAngle::powerCoefficient() const
{
    return power_coefficient_;
}

float LagAngle::interceptDeg() const
{
    return intercept_deg_;
}

float LagAngle::exponent() const
{
    return exponent_;
}

int32_t LagAngle::getLagAngleMilliDegrees(uint16_t speed_rpm) const
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