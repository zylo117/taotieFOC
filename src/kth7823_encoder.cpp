#include "kth7823_encoder.h"
#include <math.h>
#include <string.h>
#include <dsp/fast_math_functions.h>

#include "at32f403a_407_board.h"

namespace
{
    uint16_t normalize_angle(uint16_t raw, uint16_t zero)
    {
        if (raw >= zero)
        {
            return raw - zero;
        }
        return zero - raw;
    }
}

Kth7823Encoder::Kth7823Encoder()
    : zero_angle_(0U),
#ifdef USE_POLAR_COORDINATES_WRAP
      sum_s_(0.0F), sum_c_(0.0F),
#else
      sum_delta_(0),
      angle_base_(0U),
#endif
      filter_window_size_(EncoderFilterConfig::kDefaultWindow),
      filter_index_(0U), filter_count_(0U),
      filter_config_{EncoderFilterConfig::kDefaultWindow},
      last_frame_raw_(0U), last_frame_theta_(0.0F), last_tx_frame_(0U),
      read_count_(0U), all_ones_count_(0U), all_zeros_count_(0U)
{
#ifdef USE_POLAR_COORDINATES_WRAP
    memset(win_s_, 0, sizeof(win_s_));
    memset(win_c_, 0, sizeof(win_c_));
#endif
}

bool Kth7823Encoder::init()
{
    calibrationInit();  //基类校准初始化，加载flash主表，校验fast cal checksum

    spi_init_type spi_init_struct;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

    encoder_common::encoder_gpio_config_output(KTH7823_CS_PORT, KTH7823_CS_PIN);

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = KTH7823_SCLK_PIN | KTH7823_MOSI_PIN;
    gpio_init(KTH7823_SCLK_PORT, &gpio_init_struct);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = KTH7823_MISO_PIN;
    gpio_init(KTH7823_MISO_PORT, &gpio_init_struct);

    encoder_common::encoder_gpio_config_input(KTH7823_MGH_PORT, KTH7823_MGH_PIN);
    encoder_common::encoder_gpio_config_input(KTH7823_MGL_PORT, KTH7823_MGL_PIN);

    encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);

    spi_i2s_reset(KTH7823_SPI);
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(KTH7823_SPI, &spi_init_struct);
    spi_enable(KTH7823_SPI, TRUE);

    configureFilter({EncoderFilterConfig::kDefaultWindow,});

    for (uint8_t sample = 0U; sample < filter_window_size_; ++sample)
    {
        updateFilteredSample();
    }
    zero_angle_ = readFilteredAngle();
    return true;
}

uint16_t Kth7823Encoder::readRaw()
{
    uint16_t raw = 0U;
    last_tx_frame_ = 0x0000U;
    encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, false);
    raw = encoder_common::encoder_spi2_rw16(last_tx_frame_);
    encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);
    if (isNonlinearCalValid())
        raw = getCorrectedRaw(raw);
    last_frame_raw_ = raw;
    // 原始raw转为极坐标theta（0-2pi）
    last_frame_theta_ = static_cast<float>(raw) * 2.0F * static_cast<float>(M_PI) / 65536.0F;
    read_count_++;
    if (raw == 0xFFFFU)
    {
        all_ones_count_++;
    }
    else if (raw == 0x0000U)
    {
        all_zeros_count_++;
    }
    return raw;
}


uint16_t Kth7823Encoder::readFilteredRaw()
{
    return filtered_raw_;
}

float Kth7823Encoder::readFilteredAngle()
{
    return static_cast<float>(filtered_raw_) / 65536.0F * 360.0F;
}

/**
 * @brief 【圆周角度矢量滑动平均滤波】 / 整形增量累加（相对角度）+ 基准角度滑动平均滤波（快，无浮点）
 * 问题背景：编码器raw是0‑65535环形角度；0与65535物理上是相邻。
 * 不能直接对uint16原始值算术平均：跨过0点会算到对面半圆，结果错误。
 * 算法原理：极坐标转直角坐标
 *      theta = raw * 2π /65536
 *      X = cos(theta) 单位圆X坐标
 *      Y = sin(theta) 单位圆Y坐标
 *      对X、Y分别做环形滑动窗口平均；
 *      使用atan2(Yavg, Xavg)把平均后的坐标还原回弧度角度；
 *      再换算回uint16_t(0~65535)。
 *
 * @warning 本函数内部会调用readRawAngle()执行SPI读取，有浮点sin/cos/atan2开销
 * @retval false 读到0xFFFF通信错误，滤波器维持旧值；true样本有效完成更新
*/
bool Kth7823Encoder::updateFilteredSample()
{
    const uint16_t raw = readRaw();
    if (raw == 0xFFFFU)
    {
        // SPI通信错误：丢弃样本，滤波器保持上一次有效输出，上层用allOnesCount判断故障
        return false;
    }

    // 窗口大小0：关闭滤波，直接输出原始值
    if (filter_window_size_ == 0U)
    {
        filtered_raw_ = last_frame_raw_;
        return true;
    }

#ifdef USE_POLAR_COORDINATES_WRAP

    //===== 1、原始角度转为单位圆 X(cos), Y(sin) =====
    float theta = last_frame_theta_;
    float s = arm_sin_f32(theta);
    float c = arm_cos_f32(theta);

    if (filter_count_ < filter_window_size_)
    {
        // -------- 窗口填充阶段，还没有填满 --------
        win_s_[filter_count_] = s;
        win_c_[filter_count_] = c;
        sum_s_ += s;
        sum_c_ += c;
        filter_count_++;
        filter_index_ = 0U;
    }
    else
    {
        // -------- 窗口已满：O(1)环形滑动，减去被淘汰旧样本，加入新样本 --------
        sum_s_ -= win_s_[filter_index_];
        sum_c_ -= win_c_[filter_index_];

        win_s_[filter_index_] = s;
        win_c_[filter_index_] = c;

        sum_s_ += s;
        sum_c_ += c;

        filter_index_ = (filter_index_ + 1U) % filter_window_size_;
    }

    //=====2、求X、Y坐标平均值 =====
    uint8_t active_cnt = (filter_count_ < filter_window_size_) ? filter_count_ : filter_window_size_;
    float avg_s = sum_s_ / static_cast<float>(active_cnt);
    float avg_c = sum_c_ / static_cast<float>(active_cnt);

    //=====3、由平均XY坐标还原得到角度弧度 atan2(Y,X) =====
    float result;
    arm_atan2_f32(avg_s, avg_c, &result);
    auto filtered_theta_ = result;
    filtered_raw_ = filtered_theta_ * 65536.0F / 2.0F / static_cast<float>(M_PI);
#else

    // ------------------- 归一化环绕差值：相对于基准angle_base_ -------------------
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(angle_base_);
    while(delta > 32768)  delta -= 65536;
    while(delta < -32768) delta += 65536;

    // ------------------- O(1)环形滑动平均窗口 -------------------
    if(filter_count_ < filter_window_size_)
    {
        //窗口填充阶段
        delta_win_[filter_count_] = delta;
        sum_delta_ += delta;
        filter_count_ ++;
        filter_index_ = 0U;
    }
    else
    {
        //窗口已满，移除旧样本，加入新样本
        sum_delta_ -= delta_win_[filter_index_];
        delta_win_[filter_index_] = delta;
        sum_delta_ += delta;
        filter_index_ = (filter_index_ + 1U) % filter_window_size_;
    }

    //计算delta平均值
    int32_t active_cnt = (filter_count_ < filter_window_size_) ? filter_count_ : filter_window_size_;
    float avg_delta = static_cast<float>(sum_delta_) / static_cast<float>(active_cnt);

    //输出滤波后的uint16角度（环绕处理）
    int32_t out_raw_i32 = static_cast<int32_t>(angle_base_) + static_cast<int32_t>(avg_delta + 0.5F);
    uint16_t out_raw;
    out_raw = static_cast<uint16_t>(out_raw_i32 & 0xFFFFU);

    //==== 关键：定期刷新基准base，防止delta长期累积过大；每若干帧刷新一次 ====
    static uint16_t refresh_cnt = 0;
    refresh_cnt ++;
    if(refresh_cnt >= filter_window_size_)
    {
        refresh_cnt = 0;
        angle_base_ = out_raw;
        sum_delta_ = 0;
        filter_index_ = 0;
        filter_count_ = 0;
        memset(delta_win_,0,sizeof(delta_win_));
    }

    //兼容旧上层接口：把uint16角度转为弧度存入filtered_theta_
    filtered_raw_ = out_raw;
#endif

    return true;
}

void Kth7823Encoder::setFilterWindowSize(uint8_t window_size)
{
    configureFilter({window_size,});
}

void Kth7823Encoder::configureFilter(const EncoderFilterConfig& config)
{
    const uint8_t window_size = config.window_size;
    filter_config_ = config;
    filter_window_size_ = window_size;

#ifdef USE_POLAR_COORDINATES_WRAP
    memset(win_s_, 0, sizeof(win_s_));
    memset(win_c_, 0, sizeof(win_c_));
    sum_s_ = 0.0F;
    sum_c_ = 0.0F;
#endif

    filter_index_ = 0U;
    filter_count_ = 0U;
}

EncoderFilterConfig Kth7823Encoder::filterConfig() const
{
    return filter_config_;
}

uint16_t Kth7823Encoder::lastRawFrame() const
{
    return last_frame_raw_;
}

float Kth7823Encoder::lastFrameTheta() const
{
    return last_frame_theta_;
}

// 极坐标theta转0-360度角度
float Kth7823Encoder::lastFrameAngle() const
{
    return last_frame_theta_ * 180.0F / static_cast<float>(M_PI);
}

uint16_t Kth7823Encoder::lastTxFrame() const
{
    return last_tx_frame_;
}

uint32_t Kth7823Encoder::readCount() const
{
    return read_count_;
}

uint32_t Kth7823Encoder::allOnesCount() const
{
    return all_ones_count_;
}

uint32_t Kth7823Encoder::allZerosCount() const
{
    return all_zeros_count_;
}

uint8_t Kth7823Encoder::misoLevel() const
{
    return gpio_input_data_bit_read(KTH7823_MISO_PORT, KTH7823_MISO_PIN) != 0U ? 1U : 0U;
}

bool Kth7823Encoder::magneticFieldHigh() const
{
    return gpio_input_data_bit_read(KTH7823_MGH_PORT, KTH7823_MGH_PIN) != 0U;
}

bool Kth7823Encoder::magneticFieldLow() const
{
    return gpio_input_data_bit_read(KTH7823_MGL_PORT, KTH7823_MGL_PIN) != 0U;
}

void Kth7823Encoder::setZero(float zero_angle)
{
    zero_angle_ = zero_angle;
}

bool Kth7823Encoder::calibrate(const EncoderCalibrationConfig& config, EncoderCalibrationResult* result)
{
    if (result != nullptr)
    {
        result->offset_ok = false;
        result->direction_ok = false;
        result->noise_ok = false;
        result->walk_ok = false;
        result->offset_correction = 0;
        result->noise_rms = 0.0f;
        result->walk_peak = 0.0f;
    }
    if (!config.enable_offset_calibration && !config.enable_direction_calibration &&
        !config.enable_noise_calibration && !config.enable_walk_calibration)
    {
        return false;
    }
    if (result != nullptr)
    {
        result->offset_ok = true;
        result->direction_ok = true;
        result->noise_ok = true;
        result->walk_ok = true;
        result->offset_correction = config.offset_correction;
    }
    if (config.enable_direction_calibration)
    {
        uint16_t base = readRaw();
        uint16_t next = readRaw();
        if (result != nullptr)
        {
            result->direction_ok = normalize_angle(next, base) < config.sample_count;
        }
    }
    if (config.enable_noise_calibration && result != nullptr)
    {
        result->noise_rms = 0.0f;
        for (uint16_t i = 0U; i < config.sample_count; ++i)
        {
            const uint16_t raw = readRaw();
            const uint16_t diff = normalize_angle(raw, zero_angle_);
            result->noise_rms += static_cast<float>(diff * diff);
        }
        result->noise_rms = sqrt(result->noise_rms / static_cast<float>(config.sample_count));
        result->noise_ok = result->noise_rms <= config.noise_threshold;
    }
    if (config.enable_walk_calibration && result != nullptr)
    {
        result->walk_peak = 0.0f;
        uint16_t previous = readRaw();
        for (uint16_t i = 0U; i < config.sample_count; ++i)
        {
            const uint16_t current = readRaw();
            const uint16_t delta = normalize_angle(current, previous);
            if (delta > result->walk_peak)
            {
                result->walk_peak = static_cast<float>(delta);
            }
            previous = current;
        }
        result->walk_ok = result->walk_peak <= config.walk_threshold;
    }
    return true;
}
