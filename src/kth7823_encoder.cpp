#include "kth7823_encoder.h"
#include <math.h>
#include <string.h>
#include <dsp/fast_math_functions.h>

#include "at32f403a_407_board.h"

// 群延迟前馈补偿：滑动平均窗口的等相位点在窗口中心，天然滞后 (N-1)/2 个样本。
// 不补的话，转速越高、窗口越大，编码器读数越是“旧”的，跟随误差里就凭空多一项
// ω·(N-1)/2·Ts。注意 Ts 是采样周期 32us（不是旧注释里误以为的 2us），
// 所以 N=25 @1200rpm 时滞后是 (25-1)/2 * 32us = 384us ≈ 2.76°，
// 而不是按 2us 算出来的 0.17° —— 比设计预期大 16 倍，这正是“平滑窗口越大误差越大”的主因。
// 代价是用首尾样本差当斜率，会把噪声抬回去一点（静止时不抬，因为首尾样本一样）。
#ifndef ENCODER_FILTER_LAG_COMPENSATION
#define ENCODER_FILTER_LAG_COMPENSATION 1
#endif

namespace
{
    // pending_filter_window_ 的“空”标记：窗口允许取 0（关闭滤波），所以不能用 0 当空值。
    constexpr uint16_t kNoPendingFilterWindow = 0xFFFFU;

    // 注：这里曾有个 normalize_angle(raw, zero)，全项目零调用，而且实现是错的（
    // 返回的是绝对差 |raw-zero|，不是环绕差）。需要环形角度归一化时用
    // closed_loop_controller.cpp 里的 normalize_signed_delta_deg()（while 折返）。
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
      pending_filter_window_(kNoPendingFilterWindow),
      filter_config_{EncoderFilterConfig::kDefaultWindow},
      last_frame_raw_(0U), filtered_raw_(0U), filtered_angle_deg_(0.0F), last_tx_frame_(0U),
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
    // 【SPI 上限已探明：DIV_16 = 7.8MHz 是这颗芯片能用到的极限】
    //   DIV_32 = 3.9MHz  -> 4.10us/次（最早，稳定）
    //   DIV_16 = 7.8MHz   -> 2.05us/次（实测无错，当前）
    //   DIV_8  = 15.6MHz  -> 1.02us/次（实测角度错乱、电机乱转，已回退）
    //
    // DIV_8 失败的原因：15.6MHz 下单个 SCLK 周期只有 64ns，
    // MISO 必须在 32ns 内完成电平建立。KTH7823 的 MISO 驱动能力撑不住，
    // 主控采样到的就是上一位的残留 -> 位错 -> 角度随机跳变。
    // 官方参数表写的 1MHz 虽然保守（7.8MHz 已稳定），但 15.6MHz 确实是过了线。
    //
    // 不要再往 DIV_8 / DIV_4 试：这不是余量不足，是物理极限。
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_16;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(KTH7823_SPI, &spi_init_struct);
    spi_enable(KTH7823_SPI, TRUE);

    configureFilter({EncoderFilterConfig::kDefaultWindow,});

    for (uint16_t sample = 0U; sample < filter_window_size_; ++sample)
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
    {
        raw = getCorrectedRaw(raw);
    }
    last_frame_raw_ = raw;
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
    // 用亚 LSB 的浮点输出：65536 刻度的 1 LSB ≈ 0.0055°，直接返回整数会让角度
    // 一跳就是 0.0055°，控制环和上位机读数都被这个台阶卡死。
    return filtered_angle_deg_;
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
    // 窗口切换请求由控制任务登记，重配必须在这里（采样中断）做：
    // 31.25kHz 采样中断随时在改 delta_win_/sum_delta_，在任务里直接重配会把数据撕掉。
    applyPendingFilterWindow();

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
        filtered_angle_deg_ = static_cast<float>(last_frame_raw_) * 360.0F / 65536.0F;
        return true;
    }

#ifdef USE_POLAR_COORDINATES_WRAP

    //===== 1、原始角度转为单位圆 X(cos), Y(sin) =====
    // 原始raw转为极坐标theta（0-2pi）
    float theta = static_cast<float>(last_frame_raw_) * 2.0F * static_cast<float>(M_PI) / 65536.0F;
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
    if (filtered_theta_ < 0.0F)
    {
        filtered_theta_ += 2.0F * static_cast<float>(M_PI);
    }
    filtered_raw_ = filtered_theta_ * 65536.0F / 2.0F / static_cast<float>(M_PI);
    filtered_angle_deg_ = filtered_theta_ * 360.0F / 2.0F / static_cast<float>(M_PI);
#else

    // ------------------- 归一化环绕差值：相对于基准angle_base_ -------------------
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(angle_base_);
    while(delta > 32768)  delta -= 65536;
    while(delta < -32768) delta += 65536;

    // 窗口为空（刚初始化或刚改完窗口）：基准直接对齐当前样本，避免第一个块带着
    // 上一轮的旧基准算出一堆大差值。
    if (filter_count_ == 0U)
    {
        angle_base_ = raw;
        sum_delta_ = 0;
        delta = 0;
    }

    const int32_t window_size = static_cast<int32_t>(filter_window_size_);

    // ------------------- O(1)环形滑动平均窗口 -------------------
    bool has_evicted = false;
    int32_t evicted_delta = 0;
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
        evicted_delta = delta_win_[filter_index_];
        has_evicted = true;
        sum_delta_ -= evicted_delta;
        delta_win_[filter_index_] = delta;
        sum_delta_ += delta;
        filter_index_ = static_cast<uint16_t>((filter_index_ + 1U) % filter_window_size_);
    }

    //计算delta平均值
    const int32_t active_cnt = (filter_count_ < filter_window_size_) ? static_cast<int32_t>(filter_count_) : window_size;
    const float avg_delta = static_cast<float>(sum_delta_) / static_cast<float>(active_cnt);

    // 群延迟前馈：滑动平均的等相位点在窗口中心，输出天然滞后 (N-1)/2 个样本。
    // 用首尾样本差估斜率（匀速时是精确的），把这段延迟提前补回去，
    // 这样跟随误差不再随“窗口大小 × 转速”凭空变大。
    float compensated_delta = avg_delta;
#if ENCODER_FILTER_LAG_COMPENSATION
    // 首尾样本跨度：静止或微动时这个值几乎全是量化噪声（0/±1 LSB），
    // 此时滞后本身就不到 1 LSB，补上去等于白送抖动，直接不补。
    const int32_t window_span = delta - evicted_delta;
    if (has_evicted && window_size > 1 && (window_span > 1 || window_span < -1))
    {
        // 首尾两个样本相差 window_size-1 个采样间隔，斜率用这个间隔算才是真斜率；
        // 乘 (window_size-1)/2 正好把“重心滞后”补回到最新样本的时刻（匀速时零滞后）。
        const float slope_per_sample = static_cast<float>(window_span) / static_cast<float>(window_size - 1);
        compensated_delta += 0.5F * static_cast<float>(window_size - 1) * slope_per_sample;
    }
#endif

    //输出滤波后的uint16角度（环绕处理）
    const float out_raw_f = static_cast<float>(angle_base_) + compensated_delta;
    auto out_raw = static_cast<uint16_t>(static_cast<int32_t>(lroundf(out_raw_f)) & 0xFFFF);

    //==== 基准平移：只在差值漂得离基准太远时才整体平移一次 ====
    // 以前是每 N 帧就把基准重置成输出值，等于把窗口清空，群延迟因此每 N 帧
    // 从 0 锯到 (N-1)/2 再锯回 0——窗口越大锯幅越大，这就是“平滑窗口越大
    // 误差越大”的真正原因。现在窗口永远是满的，延迟恒定（且已被上面补掉）。
    if (filter_count_ == filter_window_size_ &&
        (sum_delta_ > window_size * 16384 || sum_delta_ < -(window_size * 16384)))
    {
        const int32_t shift = sum_delta_ / window_size;
        for (uint16_t i = 0U; i < filter_window_size_; ++i)
        {
            delta_win_[i] -= shift;
        }
        sum_delta_ -= shift * window_size;
        angle_base_ = static_cast<uint16_t>((static_cast<int32_t>(angle_base_) + shift) & 0xFFFF);
    }

    //兼容旧上层接口：把uint16角度转为弧度存入filtered_theta_
    filtered_raw_ = out_raw;
    // 亚 LSB 输出：把环绕后的浮点原始值换算成角度，保留小数部分
    float out_raw_wrapped = out_raw_f;
    while (out_raw_wrapped < 0.0F) out_raw_wrapped += 65536.0F;
    while (out_raw_wrapped >= 65536.0F) out_raw_wrapped -= 65536.0F;
    filtered_angle_deg_ = out_raw_wrapped * 360.0F / 65536.0F;
#endif

    return true;
}

void Kth7823Encoder::setFilterWindowSize(uint16_t window_size)
{
    // 也走“登记请求”的路径，保持单一入口，避免任务侧直接改中断正在用的滤波状态
    requestFilterWindow(window_size);
}

void Kth7823Encoder::requestFilterWindow(uint16_t window_size)
{
    if (window_size > EncoderFilterConfig::kMaxWindow)
    {
        window_size = EncoderFilterConfig::kMaxWindow;
    }
    pending_filter_window_ = window_size;
}

void Kth7823Encoder::applyPendingFilterWindow()
{
    const uint16_t requested = pending_filter_window_;
    if (requested == kNoPendingFilterWindow)
    {
        return;
    }
    pending_filter_window_ = kNoPendingFilterWindow;
    if (requested == filter_window_size_)
    {
        return;
    }

    EncoderFilterConfig config = filter_config_;
    config.window_size = requested;
    configureFilter(config);
}

void Kth7823Encoder::configureFilter(const EncoderFilterConfig& config)
{
    uint16_t window_size = config.window_size;
    if (window_size > EncoderFilterConfig::kMaxWindow)
    {
        window_size = EncoderFilterConfig::kMaxWindow;
    }

    filter_config_ = config;
    filter_config_.window_size = window_size;
    filter_window_size_ = window_size;

#ifdef USE_POLAR_COORDINATES_WRAP
    memset(win_s_, 0, sizeof(win_s_));
    memset(win_c_, 0, sizeof(win_c_));
    sum_s_ = 0.0F;
    sum_c_ = 0.0F;
#else
    // 窗口一换，旧窗口里的差值全部作废，不清零会把上一轮的 sum 混进新窗口
    sum_delta_ = 0;
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
