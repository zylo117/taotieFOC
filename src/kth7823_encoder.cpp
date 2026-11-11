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
    // 记下“查表前”的原始字：下面的 0xFFFF/0x0000 计数和跳变锁存都要靠它分区。
    // 【注意】原来的 all_ones/all_zeros 是在查表之后判的，它们统计的是“校准后的值”，
    // 不是“SPI 返回的字”——转子停在 0° 附近时 corrected==0 是完全正常的，
    // 不能拿它当“无应答”的证据。真正的电气证据要看 raw_word。
    last_spi_word_ = raw;
    // 【磁钢极性修正，见 angle_encoder.h setInvertRaw 注释】在校准查表之前取负：
    // 校准表学的就是反转之后的坐标系（校准探针先判极性、清表、再重学），
    // 滤波/行程/角度从这一层起全部同域；常数偏置由校准归零吸收。
    // last_spi_word_ 保留真实总线字，超时/全0全1 诊断分区不受影响。
    if (invert_raw_)
    {
        raw = static_cast<uint16_t>(0U - raw);
    }
    // 【有效性看总线状态，不看值】0xFFFF 既可能是“没应答”也可能是“真角 359.99°”，
    // 拿值当错误码会同时犯两个错：把合法接缝读数当垃圾丢掉，而真正的超时样本
    // 经过校准表映射后不再是 0xFFFF，反而被收下写进表。
    last_spi_ok_ = !encoder_common::encoder_spi2_last_timeout();
    if (!last_spi_ok_)
    {
        ++spi_timeout_count_;
    }
    if (isNonlinearCalValid())
    {
        raw = getCorrectedRaw(raw);
    }
    last_frame_raw_ = raw;
    read_count_++;
    // 下面两个计数只统计“查表后的值”，当诊断参考用；总线有没有应答看
    // spi_timeout_count_，否则超时会同时冒充两个计数器，三者就没法互相区分了。
    if (last_spi_ok_)
    {
        if (raw == 0xFFFFU)
        {
            all_ones_count_++;
        }
        else if (raw == 0x0000U)
        {
            all_zeros_count_++;
        }
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
    // 兼容旧的单次采样接口：自己读一次 raw 再走滤波（腿板不动，只是转发）
    return updateFilteredSample(readRaw());
}

bool Kth7823Encoder::updateFilteredSample(uint16_t raw)
{
    // 窗口切换请求由控制任务登记，重配必须在这里（采样中断）做：
    // 采样中断随时在改 delta_win_/sum_delta_，在任务里直接重配会把数据撕掉。
    applyPendingFilterWindow();

    // 未校准值不参与滤波，直接跟随本次读取结果：
    // 它只用于控制器的“行程累计”，不需要平滑（噪声零均值，累计不会偏移），
    // 而滤波会引入延迟，反而让行程与脉冲对不齐。

    if (!last_spi_ok_)
    {
        // SPI 超时：丢弃样本，滤波器保持上一次有效输出。
        // 原来这里写的是 raw == 0xFFFF：那会误杀真角 359.99° 的合法样本（接缝节点
        // 的去极值平均因此被系统性拉低），又拦不住真正的超时（查表后值已变）。
        return false;
    }

    // 窗口大小0：关闭滤波，直接输出原始值
    if (filter_window_size_ == 0U)
    {
        filtered_raw_ = last_frame_raw_;
        filtered_angle_deg_ = static_cast<float>(last_frame_raw_) * 360.0F / 65536.0F;
        // 这一拍没有任何跨拍记忆，delta/evicted/span 全传 0：
        // 如果异常在这个设置下仍然出现，就能直接排除滤波器。
        checkJumpLatch(filtered_raw_, raw, 0, 0, 0);
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
    int32_t latch_span = 0;   // 仅供跳变锁存记录，不影响任何计算
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
        latch_span = window_span;
    }
#endif

    //输出滤波后的uint16角度（环绕处理）
    const float out_raw_f = static_cast<float>(angle_base_) + compensated_delta;
    auto out_raw = static_cast<uint16_t>(static_cast<int32_t>(lroundf(out_raw_f)) & 0xFFFF);

    // 【必须在“基准平移”之前锁存】快照要反映的是“产生这个输出的那拍状态”；
    // 平移发生在后面，锁存到的是新基准，会看不到真正把输出拽歪的那个旧状态。
    checkJumpLatch(out_raw, raw, delta, has_evicted ? evicted_delta : 0, latch_span);

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

void Kth7823Encoder::sampleRawFast()
{
    // 只做 SPI 读 + 存环形缓冲，不做非线性校准查表（省 ~1.56us/次）。
    // 由独立定时器（TMR5，13us）调用，对应编码器内部更新间隔。
    last_tx_frame_ = 0x0000U;
    encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, false);
    const uint16_t raw = encoder_common::encoder_spi2_rw16(last_tx_frame_);
    encoder_common::encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);

    const uint32_t next = raw_ring_head_ + 1U;
    if ((next - raw_ring_tail_) > kRawRingSize)
    {
        // 控制环太久没取数（正常不会发生）：丢弃最旧的，只计次数
        ++raw_ring_overflow_;
        ++raw_ring_tail_;
    }
    raw_ring_[raw_ring_head_ % kRawRingSize] = raw;
    raw_ring_head_ = next;

    // 同样按总线状态而不按值记有效性（本函数是 TMR5 高频采样路径，目前未启用）
    last_spi_ok_ = !encoder_common::encoder_spi2_last_timeout();
    if (!last_spi_ok_)
    {
        ++spi_timeout_count_;
    }

    read_count_++;
    if (raw == 0xFFFFU)
    {
        ++all_ones_count_;
    }
    else if (raw == 0x0000U)
    {
        ++all_zeros_count_;
    }
}

uint16_t Kth7823Encoder::takeRawAverage()
{
    // 把上次取数之后累积的所有样本环绕安全地平均。
    //
    // 关键：raw 是 0~65535 的环绕角度，直接算算术平均会在跨零点时算到对面半圈去。
    // 做法和滤波器里的 delta 逻辑一致：以第一个样本为基准，算环绕差值后平均，再加回基准。
    const uint32_t available = raw_ring_head_ - raw_ring_tail_;
    if (available == 0U)
    {
        // 定时器还没采到新样本（首拍或 TMR5 未启动）：返回上一次的有效值
        return last_frame_raw_;
    }

    const uint16_t count = static_cast<uint16_t>(available > kRawRingSize ? kRawRingSize : available);
    const uint32_t first_index = (raw_ring_head_ - count) % kRawRingSize;
    const int32_t base = static_cast<int32_t>(raw_ring_[first_index]);

    int32_t sum_delta = 0;
    for (uint16_t i = 0U; i < count; ++i)
    {
        const int32_t sample = static_cast<int32_t>(raw_ring_[(first_index + i) % kRawRingSize]);
        int32_t delta = sample - base;
        while (delta > 32768)
        {
            delta -= 65536;
        }
        while (delta < -32768)
        {
            delta += 65536;
        }
        sum_delta += delta;
    }

    const auto averaged = static_cast<uint16_t>(
        (base + sum_delta / static_cast<int32_t>(count)) & 0xFFFF);

    // 消费掉：对齐到当前写指针（溢出的部分已在 sampleRawFast 里计过数）
    raw_ring_tail_ = raw_ring_head_;

    // 平均后统一做一次非线性校准（原来每读一次都查表，现在一拍只查一次）
    // 注意：averaged 是**未校准**的原始平均值，正好用于控制器的行程累计：
    // 行程是硬件真实量，不能掺校准表的 INL 修正（修正量积分不闭合，
    // 50 圈会累积出 9 度）。
    // 给跳变锁存续存“查表前”的值和本次实际用的样本数：这两个是分区的关键证据。
    last_avg_raw_ = averaged;
    last_avg_count_ = count;
    const uint16_t corrected = isNonlinearCalValid() ? getCorrectedRaw(averaged) : averaged;
    last_frame_raw_ = corrected;
    return corrected;
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

uint32_t Kth7823Encoder::spiTimeoutCount() const
{
    return spi_timeout_count_;
}

bool Kth7823Encoder::lastReadValid() const
{
    return last_spi_ok_;
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

// ===========================================================================
// 跳变锁存：只比较“本拍输出”与“上一拍输出”。
// 【不在中断里打印】printf 300 字节 @115200 约 26ms，会掉 500 个 50us 控制拍；
// 这里只做十几条赋值和一次比较，微秒量级。由 Telemetry_task 取快照。
// ===========================================================================
void Kth7823Encoder::checkJumpLatch(uint16_t out_raw, uint16_t in_corrected,
                                    int32_t delta, int32_t evicted, int32_t span)
{
    int32_t jump = 0;
    // 第二个独立触发条件：原始字与查表结果的环形差。
    // 正常 INL 修正量只有 ±0.35°（±64 LSB），超过 512 LSB（2.8°）就是表把值改飞了。
    // 用环形差是为了在接缝处不误报：raw=0x0002 被表映到 0xFFFE 是正常的 -4 LSB。
    int32_t spread = static_cast<int32_t>(last_spi_word_) - static_cast<int32_t>(in_corrected);
    while (spread > 32768)
    {
        spread -= 65536;
    }
    while (spread < -32768)
    {
        spread += 65536;
    }
    const int32_t abs_spread = (spread < 0) ? -spread : spread;
    if (jump_prev_valid_)
    {
        jump = static_cast<int32_t>(out_raw) - static_cast<int32_t>(jump_prev_out_);
        while (jump > 32768)
        {
            jump -= 65536;
        }
        while (jump < -32768)
        {
            jump += 65536;
        }
        const int32_t abs_jump = (jump < 0) ? -jump : jump;
        // 【只用输出跳变触发；spread 仅作为现场记录，不再单独触发】
        // 上一版把 |raw - corrected| > 512 LSB (2.8°) 也当独立触发条件，但实测那张
        // 校准表的**正当**修正量本来就有 4~7°（漂移曲线：整圈上升 ~4.5° 叠加
        // 180° 周期波纹）—— 阈值比表的正当修正量还小，于是 rsn=2 常态化误报，
        // 把真正的 rsn=1 淹掉了。表质量该由建表时的 [CAL] 两道检查负责；
        // 这里只报“相邻拍真的跳了”，spread 仍写入快照，供跳变发生时分区用。
        if (abs_jump > kJumpLatchLsb)
        {
            // 1 = 只是跳变；2 = 跳变同时表在该点改了 >2.8°（局部残差，值得怀疑）。
            const uint16_t reason = (abs_spread > kTableSpreadLsb) ? 2U : 1U;
            // 计数必须是“真实发生次数”：不管旧快照有没有被取走都要加，
            // 否则事件密集时会漏计，就估不出发生频率了。
            jump_events_++;
            if (!jump_latched_)
            {
                // 旧快照还没人取走就不覆盖（不丢失现场）。
                jump_.seq = read_count_;
                jump_.events = jump_events_;
                jump_.avg_raw = last_avg_raw_;
                jump_.consumed = last_avg_count_;
                jump_.raw_word = last_spi_word_;
                jump_.reason = reason;
                jump_.in_corrected = in_corrected;
                // base / sum_delta 只存在于“增量累加”这条路径（angle_encoder.h 顶部
                // 可以切到极坐标 XY 滑动平均）。两边都编译得过，免得那个备选
                // 配置一打开就成编译错误。
#ifndef USE_POLAR_COORDINATES_WRAP
                jump_.base = angle_base_;
                jump_.sum_delta = sum_delta_;
#else
                jump_.base = 0U;
                jump_.sum_delta = 0;
#endif
                jump_.delta = delta;
                jump_.evicted = evicted;
                jump_.span = span;
                jump_.win = filter_window_size_;
                jump_.cnt = filter_count_;
                jump_.idx = filter_index_;
                jump_.prev_out = jump_prev_out_;
                jump_.out = out_raw;
                jump_.jump = jump;
                jump_.all_ones = all_ones_count_;
                jump_.all_zeros = all_zeros_count_;
                jump_.ring_overflow = raw_ring_overflow_;
                jump_latched_ = true;
            }
        }
    }
    else
    {
        jump_prev_valid_ = true;
    }
    jump_prev_out_ = out_raw;
}

bool Kth7823Encoder::takeJumpSnapshot(JumpSnapshot* out)
{
    if (out == nullptr || !jump_latched_)
    {
        return false;
    }
    // 拷贝期间可能又被写一次：用 seq 复查，不一致就这轮不报（下轮还有）。
    // 【必须绕过优化】jump_ 不是 volatile，直接写 jump_.seq 会让编译器把两次读
    // 合并成一次（或缓存在寄存器里），这个防撕裂检查就成了摆设。
    // 所以第二次读走 volatile 别名，并用 __DMB() 禁止重排。
    const uint32_t seq_before = jump_.seq;
    *out = jump_;
    __DMB();
    const uint32_t seq_after = *reinterpret_cast<volatile uint32_t*>(&jump_.seq);
    jump_latched_ = false;
    if (seq_after != seq_before)
    {
        // 快照在拷贝过程中被写花过：宁可不报，也不给一个拼出来的假现场。
        return false;
    }
    return true;
}
