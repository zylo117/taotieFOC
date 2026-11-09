#ifndef ENCODER_CALIBRATOR_H
#define ENCODER_CALIBRATOR_H

#include <stdint.h>
#include "angle_encoder.h"   // 需要 CALIBRATION_TABLE_SIZE / CALIBRATION_STEPS

class ClosedLoopController;

// ===========================================================================
// 磁编码器双程校准（正转存储 → 反转平均 → 写表）
//
// 从 ClosedLoopController 拆出来的第一块（controller 原本 1900+ 行）。
// 和 MotionPlanner 一样用 owner 引用访问控制器状态，不复制任何共享数据。
//
// 流程：
//   HOMING   角度模式把转子拉到 0 度，等编码器读数稳定 100 拍
//   MOVING   每段走 steps_per_round/CALIBRATION_TABLE_SIZE 步（256 点、12800 细分下整 50 微步）
//   SAMPLING 停 50ms 等滞后角稳定，再采 202 个样本去极值平均
//   正向趟只存不写表；反过来反向趟把两趟平均后写入
//
// 【为什么要正反转平均】滞后角让转子正转时落后、反转时超前，
// 同一个机械位置两趟的编码器读数会一低一高，平均后正好把这块系统偏差
// 抵消（实测 1200RPM 时约 1.5 度）。代价是临时数组（点数 × uint16，256 点仅 512 字节）
// 与一倍时间；边界点（最后一个 index）没有反向采样值，直接用正向值写表。
// ===========================================================================
class EncoderCalibrator
{
public:
    explicit EncoderCalibrator(ClosedLoopController& owner);

    // 启动校准（对应原 calibrateEncoder）。
    // 会先关掉外部跟随、保证 angle_* 限幅非零、清空旧表、切到角度模式回零。
    void start();

    // 每拍推进（对应原 updateEncoderCalibration），在 TMR4 控制循环里调用。
    // raw 是最新的编码器原始值（未查表），0xFFFF 表示本次读数无效。
    void update(uint16_t raw);

    // 当前是否空闲（可接受新的校准请求）。真正生效（阶段 5 接线）后可替代
    // 各处直接读 controller->calibration_stage_ 的判断。
    bool isIdle() const;

    // 当前阶段，供遥测/调试查询。返回值与 ClosedLoopController::EncoderCalibrationStage 一致。
    uint8_t stage() const;

    // 由 MotionPlanner 在一小段校准位移走完时调用：切到 SAMPLING 并重置采样计数。
    // 采样计数已搬进本类，planner 不能再直接写，否则两个模块各持一份。
    void beginSampling();

private:
    void finish(bool save_table);
    bool startMove(uint16_t index);

    // ===== 采样节拍（集中在这里，避免“样本数”与“除数”两处不一致）=====
    // 一个点要连采 kSamplesNeeded 个才取平均（去最大最小后除 kForMean），
    // 每拍一个样本（50us）；kSettleTicks 是移动后的等待时间。
    // 时长代价：每个点≈ kSettleTicks + kSamplesNeeded 个拍，× 2047 个点。
    static constexpr uint32_t kSettleTicks = 1600U;      // 80ms（原 50ms）
    static constexpr uint16_t kSamplesNeeded = 302U;     // 原 202，同位置多采样降抖动
    static constexpr uint16_t kSamplesForMean = kSamplesNeeded - 2U;
    static constexpr uint32_t kMaxAttempts = 2000U;      // 取不满样本则判失败

    // ===== 表的完整性校验（不是“兼容性堆砌”，而是让“校准做坏了”不可能静默通过）=====
    // 每格应推进 CALIBRATION_STEPS/CALIBRATION_TABLE_SIZE；单点噪声就有几 LSB，
    // INL 与滞后角残差也在十几 LSB 量级，所以步进偏差给 4 倍理想格宽的空间
    // （按点数自动缩放，改点数不用改阈值）。
    // 【理想格宽必须四舍五入，不能取整商】点数不一定要整除 65536：
    // 800 点时真值是 81.92，取整商得 81，于是每格都算少 0.92 LSB，
    // 799 格累积就是 ≈ 730 LSB —— 会被 closure 误报成“刻度错”。
    // 阈值用取整后的格宽无关紧要（0.92 LSB 远小于 4 倍容差），
    // 但理论推进量 kExpectLsb 必须用 64 位精确算。
    static constexpr int32_t kIdealStep = static_cast<int32_t>(
        (CALIBRATION_STEPS + CALIBRATION_TABLE_SIZE / 2U) / CALIBRATION_TABLE_SIZE);
    static constexpr int32_t kStepToleranceLsb = 4 * kIdealStep;
    // 命令扫完 N-1 格时，传感器理论上应推进的量（不要求 N 整除 65536）
    static constexpr int32_t kExpectLsb = static_cast<int32_t>(
        (static_cast<int64_t>(CALIBRATION_STEPS) *
         static_cast<int64_t>(CALIBRATION_TABLE_SIZE - 1U)) /
        static_cast<int64_t>(CALIBRATION_TABLE_SIZE));
    static constexpr int32_t kClosureToleranceLsb = 182;

    // 记录每一次写表：检查与上一格的步进是否接近 64，并存下首尾两格算闭合
    void noteWrite(uint16_t index, uint16_t value);
    bool tablePlausible() const;      // 打印诊断 + 给出结论（由 finish 决定是否保存）

    // ===== 正向趟 / 反向趟的逐节点差值统计 =====
    // d(i) = 反向测到的角 - 正向测到的角。它的用途是分辨两件完全不同的事：
    //   d 的常数分量     = 静态滞后角（两趟符号相反，已被双程平均正确抵掉）
    //   d 随 i 的斜率     = 逐段累积、与方向相关的超差（平均抵不掉，会造假 closure）
    // 两者在 closure 一个数字上混在一起，只能靠这个分布分开 —— 而两份数据本来就在手上。
    void noteForwardBackward(uint16_t index, uint16_t fwd, uint16_t bwd);
    void reportForwardBackward() const;
    int64_t fb_sum_d_ = 0;      // Σd
    int64_t fb_sum_i_ = 0;      // Σi
    int64_t fb_sum_id_ = 0;     // Σ i·d
    int64_t fb_sum_ii_ = 0;     // Σ i²
    int32_t fb_max_abs_d_ = 0;  // 最大的 |d|（带符号）
    uint16_t fb_max_abs_index_ = 0U;
    uint16_t fb_pairs_ = 0U;    // 参与统计的节点对数

    // ===== 命令脉冲 vs 实发脉冲对账 =====
    // 整圈推进多出 +3.0%（实测相去约 390 微步）。补偿环已经实验排除、
    // “方向相关的逐段超差”也被 d(i) 斜率排除，剩下的分歧只可能是：
    // 要么真的多发了脉冲，要么脉冲数对但转子被负载拽着逐段爬。
    // 实发计数在 controller->step_pulses_emitted_（只统计交给脉冲出口的步数），
    // 这里只记初值与命令量，finish() 里一行带出结果。
    uint32_t pulse_counter_base_ = 0U;   // start() 时的实发脉冲累计值
    uint32_t commanded_pulses_ = 0U;     // 本场校准命令过的脉冲总数（两趟合计）
    uint16_t commanded_segments_ = 0U;   // 段数

    // ===== 逐段对账：把“每段多发 1.4”摊开成分布 =====
    // 总量只能告诉我们“多发了”，分布才能分清成因：
    //   几乎每段都 +1/+2   -> 发射本身在系统性多发（窗口铺满擦边、guard 泄入下一窗等）
    //   少数段 +6、+7      -> 个别段被卡住/重发，要看那些段落在哪些节点（尤其是绕回点）
    //   出现负值          -> 有段没走完就被下一个 startMove 抢走（remaining 钳位不对称）
    // 段的“结束时刻”定在**下一个** startMove 入口：上一段之后还有 1600 拍的静止采样，
    // 那一窗的脉冲肯定已全部发完，所以此时取差是可信的。
    void closeSegment();
    void reportSegments() const;
    bool seg_active_ = false;
    uint32_t seg_pulse_base_ = 0U;    // 本段开始时的 step_pulses_emitted_
    uint32_t seg_commanded_ = 0U;     // 本段命令的步数
    uint32_t seg_ticks_ = 0U;         // 本段花了多少个控制拍（发射窗数的上界）
    uint64_t seg_ticks_total_ = 0ULL;
    uint32_t seg_index_ = 0U;         // 已经关闭了几段
    int32_t seg_diff_min_ = 0;
    int32_t seg_diff_max_ = 0;
    uint32_t seg_diff_max_at_ = 0U;   // 最大超差发生在第几段
    uint32_t seg_over_ = 0U;          // diff > 0 的段数
    uint32_t seg_under_ = 0U;         // diff < 0 的段数
    // 【命令 → 记账 → 实发】三分解：上一批数据说明段是“跑到 51 步才停”，
    // 而不是“停了以后多吐脉冲”，所以必须把中间那一环（motion_steps_emitted_）也纳入：
    //   seg_counted_diff_sum_ = Σ(记账 - 命令)   —— 不为 0 说明段结束条件本身跑多了/少了
    //   seg_emit_vs_count_sum_ = Σ(实发 - 记账) —— 不为 0 说明发射与记账不等（钳位不对称）
    int32_t seg_counted_diff_sum_ = 0;
    int32_t seg_emit_vs_count_sum_ = 0;
    uint32_t seg_diff_sum_ = 0U;      // 正超差的总和（算平均用）
    // 直方图桶：≤-2 / -1 / 0 / +1 / +2 / +3..5 / ≥+6
    uint32_t seg_hist_[7] = {};
    bool write_prev_valid_ = false;
    uint16_t write_prev_value_ = 0U;
    uint16_t write_at_first_ = 0U;    // index 0 写进去的值
    uint16_t write_at_last_ = 0U;     // index 1023 写进去的值
    uint16_t mono_bad_ = 0U;          // 步进超偏的格子数
    int32_t mono_worst_dev_ = 0;      // 最差的 |步进-64|
    uint16_t mono_worst_index_ = 0U;

    ClosedLoopController& owner_;

    // 【阶段状态不复制在这里】唯一真源是 owner_.calibration_stage_：
    //   motion_planner.cpp 的 updateTrapezoid 会在一小段步数走完时把它从 MOVING 改成 SAMPLING，
    //   readCalibrationTablePair / readCalibrationTableChecksum 也用"是否 IDLE"做门禁。
    // 本类再存一份 stage_ 就会有两份状态，接线后必然出现"一边改了另一边不知道"。
    // 因此本类只持有校准专用、外部不会碰的状态（下面这些）。
    volatile uint16_t index_ = 0U;

    // 正向趟的采样值（反向趟取平均时用）。长度必须与主表一致，直接派生自宏。
    static constexpr uint16_t kPoints = CALIBRATION_TABLE_SIZE;
    uint16_t forward_raw_[kPoints] = {};
    bool backward_pass_ = false;

    volatile uint16_t sample_count_ = 0U;
    volatile uint16_t sample_attempts_ = 0U;
    volatile int32_t sample_sum_ = 0;
    volatile int32_t sample_min_ = 0;
    volatile int32_t sample_max_ = 0;
    volatile uint16_t sample_anchor_ = 0U;

    // HOMING 阶段：等转子稳定；SAMPLING 阶段复用 wait_ticks 做“采样前停 50ms”
    volatile uint32_t home_wait_ticks_ = 0U;
    volatile uint16_t home_stable_count_ = 0U;
    volatile float home_previous_angle_ = 0.0f;

    // 校准前的运动参数备份，结束时恢复
    float saved_start_rpm_ = 0.0f;
    float saved_max_rpm_ = 0.0f;
    float saved_accel_rpm_s_ = 0.0f;
    uint32_t saved_pulse_count_ = 0U;
    uint8_t saved_motion_mode_ = 0U;
    uint64_t saved_pulse_width_ns_ = 0U;
    bool saved_compensation_enabled_ = false;
};

#endif // ENCODER_CALIBRATOR_H
