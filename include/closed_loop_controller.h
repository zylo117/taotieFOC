#ifndef CLOSED_LOOP_CONTROLLER_H
#define CLOSED_LOOP_CONTROLLER_H

#include <cstdint>
#include <cmath>

#include "angle_encoder.h"
#include "closed_loop_protocol.h"
#include "external_pulse_follower.h"
#include "fault_monitor.h"
#include "lag_angle.h"
#include "loop_stats.h"
#include "motion_planner.h"
#include "encoder_calibrator.h"
#include "pid_auto_tuner.h"
#include "parameter_router.h"
#include "step_output_channel.h"
#include "stepper_driver.h"
#include "telemetry_reporter.h"
#include "telemetry_scheduler.h"

#ifndef USE_HARD_FLOAT_ACCELERATION
#define USE_HARD_FLOAT_ACCELERATION 1
#endif

// MAX_PULSE_ARR_LEN 已随 ARR 序列生成/输出搬到 step_output_channel.h：
// 那个常量本质上是“一个窗口的脉冲出口能装多少步”，不属于控制器。
// #define CLAMP_FOC_SPEED

#include <cmath>
#include <vector>

#include "core_cm4.h"
#include "pid_controller.h"
// AT32F403A system_core_clock 是内核时钟，例如 240000000UL
// 使用 DWT (Data Watchpoint and Trace) 实现高精度纳秒计时
// CYCCNT 是 32 位寄存器，@250MHz 时约 17.17 秒溢出一次
// 通过跟踪溢出次数，可实现长期高精度时间戳（理论上可运行数千年）
//
// 【实现必须放在单个 .cpp 里】
// 旧写法是 static inline + 三个 static 变量直接写在头文件，每个包含本头文件的
// 翻译单元都会各得一份副本：既白吃 RAM，又会让溢出检测分家——只要某个 TU 的
// 调用间隔超过 17.17 秒，它自己的 current_cc < g_last_dwt_value 判断就失效，
// 时间戳会直接倒退 17 秒，delta_ramp_ns 变成天文数字。
// 现在统一到 closed_loop_controller.cpp 里的单一实现。
uint64_t get_hw_time_ns(void);

// 自适应 PID 的基础参数配置。
// 它描述了在不同速度和加速度区间下，系统对跟随误差的容忍度与增益调整幅度。
struct PidAutoTuneConfig
{
    float min_speed_rps;
    float max_speed_rps;
    float acceleration_rps2;
    float target_follow_error;
    float gain_step;
};

// 采样周期统计结构，用于评估控制器循环运行频率。
struct LoopFrequencyStats
{
    uint32_t position_loop_hz;
    uint32_t velocity_loop_hz;
    uint32_t current_loop_hz;
    uint32_t position_samples;
    uint32_t velocity_samples;
    uint32_t current_samples;
};

// ClosedLoopController 是整个闭环步进控制器的核心。
// 它负责连接：
// - 驱动器底层：实际输出步进信号和电流配置
// - 编码器：读取位置和速度反馈
// - 协议适配器：对外呈现 TMC 兼容调参接口
// 在控制上，它采用“位置环 -> 速度参考 -> 速度环 -> 步进修正”的双环结构。
class ClosedLoopController
{
    // 编码器校准正在逐步迁到 EncoderCalibrator（见 encoder_calibrator.h 的分阶段计划）。
    // 迁移期间两个阶段共存：授权它直接读写 controller 的校准状态与运动参数备份，
    // 避免为几个字段再加一层 setter/getter。全部搬完后这个 friend 声明可删。
    friend class EncoderCalibrator;

public:
    // STEP 脉宽的硬限制（ns）。没写明细分步距时，100ns 已经接近 TMC2209 的
    // 最小脉宽识别能力；再短就会被驱动器整步忽略。上限 20us 是给低速调试留的。
    static constexpr uint64_t kMinStepPulseNs = 100ULL;
    static constexpr uint64_t kMaxStepPulseNs = 20000ULL;

    enum FaultFlags : uint32_t
    {
        FAULT_ENCODER_READ_FAILED = 1UL << 0,
        FAULT_MAGNETIC_FIELD_ALARM = 1UL << 1,
        FAULT_OUTPUT_STOPPED = 1UL << 2,
        // 有脉冲命令、编码器却长时间不动：堵转 / 相线没接好 / 电流太小 / 被按住。
        // 由 updateStallGuard() 置位，停机并失能；清除必须显式写 0x15F=1。
        FAULT_STALL_DETECTED = 1UL << 3
    };

    ClosedLoopController();

    // 绑定驱动器与编码器，并初始化 PID 与零点偏移。
    void init(StepperDriver* driver, AngleEncoder* encoder);

    // 配置故障响应策略：读编码器失败或磁场报警时是否立即停止输出。
    void setFaultPolicy(bool stop_on_encoder_fault, bool stop_on_magnetic_fault);

    // 报告编码器读取失败状态，并在设置为失效时执行停输出动作。
    void reportEncoderFault(bool active);

    // 报告磁场过高/过低状态，并在设置为失效时执行停输出动作。
    void reportMagneticFieldAlarm(bool active);

    // 设置 A/B 相电流监控值，供上位机实时显示。
    void setPhaseCurrentTelemetry(float phase_a_a, float phase_b_a);
    void setCpuUsageTelemetry(uint16_t cpu_usage_centi_percent, uint16_t control_task_usage_centi_percent);

    // 将控制器当前故障状态和采样频率同步到协议扩展寄存器表。
    void syncProtocolTelemetry();

    // 绑定协议适配器，允许外部通过 TMC 风格寄存器访问闭环参数。
    void setProtocol(ClosedLoopDriverProtocol* protocol);

    // 对外写入指定寄存器参数。
    bool writeParameter(uint16_t reg, uint32_t value);

    // 对外读取指定寄存器参数。
    bool readParameter(uint16_t reg, uint32_t* value);

    // 设置目标位置步数。
    void setTargetStep(int32_t target_step);

    // 设定目标角度（0~360°），开启基于编码器反馈的闭环位置控制。
    void setTargetAngleDeg(float angle_deg);
    float getTargetAngleErrorDeg() const;

    // 设置目标速度（转/秒）。
    void setTargetVelocity(float rps);

    // ===== 使能来源 =====
    // 0 = 上位机控制（0x124/0x125，默认）；1 = 外部 EN 引脚（PC13，低电平使能）。
    // 外部模式下 PC13 是唯一权威：固件不再用上位机命令直接写芯片 EN，只跟随引脚电平
    //（故障/用户失能仍然能压住它）。参数写 Flash，重上电保持。
    enum EnableMode : uint32_t
    {
        ENABLE_MODE_HOST = 0U,
        ENABLE_MODE_EXTERNAL_PIN = 1U
    };
    void setEnableMode(uint32_t mode);
    uint32_t getEnableMode() const;
    bool isDriverEnabled() const;
    // 实际 EN 引脚电平（外部模式用；host 模式返回 2 表示"不适用"）
    uint32_t getExternalEnableLevel() const;

    enum MotionMode : uint32_t
    {
        MOTION_MODE_POSITION_FORWARD = 0U,
        MOTION_MODE_POSITION_REVERSE = 1U,
        MOTION_MODE_VELOCITY_FORWARD = 2U,
        MOTION_MODE_VELOCITY_REVERSE = 3U,
        MOTION_MODE_HOME_FORWARD = 4U,
        MOTION_MODE_HOME_REVERSE = 5U,
        MOTION_MODE_ALTERNATING_FORWARD = 6U,
        MOTION_MODE_ALTERNATING_REVERSE = 7U
    };

    void setMotionConfig(float start_rpm, float max_rpm, float accel_rpm_s, uint32_t pulse_count, MotionMode mode);
    void startMotion();
    void stopMotion();
    void rampUpdate(uint64_t now_ns);
    bool isMotionRunning() const;
    float getMotionSpeedRpm() const;
    float getMotionPositionDeg() const;
    uint32_t getWaveformWindowMs() const;
    void setWaveformWindowMs(uint32_t window_ms);

    // 统一设置基本 PID 参数。
    void setPid(float kp, float ki, float kd);

    // 启用/关闭自适应增益调节。
    void enableAdaptivePid(bool enable);

    // 设置自适应 PID 的参数阈值。
    void setAdaptivePidConfig(const PidAutoTuneConfig& config);

    // 根据当前速度、加速度和跟随误差动态修正 PID 参数。
    void updateAdaptivePid(float speed_rps, float acceleration_rps2, float follow_error);

    // 开启/关闭循环频率统计功能。
    void enableLoopStats(bool enable);

    // 获取控制环运行频率统计。
    void getLoopFrequencyStats(LoopFrequencyStats* stats) const;

    // 清空 frequency stats。
    void resetLoopFrequencyStats();

    // 读取当前闭环位置步数。
    int32_t getPositionSteps() const;

    // 最近一次 rampUpdate 周期内采集到的外部 PUL/DIR 有符号脉冲增量
    // （由 pulse_capture_read_delta() 在 rampUpdate 开头消费）
    int32_t getExternalPulseDelta() const;

    // 每圈微步数 = 200 步/圈 x 驱动器实际微步数（默认 200x64 = 12800）。
    // [PUL] 自检行的 deg、上位机“脉冲转角度”都应该用它，不要再写死 51200。
    uint32_t getStepsPerRound() const;

    // ===================== 【外部 PUL/DIR 脉冲跟随】=====================
    // 开启后，PB4/PB5 采集到的脉冲成为“命令位置”（1 脉冲 = 1 微步），由内部 DDA 重发：
    //   闭环补偿关（0x107=0）→ 开环，按限速把积压发完，1:1 转发
    //   闭环补偿开（0x107=1）→ 在上面叠位置/速度环 PID 修正，用编码器保证实际到位
    // 与 0x11E/0x11F 的规划运动互斥（跟随时 startMotion() 直接忽略）。
    void enableExternalPulseFollow(bool enable);
    bool isExternalPulseFollowEnabled() const;
    // 跟随限速（步/s）。0 = 跟随 angle_max_rpm_ 换算出的能力上限（推荐）。
    void setExternalPulseMaxStepRate(uint32_t steps_per_second);
    uint32_t getExternalPulseMaxStepRate() const;
    int32_t getExternalCmdSteps() const;      // 外部累计命令步（有符号）
    int32_t getExternalEmittedSteps() const;  // 已重发出去（有符号）
    uint32_t getExternalBacklogSteps() const; // 当前积压（命令 - 已发）
    uint32_t getExternalPeakBacklogSteps() const;
    uint32_t getExternalDroppedSteps() const; // 因超限没及时发出的累计步数
    // 窗内换向自检：一个 ramp 窗口只对应一个 DMA 序列、一个 DIR，所以窗口里
    // “先正走后反走”的脉冲只能按净和压平（终点位置仍对，中间往返不执行）。
    // 这里只计数不改输出，用来判断实机上到底会不会出现。
    uint32_t getExternalWindowReversals() const;
    void resetExternalPulseFollowCounters();

    // 读取目标位置步数。
    int32_t getTargetSteps() const;
    // 堵转状态：置位后 stopMotion + 失能，必须写 0x15F=1 清除才能再启动。
    bool isStallDetected() const;    void clearStall();
    // 启用/关闭“堵转自动失能”（默认开；整定脚本会在每个试验结束后自己失能）。
    void setStallAutoDisable(bool enable);
    uint32_t getStallEvents() const;
    // 快速停车（0x163=1）：按 0x11C 的加速度减速到 0 后就地收尾，不等走完剩余脉冲。
    // 用来做"急刹冲击"标定：比起直接 stopMotion（瞬间关脉冲）更接近真实急停，
    // 而且整定器能在这段减速里继续采样，量到冲击峰值。
    void requestQuickStop();
    bool isQuickStopActive() const;
    // ===== 运动源绝对互斥（0x16D）+ 急停（0x16E）=====
    // 运动源二选一，且**同一时刻只有一套信号能驱动脉冲**：
    //   EXTERNAL(1，默认)：上电即外部脉冲跟随，规划运动/角度模式/目标角全部拒绝，
    //                      网页对应控件禁用，只读展示照常刷新。
    //   INTERNAL(0)：网页可操作内部规划/角度模式，外部 PUL 输入被忽略。
    enum MotionSource { MOTION_SOURCE_INTERNAL = 0U, MOTION_SOURCE_EXTERNAL = 1U };
    // 切换运动源。切到外部：先停内部运动，再开跟随；切到内部：关跟随（脉冲通道交还规划器）。
    // 【切换从不碰 EN】进入外部模式只是"进模式"：驱动器通不通电流由外部 EN 引脚
    // （PC13）决定（跟随模式下引脚自动是使能权威，上拉悬空=失能，PLC 拉低=使能）；
    // 退出到内部模式按 0x161 策略接管（host：等 0x124；external：仍看引脚）。
    void setMotionSource(uint32_t source);
    uint32_t getMotionSource() const { return motion_source_; }
    // 校准/整定入口的公共前置：外部跟随时自动接管为内部源（原子切换 + [CALIB] 日志）。
    void ensureInternalSourceForCalibration(const char* who);
    // 急停：立即断脉冲 + 失能驱动器（比 requestQuickStop 的减速停车更彻底）。
    void emergencyStop();
    float getEncoderZero() const;
    void setEncoderZero(float zero_angle);
    float getEncoderFilteredAngle() const;
    float getEncoderAngleMilliDegrees() const;
    bool isMagneticFieldHigh() const;
    bool isMagneticFieldLow() const;

    // 返回当前位置与目标之间的跟随误差。
    float getFollowError() const;

    // 读取当前相电流和 loop 频率，用于 USB telemetry 回传。
    float getPhaseCurrentTelemetryA() const;
    float getPhaseCurrentTelemetryB() const;
    uint32_t getPositionLoopHz() const;
    uint32_t getVelocityLoopHz() const;
    uint32_t getCurrentLoopHz() const;
    bool readCalibrationTablePair(uint8_t table, uint16_t pair_index, uint32_t* packed_values) const;
    bool readCalibrationTableChecksum(uint32_t* checksum) const;
    uint16_t lookupCorrectedRaw(uint16_t raw_value) const;
    uint16_t lookupOriginalRaw(uint16_t corrected_value) const;

    void calibrateEncoder();

private:
    // 参数路由需要直接读写控制器的运动状态、PID、编码器、滞后角等成员，
    // 不给它开几十个 public 访问器（那样接口会比现在更难看清）。
    friend class ParameterRouter;
    // 外部跟随同理：积压记账要和 motion_steps_emitted_signed / dma_pending_signed
    // 共用同一份，拆成 getter/setter 只会引入两套状态不同步的风险。
    friend class ExternalPulseFollower;
    friend class MotionPlanner;
    friend class FaultMonitor;
    friend class TelemetryReporter;

    void updateLoopFrequencyStats(uint64_t time_ns);
    // ExternalPulseFollower 尚未抽成独立类之前，Phase 0 先把 rampUpdate 里
    // 三段边界最清晰的逻辑抽成成员函数。后续阶段会把它们整体搬进各自的模块。
    void collectExternalPulseCommand();
    float refreshEncoderFeedback(uint64_t now_ns);
    // 速度估计：固定时间窗上的滑窗差分（见实现处注释，替代每拍 1 LSB 差分 + 0.9/0.1 低通）
    float updateSpeedEstimate(uint64_t now_ns);
    void resetSpeedEstimate();
    // 滞后角的平滑与限速：表值不得随速度噪声/方向抖动直接灌进 PID
    void updateLagAngleFeedforward(float measured_rps, float commanded_rps);
    // 堵转/不转保护：有脉冲在发、轴却长时间不动就停机失能
    void updateStallGuard(uint64_t now_ns);
    // 使能来源：外部 EN 引脚跟随 / 上位机请求（引脚权威模式下"使能"被忽略）
    void updateEnableFromExternalPin();
    // EN 使能权威是否为外部引脚：0x161=external 或 运动源=外部跟随（见 cpp 注释）
    bool enableAuthorityIsExternalPin() const;
    void requestDriverEnable(bool want);
    // 实际生效的滞后角前馈量（毫度，已带方向，正=与跟随误差同号）
    int32_t effectiveLagAngleMilliDeg() const;
    void updateExternalPulseChannel(uint64_t now_ns, float encoder_delta_deg);
    // 沉降收尾：脉冲发完 ≠ 轴到位，等编码器连续若干拍不动或超时
    void updateSettleStage(uint64_t now_ns, float encoder_delta_deg);
    // 速度规划主体：角度模式的角误差环 / 梯形规划状态机 / 校准修饰 / PID 补偿 / DDA 累加。
    // 返回 false 表示本拍已经在内部收尾（已自己调过 syncProtocolTelemetry），调用方直接 return。
    bool updateMotionProfile(uint64_t now_ns, float encoder_delta_deg, uint64_t* delta_ramp_ns_out);
    // 脉冲出口：把本拍累加到的步数铺成 ARR 序列，启动 TMR2/DMA，并完成发射记账
    void emitStepSequence(uint64_t now_ns, uint64_t delta_ramp_ns);
    // 外部跟随的脉冲重发通道，在 rampUpdate 里与 DDA 二选一执行
    void runExternalPulseFollow(uint64_t delta_ramp_ns);
    void updateEncoderCalibration(uint16_t raw);
    // finishEncoderCalibration() / startEncoderCalibrationMove() 已删：
    // 它们已搬成 EncoderCalibrator::finish()/startMove() 的私有方法，
    // 且 controller 侧没有任何调用者（只有校准模块自己用）。
    StepperDriver* driver_;
    AngleEncoder* encoder_;
    ClosedLoopDriverProtocol* protocol_;
    LagAngle lag_angle_;
    uint16_t lag_angle_point_index_;
    uint16_t lag_angle_point_speed_rpm_;
    int32_t lag_angle_point_mdeg_;
    uint16_t lag_angle_query_speed_rpm_;
    bool lag_angle_fit_saved_;
    PidController position_pid_;
    PidController velocity_pid_;
    // 三种模式（角度僚服 / 规划运动 / 外部脉冲跟随）现在共用上面这一组实例与参数：
    // 角度模式改成“梯形轮廓 + 跟踪误差”后，三个位置环的输入量级对齐了（都是跟踪误差，
    // 典型零点几度），所以同一组数值对三者都成立。“速度指令”不再经由单独的 PID 实例。
    PidAutoTuner pid_auto_tuner_;

    float base_position_kp_;
    float base_position_ki_;
    float base_position_kd_;
    float base_velocity_kp_;
    float base_velocity_ki_;
    float base_velocity_kd_;
    bool adaptive_pid_enabled_;
    PidAutoTuneConfig adaptive_config_;

    volatile int32_t target_step_;
    volatile int32_t actual_step_;
    volatile int32_t last_actual_step_;
    volatile int32_t command_step_;
    volatile float follow_error_;
    volatile float measured_velocity_rps_;
    volatile float target_velocity_rps_;
    volatile float target_angle_deg_;
    volatile float closed_loop_angle_error_deg_;
    volatile bool closed_loop_angle_mode_enabled_;
    volatile float angle_position_tolerance_deg_;
    volatile float motion_start_rpm_;
    volatile float motion_max_rpm_;
    volatile float motion_accel_rpm_s_;
    volatile float angle_max_rpm_;
    volatile float angle_accel_rpm_s_;

    // ===== 速度环/角度模式诊断快照（[VLOOP]）=====
    // printf 一律留在任务侧，控制环里只存量：这里把这一拍用到的关键量抄下来，
    // 由遥测任务按周期输出。
    // 两层目的：
    //   1) 抖动归因：把 P / I / D 三个分量分开打，直接看抖的是哪一项（而不是先猜 kp）；
    //   2) 系数透明：把“网页/Flash 口径值”(base_*) 与“此刻真正在算的 kp”并排打出来，
    //      applyGainScales() 这类运行时改增益的行为一旦生效就藏不住。
    struct VLoopDbg
    {
        volatile float err_deg = 0.0f;       // 位置误差（度，带符号）
        volatile float ref_rps = 0.0f;       // 总期望速度 = base + 位置环修正
        volatile float meas_rps = 0.0f;      // 实测速度（encoder_speed_rpm_/60）
        volatile float vel_err_rps = 0.0f;   // 速度误差
        volatile float pos_out_rps = 0.0f;   // 位置环输出
        volatile float vel_p = 0.0f;         // 速度环 P 分量（含角度模式的 ×0.01）
        volatile float vel_i = 0.0f;         // 速度环 I 分量
        volatile float vel_d = 0.0f;         // 速度环 D 分量
        volatile float out_rps = 0.0f;       // 最终交给 emitSteps 的量
        volatile float prof_vel_rps = 0.0f;  // 角度模式轮廓速度（= 速度前馈量）
        volatile float step_speed_s = 0.0f;  // current_step_speed_（微步/s）
        volatile uint32_t stamp = 0U;        // 更新计数：任务侧据此判断闭环是否在跑
    };
    VLoopDbg vloop_;

    // ===== 运动收尾报告（[SETTLE] / 行程对账），延迟到真静止后由 kSlow 档打印 =====
    // updateSettleStage 在 20kHz 控制任务里，一行带 %f 的 printf ≈ 10ms 停摆；
    // settled 路径此时轴确实停了没事，**timeout 路径轴可能还在大幅振荡**，
    // 打印会把停摆算进沉降观察窗口、还把补偿拍饿掉。所以控制环里只存快照，
    // 由 telemetry_reporter 在确认 motion_running_==false 的慢档打印。
    struct SettleReport
    {
        volatile bool pending = false;          // 有待打印的收尾报告（存快照时置位）
        volatile float elapsed_ms = 0.0f;
        volatile bool settled = false;          // true=编码器静默判停；false=超时强制停
        volatile float cmd_deg = 0.0f;
        volatile float enc_deg = 0.0f;
        volatile float follow_deg = 0.0f;
        volatile uint32_t emitted = 0U;         // 行程对账：已发步数
        volatile uint32_t pulse_count = 0U;     // 行程对账：计划步数
    };
    SettleReport settle_report_;

    // ===== 角度模式的梯形轮廓（速度前馈）=====
    // 为什么要有轮廓：纯 P 的角度模式里「速度 ∝ 误差」，越接近目标越慢（指数尾巴），
    // 而且无法“按指定转速转到目标角”。以前的 base_speed 自引用是在偷偷补这个洞，
    // 代价是把速度反馈噪声积成抖动。现在换成正规做法：
    //   轮廓以 0x12F（限速）/0x130（限加速度）向目标推进；
    //   v_ref = 轮廓速度（前馈） + 位置PID（轮廓位置 - 实际位置）（只处理跟踪误差）
    // 前馈负责“走多快”，闭环负责“跟得准”，不再挤在一个 P 上。
    // 轮廓位置不做 0~360 归一：它始终在目标附近，越缝时靠环形误差处理，不会飘远。
    volatile float angle_profile_pos_deg_ = 0.0f;
    volatile float angle_profile_vel_rps_ = 0.0f;
    volatile bool angle_profile_valid_ = false;
    // 一次性打印开关（写 0x155 置位）：遥测慢档打印一次后自动归零。
    volatile bool vloop_debug_once_ = false;
    volatile uint32_t motion_pulse_count_;
    volatile MotionMode motion_mode_;
    volatile bool closed_loop_compensation_enabled_;
    volatile bool motion_running_;
    volatile bool motion_first_run_;
    volatile bool motion_paused_;
    volatile float motion_speed_rpm_;
    volatile float encoder_speed_rpm_;
    // 只给遥测/波形用的转速（20ms 平滑）：控制环用的是上面那个 3ms 的。
    // 分开的原因：20kHz 速度估计在 25Hz 采样下看就是一团毛刺，和"电机在抖"分不清。
    volatile float encoder_speed_display_rpm_ = 0.0f;
    // ===== 速度估计（滑窗差分）=====
    // 旧估计是「相邻两拍角差 / dt」再 0.9/0.1 低通。问题不在低通，而在量化：
    // 编码器 1 LSB = 0.0055°，50us 内 1 LSB 就是 110°/s = 18.3RPM。低通只能压
    // RMS，压不掉这个"台阶"，还把群延迟做到 1ms 量级，速度环永远在追一个既滞后又
    // 台阶化的量 —— 这是"改一点点就抖"的另一半原因。
    // 现在在约 1ms 的固定窗口上做差分：量化按窗口长度缩小（18.3 -> 0.9RPM），
    // 群延迟只有半个窗口（0.5ms）而且是线性的，速度环才可以按它整定。
    static constexpr uint16_t kSpeedWindowTicks = 20U;   // 20 x 50us = 1ms
    float speed_angle_ring_[kSpeedWindowTicks] = {};
    uint32_t speed_time_ring_[kSpeedWindowTicks] = {};
    uint16_t speed_ring_index_ = 0U;
    uint16_t speed_ring_count_ = 0U;
    // 滞后角表用的更慢的速度（滞后角本身是慢变量，不需要 20kHz 更新）
    volatile float lag_angle_speed_rpm_ = 0.0f;
    // 滞后角的一阶平滑 + 变化率限幅后的“实际生效值”（度，已带方向）
    volatile float lag_angle_applied_deg_ = 0.0f;
    // 速度环 D 项用的上一拍“沿运动方向的速度”
    float last_measured_along_rps_ = 0.0f;
    // 角度模式速度环 D 项用的上一拍实测速度（与规划路径分开，切换模式时不会串）
    float last_angle_mode_measured_rps_ = 0.0f;
    volatile float motion_position_deg_;
    volatile float motion_follow_error_deg_;
    volatile float motion_commanded_travel_deg_;
    volatile int64_t motion_encoder_travel_raw_;
    volatile uint16_t motion_encoder_previous_raw_;
    volatile bool motion_encoder_reference_valid_;
    volatile uint64_t motion_last_ramp_time_ns_;
    volatile uint64_t motion_steps_emitted_;
    // 【都用 int64 防长期运行溢出】
    // 原来三个都是 int32：按 60000 步/s（281RPM 上限）算，约 9.9 小时就绕满，
    // 一旦回绕，motion_commanded_travel_deg_ 和 follow_error 会瞬间跳 360 度级的大偏差。
    // 改成 int64 后在这速率下可跑约 48 年。
    // 这些变量只做 += / -= / 比较，不做乘法，所以 64 位运算开销可以忽略。
    volatile int64_t motion_steps_emitted_signed;  // 距离总运动位移脉冲数，有方向；等于是出发至今的带方向累计脉冲数
    // 已经“真的发完”的带方向累计步数，跟随误差必须用它算命令位置。
    // 原因：motion_steps_emitted_signed 在窗口开头就把整窗步数加满了，但脉冲要 50us
    // 才发完（平滑模式），直接相减会让命令位置凭空领先一个窗口 ≈ 几十步 ≈ 0.3°。
    volatile int64_t motion_steps_delivered_signed = 0;
    // 真正交给脉冲出口（TMR2/DMA 或软脉冲）的累计步数，只增不减。
    // 用途：它和 motion_steps_emitted_（记账值）分开存在，才能查出“发的比记的多/少”
    // 这类不对称；现在主要用于校准期间的“命令脉冲 vs 实发脉冲”对账。
    volatile uint32_t step_pulses_emitted_ = 0U;
    // 【校准专用观测】查“实发(num_steps, 取自补偿累加器) != 记账(num_original,
    // 取自原始累加器)”到底出在哪：
    //   calib_bias_windows_ > 0            -> 两个累加器真的分叉了
    //   calib_comp_rps_peak_ > 0           -> 补偿在跑（说明“校准期间关闭环”没关住）
    //   calib_comp_rps_peak_ == 0 且分叉    -> 是余数/钳位路径造成（递减不对称）
    volatile uint32_t calib_bias_windows_ = 0U;
    volatile int32_t calib_bias_sum_ = 0;
    volatile float calib_comp_rps_peak_ = 0.0f;
    // 刚交给 TMR2/DMA、还没到下一拍确认的步数（下一个 rampUpdate 才并入 delivered）
    volatile int64_t motion_dma_pending_signed = 0;
    volatile uint32_t motion_dma_pending_steps_;
    volatile uint32_t motion_leg_pulse_count_;
    // 【必须是 float，不能是 double】Cortex-M4 没有 FP64 硬件，double 的加减/floor 全是
    // 软件模拟（每次 50~150 周期）。这两个累加器在 emitSteps 里每 50us 要参与 4~6 次运算，
    // 按 20kHz 算约 4.8% CPU 纯花在 FP64 模拟上，而且库函数调用导致耗时抖动（50us 硬实时循环的隐患）。
    // 精度校核：累积 640000 步（50 圈）时 float 尾数精度约 0.038 步，远小于 1 步，完全够用。
    volatile float motion_step_accumulator_;
    volatile float compensated_motion_step_accumulator_;
    volatile int8_t motion_direction_;
    volatile bool motion_leg_reversed_;
    volatile bool motion_direction_change_pending_;
    volatile bool motion_step_high_;
    volatile uint64_t step_pulse_width_ns_;
    volatile uint32_t k_step_pulse_ticks;
    volatile uint32_t step_period_us_;
    volatile float encoder_zero_;
    volatile float encoder_filtered_angle_;
    volatile uint16_t encoder_filtered_raw_;
    // 磁编码器滑动平均窗口（样本数），0 表示关闭滤波。
    // 上位机写进来的值先记在 encoder_filter_window_，真正的重配放到低速槽位
    // （ramp_loop_idx % 20000 == 0，20kHz 下约 1s 一次）做，避免运动中途改滤波把跟随误差抖出台阶。
    volatile uint16_t encoder_filter_window_ = EncoderFilterConfig::kDefaultWindow;
    volatile uint16_t encoder_filter_window_active_ = EncoderFilterConfig::kDefaultWindow;

    // 上次「保存参数到 Flash」的结果（0x154 读回用）
    volatile bool param_save_ok_ = false;

    // 【安全锁】用户主动失能后置位，任何 startMotion / setTargetAngleDeg 都拒绝启动，
    // 必须收到显式的 MOTOR_ENABLE 才能清除。
    // 背景：失能只是 setEnable(false)，而 startMotion 里无条件 setEnable(true)，
    // 于是自动测试循环的下一条 0x11F=1 又把电流打开——表现为“已失能但自己转起来”。
    volatile bool user_disabled_ = false;
    volatile bool magnetic_field_high_;
    volatile bool magnetic_field_low_;
    volatile uint32_t last_process_time_us_;
    volatile uint8_t last_step_state_;
    volatile uint8_t last_dir_state_;
    volatile uint8_t last_en_state_;
    volatile int32_t external_pulse_delta_ = 0;  // 外部 PUL/DIR 指令脉冲增量（每 rampUpdate 刷新）

    // ===== 外部脉冲跟随 =====
    // 全部状态（命令位置/已转发/积压/丢步/窗内换向/限速）都在 ExternalPulseFollower 里，
    // 控制器只保留这一个句柄；下面这几个 external_ 开头的成员属于采集侧，不属于跟随模块。
    ExternalPulseFollower external_pulse_follower_{*this};
    MotionPlanner motion_planner_{*this};
    // 编码器校准模块（拆分中，见 encoder_calibrator.h 的六阶段计划）。
    // 目前只有 finish() 与 startMove() 已搬入，主流程还在本类里；
    // 等阶段 5 把四个 public 函数改成转发后就真正接管。
    EncoderCalibrator calibrator_{*this};
    LoopStats loop_stats_;
    // TMR2+DMA 脉冲出口。规划和外部跟随共用这一份（两者在逻辑上互斥），
    // ARR 序列缓冲也只有它这一份。
    StepOutputChannel step_output_;
    // 故障监控（编码器读失败 / 磁场报警）已搬到 FaultMonitor，见 include/fault_monitor.h。
    // '要不要失能驱动、要不要禁止 STEP 输出' 这个安全决策独立成类，审阅时不用再翻两处。
    FaultMonitor fault_monitor_{*this};
    // 遥测上报（syncProtocolTelemetry 的 94 行寄存器字典）已搬到 TelemetryReporter。
    // 控制器只负责“何时刷”，刷什么归它管。
    TelemetryReporter telemetry_reporter_{*this};
    float phase_a_current_a_;
    float phase_b_current_a_;
    volatile uint16_t cpu_usage_centi_percent_;
    volatile uint16_t control_task_usage_centi_percent_;
    uint16_t calibration_lookup_raw_value_;
    uint16_t calibration_lookup_corrected_value_;

    // 频率统计（位置/速度/电流环）已整体搬到 LoopStats，见 include/loop_stats.h。
    // 里面包含 12 个成员和 500ms 窗口平均算法，控制器只保留这个句柄。

    float closed_loop_threshold_angle_ = 1.8f;
    float compensated_rps_ = 0.0f;
    // 实时滞后角（度，正值 = 电机轴相对于命令脉冲的理论滞后量）。
    // 由补偿段每拍更新（lag_angle_.getLagAngleMilliDegrees 按当前转速查模），
    // 遥测通过 0x151 发给网页显示，也用于“波形不计滞后角”的后处理。
    float motion_lag_angle_deg_ = 0.0f;

    // 换向后的“评分屏蔽拍数”。往返运动换向那一刻命令方向翻转、机械还在减速，
    // follow_error 会瞬间拉开几十度（物理必然，不是 PID 差），如果计进评分
    // 会把 RMS 抬到几十度，完全淹没真实跟随性能（实测短行程评分 58.7°、全场景 104.5°）。
    // 换向时设为 kTurnaroundSkipRamps，每拍递减到 0 后恢复采样。
    uint32_t motion_turnaround_skip_ramps_ = 0U;

    // ===== 堵转/不转保护 =====
    // 判据：命令速度足够大（真的在发脉冲）且编码器在 200ms 窗口内的**净位移**很小。
    // 阈值必须高于"静止时编码器的慢漂移"，又远低于最慢的正常转动：
    //   静止噪声峰峰约 26~31 LSB（实测），200ms 净位移不会超过 ~30 LSB；
    //   最低速 8RPM 时 200ms 也走 1750 LSB。取 100 LSB 有 3 倍/17 倍的双向余量。
    // （不能用"每拍变化 < N LSB"：噪声每拍就能超过 2 LSB，计时器会被不停清零，
    //   实测把运行电流置 0 让电机完全不动时守卫依然不动作。）
    static constexpr uint32_t kStallQuietMs = 200U;
    static constexpr int32_t kStallMoveLsb = 100;
    static constexpr float kStallMinCommandRpm = 8.0f;
    // 【运行时可调阈值（0x174/0x175，Flash v6 持久化）】默认 = 上面的编译期口径。
    // 现场常有"低速大惯量、慢响应就被误判堵转"或"要更灵敏保护"的需求，故可调：
    //   stall_quiet_ms_  加大=更宽容（慢系统防误报），减小=更灵敏。合理域 50~3000。
    //   stall_move_lsb_  净位移阈值。必须 > 静止噪声窗口净位移（~100LSB），
    //                    又 < 最低速真实位移（8RPM×窗 ≈1750LSB）。合理域 40~1500。
    // 控制任务每拍只读一次这两 volatile 单字，M4 上原子；写走 USB 任务，无锁安全。
    volatile uint32_t stall_quiet_ms_ = kStallQuietMs;
    volatile uint32_t stall_move_lsb_ = static_cast<uint32_t>(kStallMoveLsb);
    volatile bool stall_detected_ = false;
    volatile uint32_t stall_events_ = 0U;
    volatile bool stall_auto_disable_ = true;
    uint32_t stall_quiet_since_ns_ = 0U;
    uint16_t stall_last_raw_ = 0U;
    bool stall_armed_ = false;

    // ===== 使能来源（0x161）=====
    // 外部 EN 接 PC13，低电平使能。外部模式下这个引脚是唯一权威：
    // 上位机的 0x124/0x125 只改 user_disabled_ 这个"允许位"，不直接写芯片 EN。
    volatile uint32_t enable_mode_ = ENABLE_MODE_HOST;
    // 运动源（0x16D）：默认外部脉冲跟随。见 MotionSource 枚举。
    volatile uint32_t motion_source_ = MOTION_SOURCE_EXTERNAL;
    // 共振速度区间（0x16F/0x170）：巡航目标落进区间就钳到 lo，不在带内驻留。
    // 默认 300~600 RPM（用户实测的剧烈抖动区）。hi=0 或 lo>=hi 关闭该逻辑。
    volatile uint16_t resonance_lo_rpm_ = 300U;
    volatile uint16_t resonance_hi_rpm_ = 600U;
    volatile bool resonance_bypass_ = false;   // 0x171：共振自动测试临时旁路
    volatile bool driver_enabled_ = false;
    volatile uint8_t external_en_level_ = 1U;   // 1=高(失能) 0=低(使能)
    // 【消抖状态】PC13 来自 PLC/接触器，几百 ns~几 µs 的毛刺很常见。逐拍单样本
    // 直接翻 EN，一次毛刺 = 一次几十 µs 失能 = 运动中断流丢步，而且不留记录。
    // 要求连续 kEnDebounceSamples 拍同电平才承认翻转（4 拍 = 200µs）。
    static constexpr uint8_t kEnDebounceSamples = 4U;
    volatile uint8_t external_en_stable_ = 1U;      // 去抖后的稳定电平
    volatile uint8_t external_en_candidate_ = 1U;   // 候选新电平
    volatile uint8_t external_en_change_count_ = 0U;
    volatile uint32_t en_glitch_count_ = 0U;        // 被消抖吸收的毛刺次数（只读寄存器）

    // ===== 快速停车（0x163）=====
    volatile bool quick_stop_requested_ = false;
    volatile bool motion_quick_stop_ = false;

    // ===== 起步屏蔽拍数 =====
    // 规划运动第一拍脉冲频率就从 0 跳到 start_rpm，转子要靠惯性跟上去，
    // 这十几毫秒里"跟随误差"能到 -2°，是惯性暂态不是丢步。实测 1 圈短行程
    // 末端多转 0.55~0.9°，几乎全部来自这段 ~10ms 的 ±0.2rps 补偿尖峰。
    // 这段窗口里把两个环的输入清零（输出也强制 0），等转子跟上再接管。
    static constexpr uint16_t kMotionStartBlankRamps = 400U;   // 400 x 50us = 20ms
    volatile uint16_t motion_start_blank_ramps_ = 0U;

    // 速度环输出的一阶平滑状态（抑制把测速噪声直接灌进脉冲频率）
    float velocity_correction_lpf_ = 0.0f;
    // 位置环用的跟随误差一阶平滑（约 2ms）
    float follow_error_lpf_deg_ = 0.0f;

    // 滞后角前馈信任度（0.0~1.0）。
    // 滞后角只与速度有关是个简化假设：真实滞后角还取决于负载大小，而本方案
    // 无法接入 TMC2209 的电流环，感知不到负载变化。信任度降到 0.5~0.7 可以
    // 让前馈变保守，把偏差交给 PID（代价是响应变慢），避免表值偏高时过度补偿。
    //
    // 【默认 0.5 而不是 1.0】表是在某一种负载下标定的；本次空载验证发现表值
    // 比实际滞后高约 0.6°，前馈 100% 会让末端少走 0.58°（1 圈 360° 里）。
    // 50% 把最坏情况的偏差减半，同时保留"剔除物理滞后"的作用。
    // 装上真实负载、重新标定滞后角表之后，应该把 0x152 设回 100。
    float lag_angle_feedforward_ratio_ = 0.5f;

    // ===== 滞后角模型的两个可调参数（都存 Flash，见 docs/lag_angle_and_pid.md）=====
    // 【为什么不写死】滞后角表只在标定时的负载/转速下准；负载一变，表值和真实值
    // 差多少是未知的，死区必须能跟着调。写死一个 0.7° 只是把"某个负载下的经验值"
    // 固化进代码，换台机器就不对了。
    //   lag_accel_coef_mdeg_per_rpm_s：加减速项 ΔL ≈ c·a（物理上来自 J·a/k_t），
    //       单位"毫度/(RPM/s)"，默认 1 -> 600RPM/s 时约 0.6°。0 = 关闭该项。
    //   lag_deadband_mdeg：恒速段残差死区（毫度）。表越准可以越小；标定不充分就放大。
    volatile uint16_t lag_accel_coef_mdeg_per_rpm_s_ = 0U;
    volatile uint16_t lag_deadband_mdeg_ = 400U;

    // 遥测节拍器：控制环每拍推进一格，syncProtocolTelemetry 按档位选刷哪一堆。
    // 以前直接用 rampUpdate 里的 ramp_loop_idx 取模，而那个计数器是 uint16_t，
    // 65535 回绕后 % 20000 每 3.28 秒只能命中 4 次，平均只有 1.22Hz。
    TelemetryScheduler telemetry_scheduler_;
    TelemetryScheduler::Slot telemetry_slot_ = TelemetryScheduler::Slot::kNone;

    // 参数路由（writeParameter / readParameter 的大表）。
    // 这是控制器的 friend：参数分散在运动状态/PID/编码器/滞后角等多个子模块里，
    // 逐个开 public 访问器会把接口撑得比现在更难看清。
    ParameterRouter parameter_router_{*this};

    uint32_t max_step_per_ramp = 0;

    // ====================== 【补充梯形加减速预计算】======================
    // 梯形加减速预计算参数（startMotion里一次性算出）
    float motion_accel_step_s2_; // 加速度：微步/s²
    float motion_start_step_s_; // 起始速度：微步/s
    float motion_max_step_s_; // 最高速度：微步/s
    uint32_t accel_total_steps_; // 加速段总步数
    uint32_t decel_total_steps_; // 减速段总步数
    uint32_t cruise_total_steps_; // 匀速段步数
    enum MotionRampStage
    {
        RAMP_STAGE_ACCEL,
        RAMP_STAGE_CRUISE,
        RAMP_STAGE_DECEL,
        // 脉冲已经全部交给 TMR2/DMA，但电机轴还没追上（滞后角）。
        // 这个阶段只做两件事：等最后一批脉冲真的发完、等编码器静止，然后才收尾。
        // 否则 stopMotion() 会立刻关掉 TMR2，把尾巴上那几十个脉冲连同对应角度一起丢掉。
        RAMP_STAGE_SETTLE,
        RAMP_STAGE_DONE
    } motion_ramp_stage_; // 当前阶段：加速/匀速/减速/沉降确认/结束
    volatile uint64_t motion_settle_start_ns_ = 0ULL;   // 进入沉降阶段的时刻
    volatile uint32_t motion_settle_stable_ramps_ = 0U;  // 编码器连续的“不动”拍数
    volatile uint16_t motion_settle_last_raw_ = 0U;      // 上一拍的编码器原始角，用来判静止
    float current_step_speed_; // 当前瞬时速度，微步/s
    uint32_t steps_to_decel_; // 剩余多少步必须开始减速
    bool motion_zero_speed_recovery_logged_;

    enum EncoderCalibrationStage : uint8_t
    {
        CALIBRATION_IDLE,
        CALIBRATION_HOMING,
        CALIBRATION_SAMPLING,
        CALIBRATION_MOVING,
        CALIBRATION_SAVING,
        // 【极性鉴别探针】排在 HOMING 之前执行：反装磁钢让角度模式变正反馈，
        // 直接归零会在 180° 对面永远震荡。探针用开环脉冲走 1/4 圈看编码器
        // 行程的符号，反装则整体反转读数，然后才进 HOMING。
        // 编号追加在末尾：遥测里已有的 0~4 阶段含义不变。
        CALIBRATION_POLARITY
    };
    // 【校准状态只剩两类】其余 calibration_* 成员（index_ / forward_raw_[1024] /
    // backward_pass_ / sample_* / home_*）已全部搬进 EncoderCalibrator：
    //   calibration_stage_ 必须留在本类 —— motion_planner.cpp 的 updateTrapezoid 靠它
    //   把"本段校准位移走完"告知校准模块（改调 calibrator_.beginSampling()），
    //   读表接口也用"是否 IDLE"做门禁。
    //   saved_* 也留在本类 —— 它们是校准前后的运动参数备份，由
    //   EncoderCalibrator::start() 写、finish() 读，两边共用一份。
    // 原 2KB 的 calibration_forward_raw_ 删除后，RAM 相应下降。
    volatile EncoderCalibrationStage calibration_stage_;

    // 校准前的运动参数备份（见上面注释）
    float calibration_saved_start_rpm_;
    float calibration_saved_max_rpm_;
    float calibration_saved_accel_rpm_s_;
    uint32_t calibration_saved_pulse_count_;
    uint64_t calibration_saved_pulse_width_ns_;
    MotionMode calibration_saved_motion_mode_;
    bool calibration_saved_compensation_enabled_;
};

#endif
