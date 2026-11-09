#ifndef CLOSED_LOOP_PROTOCOL_H
#define CLOSED_LOOP_PROTOCOL_H

#include <stdint.h>

#include "stepper_driver.h"

// 这是闭环控制器对外的“协议桥接配置”，用于统一描述
// TMC 风格参数访问所需的最小工作状态。
// 通过这个结构体，控制器和底层驱动之间不直接绑定具体芯片寄存器，
// 而是通过统一协议语义进行参数映射与转换。
struct ClosedLoopDriverProtocolConfig
{
    uint16_t microsteps;
    uint16_t interpolation_microsteps;
    float run_current_a;
    float hold_current_a;
    bool enable_hold_current;
    bool silent_mode;
    float sense_resistor_ohm;
    bool use_single_wire_uart;
    gpio_type* uart_port;
    uint16_t uart_pin;
    uint32_t uart_baudrate;
};

// TMC2209 的官方寄存器表。
// 这里保留的是与 Klipper / TMC 实际寄存器地址相一致的核心位置，
// 用于在协议层把外部访问请求映射到闭环控制器的参数和底层驱动配置。
enum Tmc2209OfficialRegister
{
    TMC2209_REG_GCONF = 0x00U,
    TMC2209_REG_GSTAT = 0x01U,
    TMC2209_REG_IOIN = 0x06U,
    TMC2209_REG_DRV_CONF = 0x0AU,
    TMC2209_REG_IHOLD_IRUN = 0x10U,
    TMC2209_REG_TPOWERDOWN = 0x11U,
    TMC2209_REG_TSTEP = 0x12U,
    TMC2209_REG_TPWMTHRS = 0x13U,
    TMC2209_REG_TCOOLTHRS = 0x14U,
    TMC2209_REG_THIGH = 0x15U,
    TMC2209_REG_MSCNT = 0x6AU,
    TMC2209_REG_MSCURACT = 0x6BU,
    TMC2209_REG_CHOPCONF = 0x6CU,
    TMC2209_REG_COOLCONF = 0x6DU,
    TMC2209_REG_DCCTRL = 0x6EU,
    TMC2209_REG_DRV_STATUS = 0x6FU,
    TMC2209_REG_PWMCONF = 0x70U,
    TMC2209_REG_PWMSCALE = 0x71U,
    TMC2209_REG_ENCM_CTRL = 0x72U
};

// 自定义扩展寄存器区：0x100 起，专门承载闭环控制器特有参数。
// 这样不会污染 TMC 官方寄存器空间，且后续扩展更安全。
enum Tmc2209CustomExtensionRegister
{
    TMC2209_EXT_PARAM_MICROSTEPS = 0x100U,
    TMC2209_EXT_PARAM_INTERPOLATION = 0x101U,
    TMC2209_EXT_PARAM_RUN_CURRENT_A = 0x102U,
    TMC2209_EXT_PARAM_HOLD_CURRENT_A = 0x103U,
    TMC2209_EXT_PARAM_HOLD_ENABLED = 0x104U,
    TMC2209_EXT_PARAM_SILENT_MODE = 0x105U,
    TMC2209_EXT_PARAM_SENSE_RESISTOR = 0x106U,
    TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE = 0x107U,
    TMC2209_EXT_PARAM_FAULT_STATUS = 0x108U,
    TMC2209_EXT_PARAM_ENCODER_FAULT = 0x109U,
    TMC2209_EXT_PARAM_MAGNETIC_FAULT = 0x10AU,
    TMC2209_EXT_PARAM_OUTPUT_STOP = 0x10BU,
    TMC2209_EXT_PARAM_AB_CURRENT_A = 0x10CU,
    TMC2209_EXT_PARAM_AB_CURRENT_B = 0x10DU,
    TMC2209_EXT_PARAM_POS_LOOP_HZ = 0x10EU,
    TMC2209_EXT_PARAM_VEL_LOOP_HZ = 0x10FU,
    TMC2209_EXT_PARAM_CUR_LOOP_HZ = 0x110U,
    TMC2209_EXT_PARAM_LAST_FAULT = 0x111U,
    TMC2209_EXT_PARAM_TARGET_POSITION = 0x112U,
    TMC2209_EXT_PARAM_ACTUAL_POSITION = 0x113U,
    TMC2209_EXT_PARAM_FOLLOW_ERROR = 0x114U, TMC2209_EXT_PARAM_ENCODER_RAW = 0x115U,
    TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG = 0x116U, TMC2209_EXT_PARAM_MAGNETIC_HIGH = 0x117U,
    TMC2209_EXT_PARAM_MAGNETIC_LOW = 0x118U, TMC2209_EXT_PARAM_ENCODER_ZERO = 0x119U,
    TMC2209_EXT_PARAM_START_RPM = 0x11AU, TMC2209_EXT_PARAM_MAX_RPM = 0x11BU, TMC2209_EXT_PARAM_ACCEL_RPM_S = 0x11CU,
    TMC2209_EXT_PARAM_PULSE_COUNT = 0x11DU, TMC2209_EXT_PARAM_MOTION_MODE = 0x11EU,
    TMC2209_EXT_PARAM_MOTION_COMMAND = 0x11FU, TMC2209_EXT_PARAM_SPEED_RPM = 0x120U,

    // 实际是跟随误差
    TMC2209_EXT_PARAM_POSITION_DEG = 0x121U,

    TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS = 0x123U, TMC2209_EXT_PARAM_MOTOR_ENABLE = 0x124U,
    TMC2209_EXT_PARAM_MOTOR_DISABLE = 0x125U, TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS = 0x126U,
    TMC2209_EXT_PARAM_TARGET_ANGLE_DEG = 0x128U,
    TMC2209_EXT_PARAM_POSITION_KP = 0x129U,
    TMC2209_EXT_PARAM_POSITION_KI = 0x12AU,
    TMC2209_EXT_PARAM_POSITION_KD = 0x12BU,
    TMC2209_EXT_PARAM_VELOCITY_KP = 0x12CU,
    TMC2209_EXT_PARAM_VELOCITY_KI = 0x12DU,
    TMC2209_EXT_PARAM_VELOCITY_KD = 0x12EU,
    TMC2209_EXT_PARAM_ANGLE_MAX_RPM = 0x12FU,
    TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S = 0x130U,
    TMC2209_EXT_PARAM_PID_TUNE_CONTROL = 0x131U,
    TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG = 0x132U,
    TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT = 0x133U,
    TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS = 0x134U,
    TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM = 0x135U,
    TMC2209_EXT_PARAM_CALIBRATE_ENCODER = 0x136U,
    TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT = 0x137U,
    TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT = 0x138U,
    TMC2209_EXT_PARAM_CALIBRATION_TABLE_DUMP = 0x139U,
    TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED = 0x13AU,
    TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW = 0x13BU,
    TMC2209_EXT_PARAM_LAG_ANGLE_UPLOAD_COUNT = 0x13CU,
    TMC2209_EXT_PARAM_LAG_ANGLE_POINT_INDEX = 0x13DU,
    TMC2209_EXT_PARAM_LAG_ANGLE_POINT_SPEED_RPM = 0x13EU,
    TMC2209_EXT_PARAM_LAG_ANGLE_POINT_MDEG = 0x13FU,
    TMC2209_EXT_PARAM_LAG_ANGLE_POINT_COMMIT = 0x140U,
    TMC2209_EXT_PARAM_LAG_ANGLE_FIT = 0x141U,
    TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_SPEED_RPM = 0x142U,
    TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_MDEG = 0x143U,
    TMC2209_EXT_PARAM_LAG_ANGLE_POINT_COUNT = 0x144U,
    TMC2209_EXT_PARAM_LAG_ANGLE_FIT_SLOPE = 0x145U,
    TMC2209_EXT_PARAM_LAG_ANGLE_FIT_INTERCEPT = 0x146U,
    TMC2209_EXT_PARAM_LAG_ANGLE_FIT_EXPONENT = 0x147U,

    // 磁编码器滑动平均窗口（采样数 0~256，0 = 关闭滤波），上位机可调
    TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW = 0x148U,

    // ===================== 【外部 PUL/DIR 脉冲跟随】=====================
    // 使能后 PB5(PUL)/PB4(DIR) 上的脉冲变成位置命令（1 脉冲 = 1 微步），
    // 由 TMR2/DMA 重发到驱动器。与 0x11E/0x11F 的规划运动互斥；0x14B~0x14E 为只读遥测。
    TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE = 0x149U,   // R/W：0=关，1=开
    TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S = 0x14AU,  // R/W：跟随限速（步/s），0=按角度模式 RPM 上限自动换算
    TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS = 0x14BU,      // RO：外部累计命令步（有符号）
    TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS = 0x14CU,  // RO：已 1:1 转发步数（有符号）
    TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG = 0x14DU,        // RO：当前积压步数
    TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED = 0x14EU,        // RO：累计丢步（只累加超出部分的量）
    TMC2209_EXT_PARAM_EXT_FOLLOW_RESET = 0x14FU,          // WO：写 1 清零命令/已发/积压/丢步计数
    // RO：窗内发生过换向的窗口数。一个 50us 窗口只能发一个方向，这种窗口里
    // “先正走后反走”的脉冲被净和压平（终点位置仍对，中间往返不执行）。
    // 只做统计，不影响输出；持续增长就说明外部信号频率高到一窗内会换向。
    TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS = 0x150U,

    // RO：实时滞后角（mdeg，有符号）。
    // 含义：按当前转速从 lag_angle_ 拟合模型算出的“理论滞后角”。
    // 用途：网页显示 + “波形不计滞后角”勾选（勾选后用 follow_error + lag_angle 绘制）。
    // 注意：未标定滞后角表时恒为 0（isFitted() == false 不启用前馈）。
    TMC2209_EXT_PARAM_LAG_ANGLE_MDEG = 0x151U,

    // R/W：滞后角前馈信任度（百分数，0~100，默认 100）。
    // 滞后角不只与速度有关，还取决于实际负载；本方案感知不到负载变化
    // （无法接入 TMC2209 电流环），表值偏高时前馈过度会让 PID 反向修正。
    // 降到 50~70 让前馈变保守，把偏差交给 PID（代价是响应变慢）。
    TMC2209_EXT_PARAM_LAG_ANGLE_FF_RATIO = 0x152U,

    // R/W：旁路磁编码器非线性校准表（已移除，保留地址不重用）
    TMC2209_EXT_PARAM_CALIBRATION_BYPASS_RESERVED = 0x153U,

    // W：保存当前参数到 Flash（写 1 触发）；R：上次保存结果（1=成功）。
    // 保存内容：PID 六项 + 角度模式限速/加速度。写入会 stall Flash 总线几毫秒，
    // 只能在静止、用户显式点击时用。
    TMC2209_EXT_PARAM_SAVE_PARAMS = 0x154U,
    // R/W：一次性打印 [VLOOP] 速度环/增益诊断快照（写 1 = 打一次，打完自动清零）。
    // 为什么是触发式而不是周期打印：三行浮点 printf 约 270 字节 @115200 ≈ 23ms，
    // 放在 1Hz 慢档会周期性挤占任务，实测影响闭环手感 —— 诊断不该改变被诊断的对象。
    TMC2209_EXT_PARAM_VLOOP_DEBUG_PRINT = 0x155U,
    // R/W：角度模式（串级）的独立增益，口径与 0x129~0x12E 相同（实际增益 × 1000）。
    // 为什么要分开：0x129~0x12E 服务规划运动/外部跟随（梯形是主驱动，PID 只做小修正），
    // 而角度模式里 PID 输出就是全部速度指令 —— 同一个数字在两条路径上含义不同，
    // 共用一组参数就是让两种回路互相“偷走”对方的整定结果。
    // 【已废弃】0x156~0x15B：角度模式独立增益。
    // 角度模式改成“梯形轮廓 + 跟踪误差”后，它与规划运动/外部跟随的输入量级对齐了，
    // 于是三种模式合并为一组参数（0x129~0x12E）。这些编号不再接受写入，
    // 保留枚举只为旧脚本/旧前端不至于误写到别的寄存器上。
    TMC2209_EXT_PARAM_ANGLE_POSITION_KP_DEPRECATED = 0x156U,
    TMC2209_EXT_PARAM_ANGLE_POSITION_KI_DEPRECATED = 0x157U,
    TMC2209_EXT_PARAM_ANGLE_POSITION_KD_DEPRECATED = 0x158U,
    TMC2209_EXT_PARAM_ANGLE_VELOCITY_KP_DEPRECATED = 0x159U,
    TMC2209_EXT_PARAM_ANGLE_VELOCITY_KI_DEPRECATED = 0x15AU,
    TMC2209_EXT_PARAM_ANGLE_VELOCITY_KD_DEPRECATED = 0x15BU,
    // R/W：位置环超前校正系数 α，单位 0.001（填 1000 = α=1.0 = 不启用）。
    // H(s) = (1+ατp·s)/(1+τp·s)，补回微分低通吃掉的相位裕度；与 kd 配合使用，
    // 允许在不过分放大噪声的前提下提高阻尼。合法区间 1.0~5.0。
    // 不写 Flash（与 0x12F/0x130 同属“本次运行设定”），重启后回到 1.0。
    TMC2209_EXT_PARAM_POSITION_LEAD_ALPHA = 0x15CU,

    // R/W：诊断用——旁路磁编码器非线性校准表（0=正常查表，1=直接返回原始值）。
    // 用途：做“查表 / 不查表”的 A/B 对比，判断某个角度误差是不是校准表引入的。

    // ===== 堵转/不转保护（自动整定与无人值守测试的安全依据）=====
    // 0x15D RO：1 = 已判定堵转（已停机 + 失能）；用 0x15F=1 清除后才能再启动
    TMC2209_EXT_PARAM_STALL_DETECTED = 0x15DU,
    // 0x15E RO：累计堵转次数（掉电清零）
    TMC2209_EXT_PARAM_STALL_EVENTS = 0x15EU,
    // 0x15F WO：写 1 清除堵转锁存（同时解除 output_stopped_）
    TMC2209_EXT_PARAM_STALL_CLEAR = 0x15FU,
    // 0x160 RO：本次试验的峰值 |滞后角补偿后误差|（毫度），配合 0x132 的评分看：
    // 评分小而峰值大 = 偶发冲击/掉步；两个都大 = 整段都在抖。
    TMC2209_EXT_PARAM_PID_TUNE_PEAK_MDEG = 0x160U,

    // ===== 使能来源 / 急刹 / 补偿观测（2026-11 新增）=====
    // 0x161 R/W：使能来源。0 = 上位机（0x124/0x125，默认）；1 = 外部 EN 引脚（PC13，低有效）。
    // 外部模式下引脚是唯一权威：上位机只能撤销许可（失能），不能强行使能。
    // 写 Flash，重上电保持。
    TMC2209_EXT_PARAM_ENABLE_MODE = 0x161U,
    // 0x162 RO：驱动器此刻是否使能（1=已使能）。上位机/外部两种来源都适用。
    TMC2209_EXT_PARAM_ENABLE_STATE = 0x162U,
    // 0x163 WO：写 1 = 快速停车（按 0x11C 加速度减速到 0 后就地收尾）。
    // 用于"急刹冲击"标定：比 stopMotion 的瞬间关脉冲更接近真实急停，且整定器能采样。
    TMC2209_EXT_PARAM_QUICK_STOP = 0x163U,
    // 0x164 RO：外部 EN 引脚电平（0=低/使能，1=高/失能）；上位机模式返回 2（不适用）。
    TMC2209_EXT_PARAM_EXT_EN_LEVEL = 0x164U,
    // 0x165 RO：当前闭环补偿速度（毫转/秒，有符号，沿运动方向）。
    // 抖动归因用：恒速段这个值应该很小且平稳；它乱跳说明速度环在放大测速噪声。
    TMC2209_EXT_PARAM_COMP_RPS_MILLI = 0x165U,

    // ===== 滞后角模型参数（都可调、都存 Flash）=====
    // 0x166 R/W：加减速项系数，单位"毫度/(RPM/s)"。物理上 ΔL ≈ (J/k)·a：
    //   加速时转子要多落后一点，而这个量只跟加速度成正比。默认 1（600RPM/s 约 0.6°）。
    //   写 0 = 关闭加减速项（退回只按转速查表）。
    TMC2209_EXT_PARAM_LAG_ACCEL_COEF = 0x166U,
    // 0x167 R/W：恒速段残差死区（毫度）。它应该和"滞后角表有多准"匹配：
    //   表是带载标定的、负载又变了，就给它大一点；重新标定后再收小。
    TMC2209_EXT_PARAM_LAG_DEADBAND_MDEG = 0x167U,
};

// ClosedLoopDriverProtocol 是闭环控制器的协议抽象接口。
// 它负责把上层“读取参数 / 写入参数”的请求，翻译成真正的底层驱动命令。
// 这使得控制器只关心闭环逻辑，而不直接依赖具体驱动芯片实现。
class ClosedLoopDriverProtocol
{
public:
    virtual ~ClosedLoopDriverProtocol() = default;

    // 初始化协议栈，并绑定底层驱动器。
    virtual bool init() = 0;

    // 配置协议所需的运行参数。
    virtual bool configure(const ClosedLoopDriverProtocolConfig& config) = 0;

    // 返回当前协议配置。
    virtual const ClosedLoopDriverProtocolConfig& config() const = 0;

    // 写入一个协议寄存器，例如 TMC 官方寄存器或扩展参数。
    virtual bool writeRegister(uint16_t reg, uint32_t value) = 0;

    // 读取一个协议寄存器的当前值。
    virtual bool readRegister(uint16_t reg, uint32_t* value) = 0;

    // 设置闭环扩展参数（例如微步数、当前、静音模式等）。
    virtual bool setCustomParameter(uint16_t id, uint32_t value) = 0;

    // 获取闭环扩展参数。
    virtual bool getCustomParameter(uint16_t id, uint32_t* value) const = 0;

    // 把实际驱动器对象挂接到协议适配器中。
    virtual void attachDriver(StepperDriver* driver) = 0;
};

// Tmc2209ProtocolAdapter 是协议适配器的具体实现。
// 它模拟了“外部主机按 TMC2209 协议访问驱动器”的行为，
// 但实际只是把这些访问重定向到闭环控制逻辑和统一驱动后端。
class Tmc2209ProtocolAdapter : public ClosedLoopDriverProtocol
{
public:
    Tmc2209ProtocolAdapter();
    bool init() override;
    bool configure(const ClosedLoopDriverProtocolConfig& config) override;
    const ClosedLoopDriverProtocolConfig& config() const override;
    bool writeRegister(uint16_t reg, uint32_t value) override;
    bool readRegister(uint16_t reg, uint32_t* value) override;
    bool setCustomParameter(uint16_t id, uint32_t value) override;
    bool getCustomParameter(uint16_t id, uint32_t* value) const override;
    void attachDriver(StepperDriver* driver) override;

    // 将当前协议配置同步到底层驱动器。
    void syncToDriver();

private:
    static bool isOfficialTmcRegister(uint8_t reg);
    static bool isCustomExtensionRegister(uint16_t id);
    static uint32_t buildRequestWord(uint8_t reg, uint32_t value);
    static uint32_t decodeResponseWord(uint8_t reg, uint32_t value);
    bool applyOfficialRegister(uint8_t reg, uint32_t value);
    bool readOfficialRegister(uint8_t reg, uint32_t* value) const;

    ClosedLoopDriverProtocolConfig config_;
    // 扩展参数表长度：覆盖 0x100 ~ 0x167，新增寄存器时同步这个常量
    static constexpr uint16_t kCustomParameterCount = 0x167U - 0x100U + 1U;
    uint32_t custom_parameters_[kCustomParameterCount];
    StepperDriver* driver_;
};

#endif
