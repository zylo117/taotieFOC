#include "parameter_router.h"

#include "closed_loop_controller.h"
#include "param_store.h"
#include "usb_cdc_protocol.h"   // 单例计数（0x169~0x16C 只读）

#include <cstdio>

// 说明：下面这些分支里的成员访问都显式带上 owner_. 前缀。
// 这不是啰嗦 —— 这张表里既有“只改一个标量”的简单项，也有“改参数顺手清 PID 积分”
// 和“直接触发一次运动”的复合动作，去掉前缀以后很难一眼看出哪些是控制器状态、
// 哪些只是本地的临时量。

namespace
{
    // PID 增益的合法区间（单位 = **实际增益**，网页输入框填的就是这个数）。
    //
    // 【这个表必须和前端 input 的 max 一致】历史上这里按“×1000 的口径”写了
    // 30000/3000，可是 gain 已经被除以 1000 了，于是真实上限只有 30/3：用户填 60
    // 会被静默钳成 30、填 20 的 Ki 被钳成 3 —— “我填的值不是给 PID 的值”这一类
    // 玄学全部来自这里。现在按每个环的物理量级重划：
    //   位置环输入是“圈”（1° = 1/360 圈），输出 rps：60 表示 1° 误差给 0.167rps；
    //   速度环输入输出都是 rps，增益是无量纲：0.3 已经能把 1RPM 误差变成 0.3RPM 修正。
    float maxPidGain(uint16_t reg)
    {
        switch (reg)
        {
        case TMC2209_EXT_PARAM_POSITION_KP:
        case TMC2209_EXT_PARAM_POSITION_KI:
            return 1000.0f;
        case TMC2209_EXT_PARAM_POSITION_KD:
            return 100.0f;
        case TMC2209_EXT_PARAM_VELOCITY_KP:
        case TMC2209_EXT_PARAM_VELOCITY_KI:
            return 10.0f;
        case TMC2209_EXT_PARAM_VELOCITY_KD:
            return 1.0f;
        default:
            return 1.0f;
        }
    }

    float clampPidGain(uint16_t reg, uint32_t raw_value)
    {
        const float gain = static_cast<float>(raw_value) / 1000.0f;
        const float max_gain = maxPidGain(reg);
        return gain > max_gain ? max_gain : gain;
    }

    // 主机不可写的寄存器：纯遥测/计数器/一次性 WO。
    // 只影响主机写入路径（router.write）；固件内部注入遥测走 protocol_->setCustomParameter，
    // 不经过这里，所以把地址列进来不会挡住刷新。
    // 判据：0x120 转速、0x137/0x138 CPU、0x116 角度、0x121 误差这类“只读遥测”，
    // 以及本批新增的 0x168~0x16C 计数器。（WO 触发器如 0x163/0x16E 不在此列 ——
    // 它们必须可写，只是读回无意义。）
    bool isHostReadOnlyRegister(uint16_t reg)
    {
        switch (reg)
        {
        case TMC2209_EXT_PARAM_FAULT_STATUS:
        case TMC2209_EXT_PARAM_ENCODER_FAULT:
        case TMC2209_EXT_PARAM_MAGNETIC_FAULT:
        case TMC2209_EXT_PARAM_OUTPUT_STOP:
        case TMC2209_EXT_PARAM_POS_LOOP_HZ:
        case TMC2209_EXT_PARAM_VEL_LOOP_HZ:
        case TMC2209_EXT_PARAM_CUR_LOOP_HZ:
        case TMC2209_EXT_PARAM_LAST_FAULT:
        case TMC2209_EXT_PARAM_ACTUAL_POSITION:
        case TMC2209_EXT_PARAM_FOLLOW_ERROR:
        case TMC2209_EXT_PARAM_ENCODER_RAW:
        case TMC2209_EXT_PARAM_ENCODER_ANGLE_MDEG:
        case TMC2209_EXT_PARAM_MAGNETIC_HIGH:
        case TMC2209_EXT_PARAM_MAGNETIC_LOW:
        case TMC2209_EXT_PARAM_SPEED_RPM:
        case TMC2209_EXT_PARAM_POSITION_DEG:
        case TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG:
        case TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT:
        case TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS:
        case TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM:
        case TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT:
        case TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT:
        case TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_MDEG:
        case TMC2209_EXT_PARAM_LAG_ANGLE_POINT_COUNT:
        case TMC2209_EXT_PARAM_LAG_ANGLE_FIT_SLOPE:
        case TMC2209_EXT_PARAM_LAG_ANGLE_FIT_INTERCEPT:
        case TMC2209_EXT_PARAM_LAG_ANGLE_FIT_EXPONENT:
        case TMC2209_EXT_PARAM_LAG_ANGLE_MDEG:
        case TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS:
        case TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS:
        case TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG:
        case TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED:
        case TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS:
        case TMC2209_EXT_PARAM_ENABLE_STATE:
        case TMC2209_EXT_PARAM_EXT_EN_LEVEL:
        case TMC2209_EXT_PARAM_COMP_RPS_MILLI:
        case TMC2209_EXT_PARAM_STALL_DETECTED:
        case TMC2209_EXT_PARAM_STALL_EVENTS:
        case TMC2209_EXT_PARAM_PID_TUNE_PEAK_MDEG:
        case TMC2209_EXT_PARAM_EN_GLITCH_COUNT:
        case TMC2209_EXT_PARAM_RX_OVERFLOW_COUNT:
        case TMC2209_EXT_PARAM_RX_CRC_FAIL_COUNT:
        case TMC2209_EXT_PARAM_TX_DROP_COUNT:
        case TMC2209_EXT_PARAM_WRITE_DENIED_COUNT:
            return true;
        default:
            return false;
        }
    }
}

ParameterRouter::ParameterRouter(ClosedLoopController& owner) : owner_(owner)
{
}

bool ParameterRouter::write(uint16_t reg, uint32_t value)
{
    // 【只读寄存器门】以前主机写任何地址都回"成功 ACK + 你写的值"，而
    // custom_parameters_ 表被脏值污染（下次 kSlow 刷新前读回的就是脏的）——
    // 协议层不该骗人。现在显式列出主机不可写的寄存器：拒绝并让 bridge 把 ACK
    // 换成"当前真实值"（reg 能配对，waitAck 不挂死；比较数值即知被拒）。
    // 注意遥测注入走 protocol_->setCustomParameter 直写表，**不经过这里**，不受影响。
    if (isHostReadOnlyRegister(reg))
    {
        UsbCdcProtocolBridge* bridge = UsbCdcProtocolBridge::instance();
        if (bridge != nullptr)
        {
            bridge->recordWriteDenied();
        }
        return false;
    }

    // 【Flash 写入的拒绝条件】flash_write 走"读整扇区→擦→整扇区回写"，
    // AT32F403A 单 bank：擦除+编程期间所有取指停摆 **几十毫秒**，
    // 20kHz 控制环整段消失，而 TMR2 还在按最后一窗的周期发脉冲 —— 必丢步。
    // 所以：只要有任何"电机在动/即将动"的形态存在，就不允许写 Flash。
    // 外部跟随和校准以前漏在门外（只查了规划运动与角度模式）。
    auto motion_danger = [&]() -> const char* {
        if (owner_.motion_running_) return "planned motion";
        if (owner_.closed_loop_angle_mode_enabled_) return "angle mode";
        if (owner_.external_pulse_follower_.isEnabled()) return "external pulse follow";
        if (owner_.calibration_stage_ != ClosedLoopController::CALIBRATION_IDLE) return "calibration";
        return nullptr;
    };

    if (reg == TMC2209_EXT_PARAM_VLOOP_DEBUG_PRINT)
    {
        // 一次性触发：由遥测慢档打印一条 [VLOOP] 快照后自动清零。
        // 不做周期打印：三行浮点≈ 23ms 的阻塞会干扰被诊断的对象本身。
        owner_.vloop_debug_once_ = value != 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SAVE_PARAMS)
    {
        // 【写入会 stall Flash 总线几十毫秒】运动中绝对不能做，否则会丢脉冲。
        const char* danger = motion_danger();
        if (danger != nullptr)
        {
            printf("[PARAM] save rejected: %s is active\r\n", danger);
            owner_.param_save_ok_ = false;
            return true;
        }
        if (value == 0U)
        {
            return true;   // 写 0 不做任何事
        }

        PersistedMotorParams params = {};
        // 【这些字段以前根本没填，一直是 0】结构体是 `= {}` 初始化的，
        // 而保存代码只写了 PID：于是 Flash 里 microsteps/step_pulse_ns 永远是 0，
        // 重启后驱动器退回编译期默认微步（32），而脉冲数/滞后角表都是 64 口径 ——
        // 表现为“重新上电后行程正好差一倍”。现在按现场真实值一起存。
        params.microsteps = owner_.driver_ != nullptr ? owner_.driver_->config().microsteps : 0U;
        params.step_pulse_ns = static_cast<uint16_t>(
            owner_.step_pulse_width_ns_ > 65535U ? 65535U : owner_.step_pulse_width_ns_);
        params.run_current_ma = 0U;
        params.hold_current_ma = 0U;
        // PID 用上位机口径（×1000 的整数），与 param_store.h 注释一致。
        // 【必须存 base_*，不能存 live 的 position_pid_.kp】applyGainScales() 会在运行时
        // 把 kp/ki/kd 乘上速度/加速/误差相关的系数（planner 自适应路径），如果存 live 值，
        // 整定/自适应之后点“保存参数”就会把被放大过的增益永久写进 Flash：下次开机 base
        // 就是那个错值，而网页回显读的是 base —— 于是“我填的值并不是给 PID 的值”，
        // 而且下次再也不会恢复。这就是系数“莫名失效”的真实通道。
        params.position_kp = static_cast<uint32_t>(owner_.base_position_kp_ * 1000.0f + 0.5f);
        params.position_ki = static_cast<uint32_t>(owner_.base_position_ki_ * 1000.0f + 0.5f);
        params.position_kd = static_cast<uint32_t>(owner_.base_position_kd_ * 1000.0f + 0.5f);
        params.velocity_kp = static_cast<uint32_t>(owner_.base_velocity_kp_ * 1000.0f + 0.5f);
        params.velocity_ki = static_cast<uint32_t>(owner_.base_velocity_ki_ * 1000.0f + 0.5f);
        params.velocity_kd = static_cast<uint32_t>(owner_.base_velocity_kd_ * 1000.0f + 0.5f);
        // 三模式已合并为一组参数，上面这几个角度专用字段不再使用，仅保留布局兼容（写 0）；
        // 结构体大小未变，所以 param_store 的 version 仍然是 2，不需要再迁移。
        params.angle_position_kp = 0U;
        params.angle_position_ki = 0U;
        params.angle_position_kd = 0U;
        params.angle_velocity_kp = 0U;
        params.angle_velocity_ki = 0U;
        params.angle_velocity_kd = 0U;
        // 使能来源（0=上位机 / 1=外部 EN 引脚）：网页连上时会读回这个值
        params.en_mode = static_cast<uint16_t>(owner_.getEnableMode());
        // 滞后角模型的可调参数（见 docs/lag_angle_and_pid.md）
        params.lag_accel_coef_mdeg_per_rpm_s = owner_.lag_accel_coef_mdeg_per_rpm_s_;
        params.lag_deadband_mdeg = owner_.lag_deadband_mdeg_;
        // v5：运动源 + 共振带。运动源是"设备形态"（PLC 直连 vs 网页调试），
        // 重启必须保持；共振带是这台机器的物理属性，更该持久。
        params.motion_source = static_cast<uint16_t>(owner_.getMotionSource());
        params.resonance_lo_rpm = owner_.resonance_lo_rpm_;
        params.resonance_hi_rpm = owner_.resonance_hi_rpm_;
        // v6：堵转阈值。这台机器"多慢算堵转"是物理属性（惯量/负载/电流），该持久。
        params.stall_quiet_ms = static_cast<uint16_t>(owner_.stall_quiet_ms_);
        params.stall_move_lsb = static_cast<uint16_t>(owner_.stall_move_lsb_);
        // v7：磁钢极性。校准探针自动管理并在校准完成时自动固化；「写入参数」时
        // 也带上它，保证手动排障改过 0x176 后用户能显式固化。
        params.encoder_invert = (owner_.encoder_ != nullptr && owner_.encoder_->isInvertRaw()) ? 1U : 0U;
        // 角度限速/加速度不存 Flash：它们属于“本次运行设定”，每次按现场需要在界面上调，
        // 存进 Flash 反而会让一个旧值在重启后静默生效。结构体字段保留（置 0）用于兼容。
        params.angle_max_rpm = 0U;
        params.angle_accel_rpm_s = 0U;

        owner_.param_save_ok_ = param_store_save(&params);
        // 日志按“网页上的值”打印：存储里放大 1000 倍只为保留小数，
        // 直接打 60000 会让人误以为有个倍率在捣鬼。
        printf("[PARAM] save %s (pid %.3f/%.3f/%.3f %.3f/%.3f/%.3f)\r\n",
               owner_.param_save_ok_ ? "ok" : "FAILED",
               params.position_kp / 1000.0f,
               params.position_ki / 1000.0f,
               params.position_kd / 1000.0f,
               params.velocity_kp / 1000.0f,
               params.velocity_ki / 1000.0f,
               params.velocity_kd / 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FF_RATIO)
    {
        // 滞后角前馈信任度：网页传百分数 0~100，内部存 0.0~1.0。
        // 写 0 = 完全关闭前馈（退回无前馈行为）；写 100 = 完全相信表。
        const uint32_t percent = value > 100U ? 100U : value;
        owner_.lag_angle_feedforward_ratio_ = static_cast<float>(percent) / 100.0f;
        return owner_.protocol_ != nullptr && owner_.protocol_->writeRegister(reg, percent);
    }
    // ===== 校准期间：所有“闭环 / 运动命令”入口由校准器接管 =====
    // 磁编码器校准必须纯开环跑：每一段的“真角”完全由脉冲数定义。如果这时候还允许
    // 闭环补偿（0x107）、角度模式（0x128）、外部脉冲跟随（0x149）或主机的运动命令
    //（0x11F=1）进来，PID 就会往校准段里掺额外脉冲 —— emitSteps 实际发的是
    // 补偿累加器的步数，而“这段走完了”看的是原始累加器的记账，两者一旦不等，
    // 整圈进给就会被抬高（实测 +3.0%，约 1.5 微步/段）。
    // 校准器自己的 startMotion()/setTargetAngleDeg() 是直接 C++ 调用、不走寄存器入口，
    // 所以“归零用角度模式 + 扫段用开环规划运动”的结构不受影响。
    // 0x11F=0（停下）仍然放行：那是急停，不该被挡；校准器会在下一段发现起不来时
    // 自己 finish(false) 干净中止。
    if (!owner_.calibrator_.isIdle() &&
        (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE ||
         reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG ||
         reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE ||
         (reg == TMC2209_EXT_PARAM_MOTION_COMMAND && value != 0U)))
    {
        printf("[CALIBRATION] write rejected while calibrating (open-loop only): reg=0x%03lX value=%lu\r\n",
               static_cast<unsigned long>(reg), static_cast<unsigned long>(value));
        // 被挡掉的写入不能让前端“以为成功了”：立即把固件里的真值回灌一次。
        // rx.js 会用 0x0107 的回报帧把“闭环使能”勾选框弹回真实状态；
        // 其他几个被挡的寄存器没有对应控件状态要同步，不必回灌。
        if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE && owner_.protocol_ != nullptr)
        {
            owner_.protocol_->setCustomParameter(reg,
                owner_.closed_loop_compensation_enabled_ ? 1U : 0U);
        }
        return true;   // 照常回 ACK，但不让控制结构在校准中途被改掉
    }
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        owner_.closed_loop_compensation_enabled_ = value != 0U;
        owner_.position_pid_.resetIntegral();
        owner_.position_pid_.resetDeriv();
        owner_.velocity_pid_.resetIntegral();
        owner_.velocity_pid_.resetDeriv();
        return owner_.protocol_ != nullptr && owner_.protocol_->writeRegister(reg, value);
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        owner_.setEncoderZero(static_cast<uint16_t>(value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_UPLOAD_COUNT)
    {
        owner_.lag_angle_fit_saved_ = false;
        return owner_.lag_angle_.beginUpload(static_cast<uint16_t>(value));
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_INDEX)
    {
        owner_.lag_angle_point_index_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_SPEED_RPM)
    {
        owner_.lag_angle_point_speed_rpm_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_MDEG)
    {
        owner_.lag_angle_point_mdeg_ = static_cast<int32_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_COMMIT)
    {
        return value != 0U && owner_.lag_angle_.setPoint(owner_.lag_angle_point_index_,
                                                         owner_.lag_angle_point_speed_rpm_,
                                                         owner_.lag_angle_point_mdeg_);
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FIT)
    {
        if (value == 0U)
        {
            return true;
        }
        // 【fit() 之后紧跟 saveToFlash()，同样 stall 总线几十毫秒】原来这条路径
        // 完全没有运动门，比 0x154 更隐蔽：滞后角整定的收尾就是"写 0x141 拟合并存 Flash"，
        // 如果此时电机还在扫速运动，擦除窗口内控制环消失 → 丢步。整定流程应已停转，
        // 但这是安全不变量，固件侧必须自己挡（上位机 bug 不能变成丢步）。
        const char* danger = motion_danger();
        if (danger != nullptr)
        {
            printf("[LAG] fit rejected: %s is active\r\n", danger);
            owner_.lag_angle_fit_saved_ = false;
            return true;
        }
        owner_.lag_angle_fit_saved_ = owner_.lag_angle_.fit() && owner_.lag_angle_.saveToFlash();
        return owner_.lag_angle_fit_saved_;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_SPEED_RPM)
    {
        owner_.lag_angle_query_speed_rpm_ = static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        owner_.motion_start_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        owner_.motion_max_rpm_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        owner_.motion_accel_rpm_s_ = static_cast<float>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        owner_.angle_max_rpm_ = static_cast<float>(value) < 1.0f ? 1.0f
            : (static_cast<float>(value) > 12000.0f ? 12000.0f : static_cast<float>(value));
        return owner_.protocol_ != nullptr &&
               owner_.protocol_->setCustomParameter(reg, static_cast<uint32_t>(owner_.angle_max_rpm_));
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        owner_.angle_accel_rpm_s_ = static_cast<float>(value) < 1.0f ? 1.0f
            : (static_cast<float>(value) > 12000.0f ? 12000.0f : static_cast<float>(value));
        return owner_.protocol_ != nullptr &&
               owner_.protocol_->setCustomParameter(reg, static_cast<uint32_t>(owner_.angle_accel_rpm_s_));
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_CONTROL)
    {
        if (value != 0U)
        {
            owner_.pid_auto_tuner_.beginTrial();
        }
        else
        {
            owner_.pid_auto_tuner_.finishTrial();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        owner_.motion_pulse_count_ = value;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        owner_.motion_mode_ = static_cast<ClosedLoopController::MotionMode>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        if (value != 0U)
        {
            owner_.startMotion();
        }
        else
        {
            owner_.closed_loop_angle_mode_enabled_ = false;
            // 跟随时 stopMotion() 停不掉“跟随”本身：external_follow_enabled_ 还开着，
            // 下一拍就会接着发。所以停运动必须把跟随一起关掉。
            owner_.enableExternalPulseFollow(false);
            owner_.stopMotion();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATE_ENCODER)
    {
        owner_.calibrateEncoder();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        // 只登记期望值，真正的重配交给遥测慢档（syncProtocolTelemetry），
        // 否则会在运动中途改滤波窗口，把跟随误差抖出台阶。
        owner_.encoder_filter_window_ = value > EncoderFilterConfig::kMaxWindow
            ? EncoderFilterConfig::kMaxWindow
            : static_cast<uint16_t>(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        // 0x149 是 0x16D 运动源的"跟随开关"别名：走同一个 setMotionSource，
        // 两个寄存器永远同步（以前各写各的，切一半会出现"跟随开着但运动源显示内部"）。
        owner_.setMotionSource(value != 0U ? ClosedLoopController::MOTION_SOURCE_EXTERNAL
                                           : ClosedLoopController::MOTION_SOURCE_INTERNAL);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        // 0 = 自动（按 angle_max_rpm_ 换算）；上限按 2.56Mstep/s（128 步/窗）夹住
        owner_.external_pulse_follower_.setMaxStepRate(
            value > ExternalPulseFollower::kMaxStepRateLimit
                ? ExternalPulseFollower::kMaxStepRateLimit
                : value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_RESET)
    {
        if (value != 0U)
        {
            owner_.resetExternalPulseFollowCounters();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        const uint64_t clamped_ns = value < ClosedLoopController::kMinStepPulseNs
            ? ClosedLoopController::kMinStepPulseNs
            : (value > ClosedLoopController::kMaxStepPulseNs
                   ? ClosedLoopController::kMaxStepPulseNs
                   : static_cast<uint64_t>(value));
        owner_.step_pulse_width_ns_ = clamped_ns;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        owner_.calibration_lookup_raw_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        owner_.calibration_lookup_corrected_value_ = owner_.encoder_ == nullptr
            ? 0U
            : owner_.encoder_->getCorrectedRaw(owner_.calibration_lookup_raw_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        owner_.calibration_lookup_corrected_value_ = static_cast<uint16_t>(value & 0xFFFFU);
        owner_.calibration_lookup_raw_value_ = owner_.encoder_ == nullptr
            ? 0U
            : owner_.lookupOriginalRaw(owner_.calibration_lookup_corrected_value_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        owner_.setTargetAngleDeg(static_cast<float>(value) / 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
        reg == TMC2209_EXT_PARAM_POSITION_KD || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
        reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        // 上位机传的是放大 1000 倍的整数，保留三位小数；上限见 clampPidGain。
        const float gain = clampPidGain(reg, value);
        if (reg == TMC2209_EXT_PARAM_POSITION_KP) owner_.base_position_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KI) owner_.base_position_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_POSITION_KD) owner_.base_position_kd_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KP) owner_.base_velocity_kp_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KI) owner_.base_velocity_ki_ = gain;
        if (reg == TMC2209_EXT_PARAM_VELOCITY_KD) owner_.base_velocity_kd_ = gain;
        owner_.position_pid_.setGains(owner_.base_position_kp_, owner_.base_position_ki_, owner_.base_position_kd_);
        owner_.velocity_pid_.setGains(owner_.base_velocity_kp_, owner_.base_velocity_ki_, owner_.base_velocity_kd_);
        owner_.position_pid_.resetIntegral();
        owner_.position_pid_.resetDeriv();
        owner_.velocity_pid_.resetIntegral();
        owner_.velocity_pid_.resetDeriv();
        return owner_.protocol_ != nullptr && owner_.protocol_->setCustomParameter(reg, value);
    }
    // 【已废弃】0x156~0x15B：三模式已合并为一组参数（0x129~0x12E）。
    // 不默默丢弃也不默默重定向：写它们会报错并提示新入口，避免旧脚本悄悄改错参数。
    if (reg >= TMC2209_EXT_PARAM_ANGLE_POSITION_KP_DEPRECATED &&
        reg <= TMC2209_EXT_PARAM_ANGLE_VELOCITY_KD_DEPRECATED)
    {
        printf("[PARAM] reg 0x%03lX is deprecated (angle-mode gains merged into 0x129~0x12E)\r\n",
               static_cast<unsigned long>(reg));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_LEAD_ALPHA)
    {
        // 位置环超前系数 α：填 0.001 口径（1000 = 1.0 = 不启用），限 1.0~5.0。
        // 只作用于位置环的 D 通道（超前网络级在微分低通之后），不影响速度环。
        float alpha = static_cast<float>(value) / 1000.0f;
        if (alpha < 1.0f)
        {
            alpha = 1.0f;
        }
        else if (alpha > 5.0f)
        {
            alpha = 5.0f;
        }
        owner_.position_pid_.lead_alpha = alpha;
        // 改了超前时间常数就要清掉它的历史状态，否则旧状态会在新 α 下发一段错值
        owner_.position_pid_.resetDeriv();
        printf("[PARAM] position lead alpha = %.3f\r\n", static_cast<double>(alpha));
        return owner_.protocol_ != nullptr &&
               owner_.protocol_->setCustomParameter(reg, static_cast<uint32_t>(alpha * 1000.0f + 0.5f));
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ACCEL_COEF)
    {
        // 毫度/(RPM/s)：上限 50 已经很夸张（600RPM/s 会给 30°），只做安全钳位
        const uint32_t clamp_value = value > 50U ? 50U : value;
        owner_.lag_accel_coef_mdeg_per_rpm_s_ = static_cast<uint16_t>(clamp_value);
        printf("[PARAM] lag accel coef = %lu mdeg/(RPM/s)\r\n", static_cast<unsigned long>(clamp_value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_DEADBAND_MDEG)
    {
        const uint32_t clamp_value = value > 2000U ? 2000U : value;
        owner_.lag_deadband_mdeg_ = static_cast<uint16_t>(clamp_value);
        printf("[PARAM] lag deadband = %lu mdeg\r\n", static_cast<unsigned long>(clamp_value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENABLE_MODE)
    {
        owner_.setEnableMode(value);
        printf("[PARAM] enable mode = %s (0x161=%lu)\r\n",
               value == ClosedLoopController::ENABLE_MODE_EXTERNAL_PIN ? "external EN pin (PC13, low-active)" : "host",
               static_cast<unsigned long>(value));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_QUICK_STOP)
    {
        if (value != 0U)
        {
            owner_.requestQuickStop();
        }
        return true;
    }
    // ===== 运动源 / 急停 / 共振带（2026-11 第二批）=====
    if (reg == TMC2209_EXT_PARAM_MOTION_SOURCE)
    {
        owner_.setMotionSource(value);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EMERGENCY_STOP)
    {
        if (value != 0U)
        {
            owner_.emergencyStop();
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_LO_RPM)
    {
        owner_.resonance_lo_rpm_ = static_cast<uint16_t>(value > 6000U ? 6000U : value);
        printf("[RESONANCE] band lo = %lu rpm\r\n", static_cast<unsigned long>(owner_.resonance_lo_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_HI_RPM)
    {
        owner_.resonance_hi_rpm_ = static_cast<uint16_t>(value > 6000U ? 6000U : value);
        printf("[RESONANCE] band hi = %lu rpm\r\n", static_cast<unsigned long>(owner_.resonance_hi_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_BYPASS)
    {
        owner_.resonance_bypass_ = value != 0U;
        printf("[RESONANCE] 0x171 = %u recorded (no-op: since 2026-11-12 the band no longer "
               "clamps any motion, it is informational only)\r\n",
               owner_.resonance_bypass_ ? 1U : 0U);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_QUIET_MS)
    {
        // 堵转判定的"静止时长"阈值。钳位 50~3000ms。USB 任务里跑，printf 安全。
        uint32_t v = value;
        if (v < 50U)
        {
            v = 50U;
        }
        else if (v > 3000U)
        {
            v = 3000U;
        }
        owner_.stall_quiet_ms_ = v;
        printf("[STALL] quiet time = %lu ms\r\n", static_cast<unsigned long>(v));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_MOVE_LSB)
    {
        // 堵转判定的"净位移"阈值，单位 LSB。钳位 40~1500（须高于静止噪声 ~100LSB
        // 窗内净位移，又远低于最低速真实位移 1750/200ms）。
        uint32_t v = value;
        if (v < 40U)
        {
            v = 40U;
        }
        else if (v > 1500U)
        {
            v = 1500U;
        }
        owner_.stall_move_lsb_ = v;
        printf("[STALL] move threshold = %lu LSB\r\n", static_cast<unsigned long>(v));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_INVERT)
    {
        // 磁钢极性修正开关。正常由校准探针自动管理，这里允许手动改写做排障。
        // 校准进行中禁止改：会把"探针正在测的坐标系"和"表正在学的坐标系"撕裂。
        if (!owner_.calibrator_.isIdle())
        {
            printf("[ENCODER] invert write rejected: calibration active\r\n");
            return true;
        }
        const bool invert = (value != 0U);
        if (owner_.encoder_ != nullptr)
        {
            owner_.encoder_->setInvertRaw(invert);
            printf("[ENCODER] raw angle invert = %u (recalibrate to keep table in sync)\r\n",
                   invert ? 1U : 0U);
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_CLEAR)
    {
        if (value != 0U)
        {
            owner_.clearStall();
            printf("[STALL] cleared by host\r\n");
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_ENABLE)
    {
        // 显式使能：清除安全锁，允许启动
        owner_.user_disabled_ = false;
        owner_.stopMotion();
        // 统一入口：外部 EN 模式下这里不会强行把芯片 EN 拉低（引脚说了算）。
        owner_.requestDriverEnable(true);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTOR_DISABLE)
    {
        // 【安全锁】记住“用户主动失能”：之后任何 0x11F=1 / setTargetAngleDeg 都会拒绝，
        // 必须重新收到 MOTOR_ENABLE 才能再动。
        // 否则自动测试循环里的下一条 0x11F=1 会通过 startMotion 的 setEnable(true)
        // 把电流重新打开——表现为“已失能但电机自己转起来”。
        owner_.user_disabled_ = true;
        owner_.closed_loop_angle_mode_enabled_ = false;
        owner_.stopMotion();
        owner_.requestDriverEnable(false);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SINGLE_HALF_ROUND_FORWARD_STEPS)
    {
        const uint32_t micro_steps_per_round = owner_.getStepsPerRound();
        const uint32_t half_round_steps = micro_steps_per_round / 2U;

        // 设置运动参数，交给梯形规划引擎输出脉冲
        owner_.setMotionConfig(owner_.motion_start_rpm_, owner_.motion_max_rpm_, owner_.motion_accel_rpm_s_,
                               half_round_steps, ClosedLoopController::MOTION_MODE_POSITION_FORWARD);
        owner_.startMotion();

        owner_.last_en_state_ = 2U;
        return true;
    }
    if (owner_.protocol_ == nullptr)
    {
        return false;
    }
    return owner_.protocol_->writeRegister(reg, value);
}

bool ParameterRouter::read(uint16_t reg, uint32_t* value)
{
    if (value == nullptr)
    {
        return false;
    }
    if (reg == TMC2209_EXT_PARAM_SAVE_PARAMS)
    {
        *value = owner_.param_save_ok_ ? 1U : 0U;
        return true;
    }
    // 【PID 回读必须实时取值】原来 PID 六项走的是 custom_parameters_ 缓存，
    // 而 init() 从 Flash 恢复时只改了 position_pid_ / velocity_pid_，没同步那份缓存，
    // 于是重启后网页读到的仍是旧值（0 或编译期默认），看起来“写入成功但重连没读出来”。
    if (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
        reg == TMC2209_EXT_PARAM_POSITION_KD || reg == TMC2209_EXT_PARAM_VELOCITY_KP ||
        reg == TMC2209_EXT_PARAM_VELOCITY_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        const PidController& pid = (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_POSITION_KI ||
                                    reg == TMC2209_EXT_PARAM_POSITION_KD)
            ? owner_.position_pid_ : owner_.velocity_pid_;
        const float gain = (reg == TMC2209_EXT_PARAM_POSITION_KP || reg == TMC2209_EXT_PARAM_VELOCITY_KP) ? pid.kp
                         : (reg == TMC2209_EXT_PARAM_POSITION_KI || reg == TMC2209_EXT_PARAM_VELOCITY_KI) ? pid.ki
                         : pid.kd;
        *value = static_cast<uint32_t>(gain * 1000.0f + 0.5f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FF_RATIO)
    {
        // 回读百分数（0~100），与写入口径一致
        *value = static_cast<uint32_t>(owner_.lag_angle_feedforward_ratio_ * 100.0f + 0.5f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CLOSED_LOOP_ENABLE)
    {
        *value = owner_.closed_loop_compensation_enabled_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_ZERO)
    {
        *value = owner_.encoder_zero_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_START_RPM)
    {
        *value = static_cast<uint32_t>(owner_.motion_start_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MAX_RPM)
    {
        *value = static_cast<uint32_t>(owner_.motion_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(owner_.motion_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_MAX_RPM)
    {
        *value = static_cast<uint32_t>(owner_.angle_max_rpm_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ANGLE_ACCEL_RPM_S)
    {
        *value = static_cast<uint32_t>(owner_.angle_accel_rpm_s_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PULSE_COUNT)
    {
        *value = owner_.motion_pulse_count_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_MODE)
    {
        *value = static_cast<uint32_t>(owner_.motion_mode_);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND)
    {
        *value = owner_.motion_running_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_SPEED_RPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.encoder_speed_rpm_));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_DEG)  // 实际是跟随误差
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.motion_follow_error_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_FILTER_WINDOW)
    {
        *value = owner_.encoder_filter_window_active_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_ENABLE)
    {
        *value = owner_.external_pulse_follower_.isEnabled() ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_MAX_STEP_S)
    {
        *value = owner_.external_pulse_follower_.maxStepRate();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_CMD_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_pulse_follower_.cmdSteps());
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_EMITTED_STEPS)
    {
        *value = static_cast<uint32_t>(owner_.external_pulse_follower_.emittedSteps());
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_BACKLOG)
    {
        *value = owner_.external_pulse_follower_.backlogSteps();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_DROPPED)
    {
        *value = owner_.external_pulse_follower_.droppedSteps();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_FOLLOW_WINDOW_REVERSALS)
    {
        *value = owner_.external_pulse_follower_.windowReversals();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_PULSE_WIDTH_NS)
    {
        *value = owner_.step_pulse_width_ns_;
        return true;
    }
    // 脉冲质量观测（只读）。0x177：上次 startMotion 以来单窗"索要"的最大步数
    // （450RPM/64细分 正常 ≈5~6；几十 = 控制任务停过，DDA 截断已挡住爆发）。
    // 0x178：任务停顿事件累计（窗口间隔 >150µs），持续增长说明有 CPU 抢占源。
    if (reg == TMC2209_EXT_PARAM_STEP_MAX_WINDOW_STEPS)
    {
        *value = owner_.max_step_per_ramp;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STEP_TASK_STALL_EVENTS)
    {
        *value = owner_.step_catchup_events_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_RAW_TO_CORRECTED)
    {
        *value = owner_.calibration_lookup_corrected_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CALIBRATION_LOOKUP_CORRECTED_TO_RAW)
    {
        *value = owner_.calibration_lookup_raw_value_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_TARGET_ANGLE_DEG)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.target_angle_deg_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_INDEX)
    {
        *value = owner_.lag_angle_point_index_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_SPEED_RPM)
    {
        uint16_t speed_rpm = owner_.lag_angle_point_speed_rpm_;
        int32_t lag_angle_mdeg = owner_.lag_angle_point_mdeg_;
        owner_.lag_angle_.getPoint(owner_.lag_angle_point_index_, &speed_rpm, &lag_angle_mdeg);
        *value = speed_rpm;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_MDEG)
    {
        uint16_t speed_rpm = owner_.lag_angle_point_speed_rpm_;
        int32_t lag_angle_mdeg = owner_.lag_angle_point_mdeg_;
        owner_.lag_angle_.getPoint(owner_.lag_angle_point_index_, &speed_rpm, &lag_angle_mdeg);
        *value = static_cast<uint32_t>(lag_angle_mdeg);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_SPEED_RPM)
    {
        *value = owner_.lag_angle_query_speed_rpm_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_QUERY_MDEG)
    {
        *value = static_cast<uint32_t>(owner_.lag_angle_.getLagAngleMilliDegrees(owner_.lag_angle_query_speed_rpm_));
        return owner_.lag_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_POINT_COUNT)
    {
        *value = owner_.lag_angle_.pointCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FIT)
    {
        *value = owner_.lag_angle_fit_saved_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FIT_SLOPE)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.lag_angle_.powerCoefficient() * 1000000.0f));
        return owner_.lag_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FIT_INTERCEPT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.lag_angle_.interceptDeg() * 1000.0f));
        return owner_.lag_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ANGLE_FIT_EXPONENT)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.lag_angle_.exponent() * 1000000.0f));
        return owner_.lag_angle_.isFitted();
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KP)
    {
        *value = static_cast<uint32_t>(owner_.base_position_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KI)
    {
        *value = static_cast<uint32_t>(owner_.base_position_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_POSITION_KD)
    {
        *value = static_cast<uint32_t>(owner_.base_position_kd_ * 1000.0f);
        return true;
    }
    // 位置环超前系数 α（不存 Flash，重启后回到 1.0）
    if (reg == TMC2209_EXT_PARAM_POSITION_LEAD_ALPHA)
    {
        *value = static_cast<uint32_t>(owner_.position_pid_.lead_alpha * 1000.0f + 0.5f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SCORE_MDEG)
    {
        *value = owner_.pid_auto_tuner_.scoreMilliDegrees();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_SAMPLE_COUNT)
    {
        *value = owner_.pid_auto_tuner_.sampleCount();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_PID_TUNE_PEAK_MDEG)
    {
        *value = owner_.pid_auto_tuner_.peakMilliDegrees();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_ACCEL_COEF)
    {
        *value = owner_.lag_accel_coef_mdeg_per_rpm_s_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_LAG_DEADBAND_MDEG)
    {
        *value = owner_.lag_deadband_mdeg_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENABLE_MODE)
    {
        *value = owner_.getEnableMode();
        return true;
    }
    // ===== 运动源 / 共振带 / 可观测计数器（2026-11 第二批）=====
    if (reg == TMC2209_EXT_PARAM_MOTION_SOURCE)
    {
        *value = owner_.getMotionSource();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_LO_RPM)
    {
        *value = owner_.resonance_lo_rpm_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_HI_RPM)
    {
        *value = owner_.resonance_hi_rpm_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RESONANCE_BYPASS)
    {
        *value = owner_.resonance_bypass_ ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EN_GLITCH_COUNT)
    {
        *value = owner_.en_glitch_count_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_RX_OVERFLOW_COUNT ||
        reg == TMC2209_EXT_PARAM_RX_CRC_FAIL_COUNT ||
        reg == TMC2209_EXT_PARAM_TX_DROP_COUNT ||
        reg == TMC2209_EXT_PARAM_WRITE_DENIED_COUNT)
    {
        UsbCdcProtocolBridge* bridge = UsbCdcProtocolBridge::instance();
        if (bridge != nullptr)
        {
            *value = (reg == TMC2209_EXT_PARAM_RX_OVERFLOW_COUNT) ? bridge->rxOverflowCount()
                   : (reg == TMC2209_EXT_PARAM_RX_CRC_FAIL_COUNT) ? bridge->rxCrcFailCount()
                   : (reg == TMC2209_EXT_PARAM_TX_DROP_COUNT) ? bridge->txDropCount()
                   : bridge->writeDeniedCount();
        }
        else
        {
            *value = 0U;
        }
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENABLE_STATE)
    {
        *value = owner_.isDriverEnabled() ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_EXT_EN_LEVEL)
    {
        *value = owner_.getExternalEnableLevel();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_COMP_RPS_MILLI)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.compensated_rps_ * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_DETECTED)
    {
        *value = owner_.isStallDetected() ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_EVENTS)
    {
        *value = owner_.getStallEvents();
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_QUIET_MS)
    {
        *value = owner_.stall_quiet_ms_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_STALL_MOVE_LSB)
    {
        *value = owner_.stall_move_lsb_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_ENCODER_INVERT)
    {
        *value = (owner_.encoder_ != nullptr && owner_.encoder_->isInvertRaw()) ? 1U : 0U;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_SCHEDULED_STEPS)
    {
        *value = owner_.motion_steps_emitted_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_MOTION_COMMAND_SPEED_MRPM)
    {
        *value = static_cast<uint32_t>(static_cast<int32_t>(owner_.motion_speed_rpm_ *
                                                             static_cast<float>(owner_.motion_direction_) * 1000.0f));
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CPU_USAGE_CENTIPERCENT)
    {
        *value = owner_.cpu_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_CONTROL_TASK_USAGE_CENTIPERCENT)
    {
        *value = owner_.control_task_usage_centi_percent_;
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KP)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_kp_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KI)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_ki_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_EXT_PARAM_VELOCITY_KD)
    {
        *value = static_cast<uint32_t>(owner_.base_velocity_kd_ * 1000.0f);
        return true;
    }
    if (reg == TMC2209_REG_TSTEP)
    {
        *value = owner_.step_period_us_;
        return true;
    }
    if (reg == TMC2209_REG_MSCNT)
    {
        *value = owner_.encoder_filtered_angle_;
        return true;
    }
    if (reg == TMC2209_REG_DRV_STATUS)
    {
        *value = owner_.magnetic_field_high_ || owner_.magnetic_field_low_ ? (1U << 24U) : 0U;
        return true;
    }
    // 扩展表里没认识的地址，交给协议层的官方 TMC 寄存器分支。
    if (owner_.protocol_ == nullptr)
    {
        return false;
    }
    return owner_.protocol_->readRegister(reg, value);
}
