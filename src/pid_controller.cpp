//
// Created by Jessi on 2026/10/7.
//

#include "pid_controller.h"

PidController::PidController()
    : kp(1.0f), ki(0.04f), kd(0.02f), integral(0.0f), last_error(0.0f),
      filtered_derivative(0.0f), max_integral(MAX_I_TERM), max_output(MAX_PID_OUTPUT),
      lead_alpha(1.0f), lead_prev_out(0.0f), lead_prev_in(0.0f)
{
}

void PidController::setGains(float kp_value, float ki_value, float kd_value)
{
    kp = kp_value;
    ki = ki_value;
    kd = kd_value;
}

// PID 的核心计算公式：
// output = Kp * error + Ki * integral(error) + Kd * d(error)/dt
// 其中：
// - P 项让系统对误差有直接修正能力
// - I 项用于消除稳态误差
// - D 项用于抑制超调和提高阻尼
// 但在电机闭环中，积分项和微分项不能无限大，否则会导致振荡，因此要做限幅。
float PidController::update(float error, float dt, float p_scale,
                            float measured_rate, bool d_on_measurement)
{
    // 时间差非法（首拍 / 调度异常）：只给比例项，不碰积分和微分状态。
    // 旧实现里 dt=0 时 derivative 保持 0、但 integral 仍然累加一个 0，
    // 看上去无害，可一旦外界传入极小的 dt（比如 1ns）就会把 derivative 推到 inf。
    if (dt <= 0.0f)
    {
        return kp * p_scale * error;
    }

    // ---- 微分项：先算原始斜率，再做一阶低通 ----
    // 直接对 (error - last_error)/dt 做 D 会被编码器量化噪声主导：
    // 1 LSB = 0.0055°，dt = 50us 时原始斜率就是 114°/s 的尖峰，
    // 乘 kd=60 得 6800，实际效果接近 bang-bang，D 项根本没在提供阻尼。
    // 微分先行：d_on_measurement 时不对误差求导，而是用测量变化率的负值。
    // 两者在线性段的阻尼效果等价，但前者在改目标值时不会产生微分冲击。
    const float raw_derivative = d_on_measurement
        ? (-measured_rate)
        : ((error - last_error) / dt);
    filtered_derivative = PID_DERIVATIVE_LPF * filtered_derivative +
                          (1.0f - PID_DERIVATIVE_LPF) * raw_derivative;

    const float p_term = kp * p_scale * error;

    // 【超前级】H(s) = (1 + τz·s)/(1 + τp·s)，τz = α·τp，级在上面的 LPF 之后。
    // 后向欧拉离散：(T+τp)·y[k] = τp·y[k-1] + (T+τz)·x[k] - τz·x[k-1]
    // α==1 时 τz==τp，代入可得 y[k] == x[k]（逐位纯通过，含首拍），所以默认值
    // 不会改变任何已有行为；要启用只需写 0x15C（α>1）。
    float d_signal = filtered_derivative;
    if (lead_alpha > 1.0001f)
    {
        const float beta = PID_DERIVATIVE_LPF;
        const float tau_p = (beta < 0.999f) ? (dt * beta / (1.0f - beta)) : dt;
        const float tau_z = tau_p * lead_alpha;
        const float denom = dt + tau_p;
        if (denom > 0.0f)
        {
            const float y = (tau_p * lead_prev_out
                             + (dt + tau_z) * filtered_derivative
                             - tau_z * lead_prev_in) / denom;
            lead_prev_out = y;
            lead_prev_in = filtered_derivative;
            d_signal = y;
        }
    }

    const float d_term = kd * d_signal;

    // ---- 条件积分（anti-windup）----
    // 只在“误差方向会加剧饱和”时停止积分；误差反号（正在往回调）时继续积分，
    // 否则退饱和会慢半拍，反而造成大超调。
    // 注意：max_output 目前是 50000，对 rps 输出而言等于不限幅，
    // 所以这条保护实际主要靠 max_integral 兜底。要让它真正生效，
    // 需要把某个环的 max_output 设成该环执行器的物理上限。
    float output = p_term + ki * integral + d_term;
    const bool saturating_up = (output >= max_output) && (error > 0.0f);
    const bool saturating_down = (output <= -max_output) && (error < 0.0f);
    if (!saturating_up && !saturating_down)
    {
        // 积分项是误差的累积，能消除长期偏差。
        // 但如果不限制，它很容易带来持续过冲，所以必须做区间裁剪。
        integral += error * dt;
        if (integral > max_integral)
        {
            integral = max_integral;
        }
        else if (integral < -max_integral)
        {
            integral = -max_integral;
        }
        output = p_term + ki * integral + d_term;
    }

    // 总输出等于三个项加权和：位置和速度环都使用这一套结构。
    if (output > max_output)
    {
        output = max_output;
    }
    else if (output < -max_output)
    {
        output = -max_output;
    }

    // 更新微分历史值，保证下一周期以最新误差为基准。
    last_error = error;
    return output;
}

void PidController::resetIntegral()
{
    integral = 0.0f;
}

void PidController::resetDeriv()
{
    last_error = 0.0f;
    // 滤波器状态也要一起清，否则重新使能后前几拍的 D 项会带着上一次运动的旧斜率。
    filtered_derivative = 0.0f;
    // 超前级是两个一阶状态，同样必须清：否则它的记忆会跳过一次模式切换。
    lead_prev_out = 0.0f;
    lead_prev_in = 0.0f;
}

void PidController::reset()
{
    resetIntegral();
    resetDeriv();
}

void PidController::setOutputLimit(float limit)
{
    const float limit_abs = (limit < 0.0f) ? -limit : limit;
    // 上限为 0 会让环路彻底失去作用（输出恒 0），按“不限制”处理更安全：
    // 调用方算出的限幅在极端情况下（profile 速度为 0）确实可能是 0。
    max_output = (limit_abs > 1.0e-6f) ? limit_abs : MAX_PID_OUTPUT;

    // 反推积分限幅：让 I 项单独最多顶到输出上限，否则 I 会先绕到 100
    // 再由下游钳位削掉，退饱和要等好几拍（这就是“超调”的来源之一）。
    const float ki_abs = (ki < 0.0f) ? -ki : ki;
    float integral_limit = (ki_abs > 1.0e-6f) ? (max_output / ki_abs) : MAX_I_TERM;
    if (integral_limit > MAX_I_TERM)
    {
        integral_limit = MAX_I_TERM;
    }
    max_integral = integral_limit;
    if (integral > max_integral)
    {
        integral = max_integral;
    }
    else if (integral < -max_integral)
    {
        integral = -max_integral;
    }
}