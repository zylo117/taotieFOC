//
// Created by Jessi on 2026/10/7.
//

#include "pid_controller.h"

PidController::PidController()
    : kp(1.0f), ki(0.04f), kd(0.02f), integral(0.0f), last_error(0.0f),
      filtered_derivative(0.0f), max_integral(MAX_I_TERM), max_output(MAX_PID_OUTPUT)
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
float PidController::update(float error, float dt)
{
    // 时间差非法（首拍 / 调度异常）：只给比例项，不碰积分和微分状态。
    // 旧实现里 dt=0 时 derivative 保持 0、但 integral 仍然累加一个 0，
    // 看上去无害，可一旦外界传入极小的 dt（比如 1ns）就会把 derivative 推到 inf。
    if (dt <= 0.0f)
    {
        return kp * error;
    }

    // ---- 微分项：先算原始斜率，再做一阶低通 ----
    // 直接对 (error - last_error)/dt 做 D 会被编码器量化噪声主导：
    // 1 LSB = 0.0055°，dt = 50us 时原始斜率就是 114°/s 的尖峰，
    // 乘 kd=60 得 6800，实际效果接近 bang-bang，D 项根本没在提供阻尼。
    const float raw_derivative = (error - last_error) / dt;
    filtered_derivative = PID_DERIVATIVE_LPF * filtered_derivative +
                          (1.0f - PID_DERIVATIVE_LPF) * raw_derivative;

    const float p_term = kp * error;
    const float d_term = kd * filtered_derivative;

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
}