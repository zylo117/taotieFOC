//
// Created by Jessi on 2026/10/7.
//

#include "pid_controller.h"

PidController::PidController()
    : kp(1.0f), ki(0.04f), kd(0.02f), integral(0.0f), last_error(0.0f),
      max_integral(MAX_I_TERM), max_output(MAX_PID_OUTPUT)
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
    float derivative = 0.0f;
    float output = 0.0f;

    if (dt > 0.0f)
    {
        // 误差变化率用于反映系统“趋势”，可帮助提前修正冲击和速度变化。
        derivative = (error - last_error) / dt;
    }

    // 科学意义：积分项是误差的累积，能消除长期偏差。
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

    // 总输出等于三个项加权和：位置和速度环都使用这一套结构。
    output = kp * error + ki * integral + kd * derivative;
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
}