//
// Created by Jessi on 2026/10/7.
//

#ifndef TAOTIEFOC_PID_CONTROLLER_H
#define TAOTIEFOC_PID_CONTROLLER_H

#define MAX_PID_OUTPUT          50000.0f
#define MAX_I_TERM              100.0f

// 【微分项一阶低通系数】
// 0 = 不滤波（旧行为），越大越平滑，1 = 冻结（完全不用新数据）。
// 取 0.85 对应截止频率约 10kHz（dt=50us 时），足够压掉编码器量化噪声，
// 又不影响真实的加减速趋势（那条在几百 Hz 以下）。
#define PID_DERIVATIVE_LPF      0.85f

// PidController 是一个通用 PID 控制器。
// 在闭环步进控制中，它同时用于两层控制：
// 1）位置环：控制目标位置和当前编码器位置之间的误差
// 2）速度环：控制速度参考值和真实速度之间的误差
class PidController
{
public:
    PidController();

    // 设定比例、积分、微分三个增益参数。
    void setGains(float kp, float ki, float kd);

    // 计算当前误差下的控制输出。
    // error：当前误差，dt：控制周期时间
    float update(float error, float dt);

    // 清零积分项，防止误差长期累积导致超调。
    void resetIntegral();

    // 清零微分项历史值，避免误差突变导致抖动。
    void resetDeriv();

    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    // 低通后的微分值。直接对 (error-last_error)/dt 做 D 会被编码器量化噪声主导：
    // 1 LSB = 0.0055°，dt = 50us 时原始斜率就是 114°/s 的尖峰，乘 kd=60 得到 6800，
    // 实际效果接近 bang-bang。
    float filtered_derivative;
    float max_integral;
    float max_output;
};

#endif //TAOTIEFOC_PID_CONTROLLER_H
