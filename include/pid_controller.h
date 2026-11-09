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
    void setGains(float kp_value, float ki_value, float kd_value);

    // 计算当前误差下的控制输出。
    // error：当前误差，dt：控制周期时间
    // p_scale 只缩放比例项，不影响积分和微分（保留是为了向后兼容旧调用，新代码不该再依赖）。
    //
    // 【d_on_measurement + measured_rate：微分先行（超前校正的等价实现之一）】
    // 对「误差」求导时，改目标值那一刻误差是阶跃，微分会打出一脚冲击（derivative kick），
    // 表现为“换个目标角度第一下猛冲”。改成对「测量值」求导（取负号），
    // 阻尼作用完全相同但没有冲击。默认 false，所有旧调用行为不变。
    float update(float error, float dt, float p_scale = 1.0f,
                 float measured_rate = 0.0f, bool d_on_measurement = false);

    // 清零积分项，防止误差长期累积导致超调。
    void resetIntegral();

    // 清零微分项历史值，避免误差突变导致抖动。
    void resetDeriv();

    // 清零全部状态（积分 + 微分 + 超前级）。
    void reset();

    // 设置本环的“物理输出上限”，并按它反推积分限幅。
    // 【为什么必须有】max_output 原来是 50000，对 rps 输出来说等于不限幅：
    // 位置环一个 1° 的误差经 kp=30 就能算出 0.083rps，而 D 项在 50us 采样下
    // 一 LSb 的跳变就是几千 rps —— 只靠下游 ±max_rpm 钳位，环路实际工作在
    // bang-bang 区，表现就是“动一点点参数就抖/超调”。把上限放到环里，
    // anti-windup 才真正生效，而且任何一组错误增益都不再可能打穿速度指令。
    // 每拍调用是允许的（只做几个比较），调用点应在 update() 之前。
    void setOutputLimit(float limit);

    // ===== 公开状态 =====
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

    // ===== 超前校正系数 α（建议只在位置环使用；默认 1.0 = 不启用）=====
    // 现状：D 项 = kd · LPF(斜率)，而那个低通会吃掉相位裕度 —— 想靠加 kd 提高阻尼，
    // 反而先引入相位落后。超前网络把这点相位补回来：
    //     H(s) = (1 + α·τp·s) / (1 + τp·s)          α>1 超前，α=1 退化成纯通过
    // 级在已有 LPF 之后，所以 α=1 时输出与不启用该项时逐位一致（默认值就是 1.0）。
    // τp 用现有 LPF 的等效时间常数 T·β/(1-β)，不再新增第二个滤波参数。
    float lead_alpha;

    // 超前级的差分方程状态（后向欧拉）：
    //   (T+τp)·y[k] = τp·y[k-1] + (T+τz)·x[k] - τz·x[k-1]
    float lead_prev_out;
    float lead_prev_in;
};

#endif //TAOTIEFOC_PID_CONTROLLER_H
