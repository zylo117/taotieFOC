#include <cstdio>

#include "at32f403a_407.h"
#include <dsp/fast_math_functions.h>

#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

/**
 * @brief 初始化DWT周期计数器，必须在main最开头调用一次
 */
void dwt_cycle_counter_init(void)
{
    // 开启DWT访问权限
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;          // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 开启周期计数
    __DSB();
    __ISB();
}

/**
 * @brief 获取当前CPU周期计数值
 * @retval uint32_t CPU cycle计数
 */
static inline uint32_t dwt_get_cycle(void)
{
    return DWT->CYCCNT;
}

// 测试函数：测量 sin/cos/atan2 消耗多少cycle
void trig_test_measure(void)
{
    float theta = 1.0471975F; // 60° 弧度
    uint32_t start, end;

    start = dwt_get_cycle();

#if 0
    // -------- newlib libm --------
    float s = sinf(theta);
    float c = cosf(theta);
    float res = atan2f(s,c);
#else
    // -------- CMSIS‑DSP --------
#include <dsp/fast_math_functions.h>
    float s = arm_sin_f32(theta);
    float c = arm_cos_f32(theta);
    float res = 0;
    arm_atan2_f32(s, c, &res);
#endif

    end = dwt_get_cycle();
    uint32_t cycle_used = end - start;

    // 250MHz：1 cycle =4ns
    uint32_t ns_used = cycle_used *4U;

    // 这里用串口打印 cycle_used、ns_used
    printf("trig cycles:%u , time(ns):%u \r\n", cycle_used, ns_used);
}

int main(void)
{
    system_clock_config();
    dwt_cycle_counter_init();   //main最开头初始化DWT

    while(1)
    {
        trig_test_measure();
        //延时一段时间再打印，不要疯狂刷屏
        delay_ms(500);
    }
}
