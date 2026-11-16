#include "stepper_driver.h"
#include "at32f403a_407_board.h"

namespace stepper_common
{
    namespace
    {
        bool motion_timer_running = false;
        uint32_t motion_timer_frequency = 0U;
        uint32_t motion_timer_pulse_width = 0U;
        // STEP 引脚归谁：true = 普通输出钉低（停车态），false = 定时器复用（发脉冲态）。
        bool step_pin_driven_low = false;

        // 【J1 STEP 空闲电平 2026-11-16】STEP(PB10) 平时是 TMR2_CH3 的 MUX 引脚。
        // TMR2 是高级定时器，MOE 关断后输出级断开、BDTR/OSSI 我们从来没配过 = 引脚
        // 变高阻；而 MUX 模式下 `port->clr` 写 ODR 根本不生效（本项目 DIR 引脚同一个
        // 坑，见 stepper_init_step_gpio 注释）。于是每次停车/重装之后，TMC2209 的 STEP
        // 输入是**悬空**的：任何串扰都能被计成一步 —— 实测现象就是"减速到 0，过一会儿
        // 抖一下 + 叮一声"。修法只有把引脚真正切成普通输出钉住电平。
        //
        // 切换前后电平必须一致（都是低），否则这个模式切换本身就是白送的一个沿：
        //   钉低 = 先写 ODR（MUX 下只是备好），再切 OUTPUT —— 切过去就驱低，不产生上跳。
        //   交还 = 必须在 MOE 已开、CNT=0 的状态下切 —— 此时 PWM 模式 A + STEP 低有效
        //          输出的是 CNT(0) < CCR ⇒ 参考高 ⇒ 引脚低，和钉住的低电平相同，无沿。
        void stepper_step_pin_idle_low(void)
        {
            // 【取消 STEP 引脚 GPIO↔MUX 来回重配 2026-11-17】
            // 实测：每次装载引擎都白送约 1 微步（0.056°）。12800 步最可信那组
            // 0x185=1 次装载 → +1.10 微步；0x187=1 逼出 2 次装载 → +2.89 微步。
            // 用户侧"先归零转 5 圈误差接近 0，再转 5 圈就 +0.1~0.3°"完全吻合按装载计账。
            // 嫌疑就是每次停车把 STEP 从定时器手里（MUX）切成普通 GPIO 输出、下次装载
            // 再切回去：gpio_init 改 CNF/MODE 的那几微秒里引脚不由定时器驱动，而恢复 MUX
            // 的一瞬间输出电平取决于当时的 CNT/CCR —— 那一刻只要 CNT>=CCR 引脚就是高，
            // 驱动器只认上升沿，于是每次装载多计一步。
            // 改法：STEP **永久留在 MUX**，停车时让定时器自己把输出钉低。
            // PWM 模式 A 下 CNT < CCR 期间输出有效电平（本工程 ACTIVE_LOW → 实际为低），
            // 所以只要 CCR > PR 且 CNT 摁在 0，输出就恒为低。写 CCR 要更新事件才生效
            // （c3oben 已开），顺手发一个 ovfswtr：它同时把 CCR 影子就地生效并清零 CNT。
            // 进这里的两条路径都已先关 CEN、关溢出 DMA 请求，TMR2 也没开更新中断。
            // 【改动 A 2026-11-17】上面那套"CCR>PR 且 CNT=0 就恒低"只有 in 输出级
            // 接通时才成立：本工程从未配置 OSSI/OIS（全项目 grep 'ossi|brk_init|brk_bit'
            // 无匹配），MOE 一断，输出级直接与引脚断开，上面写的 CCR/CNT 全都不驱动引脚，
            // STEP 变成高阻 —— 而 DIR(PB11) 恰好在同一个窗口里被真 GPIO 翻电平，PB10/PB11
            // 相邻，耦合出一个 Vih 沿被 TMC 白计一步：跟运动方向、与行程无关、严格正比于
            // 装载次数 0x185（实测 1 次装载 +1.10 微步、2 次 +2.89；把 GPIO 钉低删掉后
            // 涨到 +2.68，正因为它把最后一个"真在驱动引脚"的东西也删了）。
            // 所以钉低之前先把通道和输出级打开，停车态固定为 CEN=0 / MOE=1 / CCR>PR /
            // CNT=0 —— 引脚由定时器主动驱低，任何时刻都不许出现高阻窗。
            tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
            tmr_output_enable(TMR2, TRUE);
            uint32_t park = TMR2->pr + 1U;
            if (park > 65535U)
            {
                park = 65535U; // PR 已顶格：CNT 固定在 0 就是低，不需要更大 CCR
            }
            tmr_counter_value_set(TMR2, 0U);
            tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, park);
            TMR2->swevt_bit.ovfswtr = TRUE;
            step_pin_driven_low = true;
        }

        void stepper_step_pin_take_over_by_timer(void)
        {
            // 【同上 2026-11-17】STEP 从开电起就在 MUX 手里，装载引擎时不再做 GPIO 重配，
            // 也就没有"交接瞬间不定驱动"的空窗。这里只清标志；真正的低电平由
            // stepper_init_motion_timer 写好的 PR/CCR 保证（装载后 CNT=0 < CCR → 输出低）。
            step_pin_driven_low = false;
        }
    }

    void stepper_write_gpio_high(gpio_type* port, uint16_t pin) { port->scr = pin; }
    void stepper_write_gpio_low(gpio_type* port, uint16_t pin) { port->clr = pin; }

    void stepper_write_gpio(gpio_type* port, uint16_t pin, bool state)
    {
        if (state)
        {
            port->scr = pin;
        }
        else
        {
            port->clr = pin;
        }
    }

    void stepper_delay_us(uint32_t microseconds)
    {
        delay_us(microseconds);
    }

    void stepper_delay_ns(uint64_t nanoseconds)
    {
        delay_ns(nanoseconds);
    }

    // k_psc: 每tick时间， k_step_pulse_ticks: 脉宽tick数， direction: true为正
    void stepper_init_motion_timer(uint32_t k_psc, uint32_t k_step_pulse_ticks, bool direction)
    {
        tmr_counter_enable(TMR2, FALSE);
        // MOE 不再关断：一断就把输出级与引脚断开，STEP 变高阻（见 idle_low 注释）
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        // 【J1】重装期间引脚先由普通输出钉低（MUX + MOE 关断 = 高阻，见上面注释）。
        // 从这里到函数末尾交还给定时器之前，STEP 全程被驱在低电平上，不再有悬空窗口。
        stepper_step_pin_idle_low();

        tmr_output_config_type output_config;
        tmr_output_default_para_init(&output_config);

        tmr_base_init(TMR2, 1, k_psc);  // 1是垃圾值，只为了初始，后期会被dma的arr取代
        tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
        tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV2);
        tmr_period_buffer_enable(TMR2, TRUE);  //ARR预装载，保证周期不会中途撕裂波形，高频必须打开

        output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        output_config.oc_idle_state = FALSE;
        output_config.occ_idle_state = FALSE;
        output_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
        output_config.oc_output_state = TRUE;
        tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
        // 【J3a】CCR 必须预装载。换档时 CCR 数值会跳（tier0 的 250tick → tier3 的 1tick），
        // 若立即生效：此刻输出已拉高，而新 CCR 大于 CNT 会把输出又拉回低，随后 CNT 再次
        // 越过 CCR = 白送一个被驱动器计数的上升沿。开了预装载后整个周期内 CCR 不变，
        // 换档值在周期边界和 DIV / PR 一起生效，结构上杜绝这种中途改电平。
        tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
        // 高电平宽度按真正被要求的脉宽保持，随后拉低，避免过窄导致被忽略。
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, k_step_pulse_ticks);
        // 【CCR 影子必须就地生效 2026-11-16】上面 tmr_base_init 里那次 ovfswtr 更新事件发生
        // 在本行之前，而 CH3 的 CCR 是预装载的（上面 J3a）：于是此刻**工作寄存器里还是上
        // 一次运动留下的旧 CCR**（正常收尾时是 tier3 的 1~24 tick）。CEN 一放，CNT=1 就已经
        // 越过旧 CCR → 输出立刻拉高 = 在 t≈0 白送一个会被驱动器计数的上升沿，而本该充当
        // "正经第一步"的新 CCR 位置在整个第一周期里再也不会出现。这就是"每装载一次引擎
        // 就多走 1 微步"的机制：F2 当年量到的"0→250RPM 重装 3 次 = 3 微步 = 0.169° 正偏"
        // 是它，用户现在报的"永远多走 0.1~0.3°"也是它（起步装载 1 次，遇上换向 / 补尾再
        // 装载就是 2~4 微步，且与行程无关——正好是"永远"）。J1 把 STEP 钉低、把
        // MOE→引脚交接→CEN 的顺序排对都挡不住：交接后引脚确实是低，可计数器一跑就越过
        // 旧 CCR 把它顶上去。
        // 补一次更新事件把影子装进工作寄存器即可：PR / DIV 写的是同一个值，重写无副作用；
        // CNT 随后还要被显式清零（下面 tmr_counter_value_set）；DMA 溢出请求此刻也还没使能
        // （再下面才 tmr_dma_request_enable）。所以这次事件除了把 CCR 装进去没有别的后果。
        TMR2->swevt_bit.ovfswtr = TRUE;
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

        // 【DIR 修复 2026-11-12】原 CH4 PWM 恒平 trick 不可靠：CCR4 是 16 位，
        // 0xFFFFFFFF 截成 0xFFFF 与 PR 的关系随档位漂移，反向时引脚被钉在正向电平
        // ——统一引擎反向测试当场抓包（转子原地不动/仍正转）。CH4 关闭，
        // DIR 改由真 GPIO 推挽直驱（stepper_init_step_gpio 已把 PB11 配成输出），
        // 本函数在发脉冲前写入方向，天然满足建立时间。
        tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, FALSE);
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 0U);
        // 电平语义沿用旧 CH4 trick：低有效 PWM 恒置位 = 正向恒 LOW、反向 HIGH。
        // 极性搞反 = 全部正转变反转，boot 初始化 DIR=LOW(正向) 与此一致。
        if (direction)
        {
            DIR_OUTPUT_PORT->clr = (uint32_t)DIR_OUT_PIN;  // LOW = 正向
        }
        else
        {
            DIR_OUTPUT_PORT->scr = (uint32_t)DIR_OUT_PIN;  // HIGH = 反向
        }

        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);

        tmr_counter_value_set(TMR2, 0U);
        // 【J1 交接顺序 2026-11-16】先开 MOE：此刻引脚仍由普通输出驱低，开 MOE 不
        // 改变引脚电平；再把引脚交还 TMR2_CH3 —— CNT=0 < CCR ⇒ PWM 模式 A 参考为高
        // ⇒ 低有效输出仍是低，和钉住的低电平相同，交接瞬间没有上跳 = 不白送沿；
        // 最后才放 CEN，第一个上升沿要等 CNT 走到 CCR，那是正经的第一步。
        // 旧顺序（CEN 直接开、引脚整段重装时间挂在高阻上）就是 F2 实测到的
        // 「每重装一次定时器多计一步」：0→250RPM 重装 3 次 = 3 微步 = 0.169° 正偏。
        tmr_output_enable(TMR2, TRUE);
#ifndef USE_SOFT_PULSE
        stepper_step_pin_take_over_by_timer();
#endif
        tmr_counter_enable(TMR2, TRUE);
    }

    void stepper_apply_motion_rate(uint32_t k_psc, uint32_t k_step_pulse_ticks)
    {
        // 与 tmr_base_init 写的是同一个 DIV 寄存器：它是预装载的，写入只改影子，
        // 正在跑的计数器按旧分频走到周期尽头，更新事件把 DIV / PR / CCR 三份影子一起
        // 装进工作寄存器。新旧两档在同一 pps 下算出的周期时长相同（相差 <1 tick），
        // 所以脉冲间隔在换档处不出现台阶，也不需要关 MOE / 钉低 / CNT 归零。
        TMR2->div = k_psc;
        tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, k_step_pulse_ticks);
    }

    // void stepper_start_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    // {
    //     if (step_hz == 0U)
    //     {
    //         stepper_stop_motion_timer();
    //         return;
    //     }
    //
    //     crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    //     gpio_pin_remap_config(TMR2_MUX_10, TRUE);
    //
    //     gpio_init_type gpio_init_struct;
    //     gpio_default_para_init(&gpio_init_struct);
    //     gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    //     gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    //     gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    //     gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    //     gpio_init_struct.gpio_pins = STEP_OUT_PIN;
    //     gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);
    //
    //     // TMR2_MUX_10 also selects PB11 as TMR2_CH4. Re-assert PB11 as a normal
    //     // GPIO so DIR remains a static logic output while PB10 carries STEP.
    //     gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    //     gpio_init_struct.gpio_pins = DIR_OUT_PIN;
    //     gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);
    //
    //     const uint32_t timer_tick_hz = system_core_clock;
    //     uint32_t period = timer_tick_hz / step_hz;
    //     if (period < 2U)
    //     {
    //         period = 2U;
    //     }
    //     --period;
    //
    //     uint32_t pulse_ticks = (timer_tick_hz / 1000000U) * pulse_width_us;
    //     if (pulse_ticks < 1U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     if (pulse_ticks > period)
    //     {
    //         pulse_ticks = period / 2U;
    //     }
    //     if (pulse_ticks == 0U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //
    //     tmr_counter_enable(TMR2, FALSE);
    //     tmr_base_init(TMR2, period, 0U);
    //     tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    //
    //     tmr_output_config_type output_config;
    //     tmr_output_default_para_init(&output_config);
    //     output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    //     output_config.oc_output_state = TRUE;
    //     output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    //     output_config.oc_idle_state = FALSE;
    //     tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &output_config);
    //     tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
    //     tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
    //     tmr_output_enable(TMR2, TRUE);
    //     tmr_counter_enable(TMR2, TRUE);
    //
    //     motion_timer_frequency = step_hz;
    //     motion_timer_pulse_width = pulse_width_us;
    //     motion_timer_running = true;
    // }
    //
    // void stepper_update_motion_timer(uint32_t step_hz, uint32_t pulse_width_us)
    // {
    //     if (!motion_timer_running)
    //     {
    //         stepper_start_motion_timer(step_hz, pulse_width_us);
    //         return;
    //     }
    //     if (step_hz == 0U)
    //     {
    //         stepper_stop_motion_timer();
    //         return;
    //     }
    //     if (step_hz == motion_timer_frequency && pulse_width_us == motion_timer_pulse_width)
    //     {
    //         return;
    //     }
    //
    //     const uint32_t timer_tick_hz = system_core_clock;
    //     uint32_t period = timer_tick_hz / step_hz;
    //     if (period < 2U)
    //     {
    //         period = 2U;
    //     }
    //     --period;
    //     uint32_t pulse_ticks = (timer_tick_hz / 1000000U) * pulse_width_us;
    //     if (pulse_ticks < 1U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     if (pulse_ticks > period)
    //     {
    //         pulse_ticks = period / 2U;
    //     }
    //     if (pulse_ticks == 0U)
    //     {
    //         pulse_ticks = 1U;
    //     }
    //     tmr_period_value_set(TMR2, period);
    //     tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pulse_ticks);
    //     motion_timer_frequency = step_hz;
    //     motion_timer_pulse_width = pulse_width_us;
    // }
    //
    // void stepper_stop_motion_timer(void)
    // {
    //     tmr_counter_enable(TMR2, FALSE);
    //     tmr_output_enable(TMR2, FALSE);
    //     tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, FALSE);
    //     gpio_pin_remap_config(TMR2_MUX_10, FALSE);
    //
    //     gpio_init_type gpio_init_struct;
    //     gpio_default_para_init(&gpio_init_struct);
    //     gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    //     gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    //     gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    //     gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    //     gpio_init_struct.gpio_pins = STEP_OUT_PIN | DIR_OUT_PIN;
    //     gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);
    //     stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, false);
    //     motion_timer_running = false;
    //     motion_timer_frequency = 0U;
    //     motion_timer_pulse_width = 0U;
    // }

    void stepper_stop_motion_timer(void)
    {
        tmr_counter_enable(TMR2, FALSE);
        // MOE 不再关断：一断就把输出级与引脚断开，STEP 变高阻（见 idle_low 注释）
        tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        dma_flag_clear(DMA1_FDT2_FLAG);
        tmr_counter_value_set(TMR2, 0U);
        // 【J1】原来这一行是 stepper_write_gpio_low(...)：STEP 处于 MUX 模式时写 ODR
        // 完全不生效，而 MOE 已关 = 输出级断开，引脚实际留在高阻，驱动器 STEP 悬空吃
        // 干扰。改成切成普通输出真正钉低（最多产生一个下降沿，驱动器只认上升沿）。
        stepper_step_pin_idle_low();
    }

    void stepper_init_step_gpio(void)
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE); // 非常重要，重映射io必须有这句话
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
        gpio_default_para_init(&gpio_init_struct);

        // TMR2 CH3/CH4 are exposed on PB10/PB11 only after this remap is enabled.
        gpio_pin_remap_config(TMR2_MUX_11, TRUE);

        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
#ifdef USE_SOFT_PULSE
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
#else
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
#endif
        gpio_init_struct.gpio_pins = STEP_OUT_PIN;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(STEP_OUTPUT_PORT, &gpio_init_struct);

        // 【DIR = 真 GPIO 推挽 2026-11-12】MUX 模式下写 ODR 不生效，CH4 trick 又
        // 不可靠（见 stepper_init_motion_timer 注释）。PB11 配成普通输出，
        // 方向由 stepper_init_motion_timer / setDirection 直接写引脚。
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = DIR_OUT_PIN;
        gpio_init(DIR_OUTPUT_PORT, &gpio_init_struct);

        gpio_init_struct.gpio_pins = EN_OUT_PIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init(EN_OUTPUT_PORT, &gpio_init_struct);
        stepper_init_motion_timer(k_psc, k_step_pulse_ticks, true);
        stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, true);
        stepper_write_gpio(DIR_OUTPUT_PORT, DIR_OUT_PIN, false);
        stepper_step_pin_idle_low(); // MUX 下写 ODR 不生效，改用定时器钉低
    }

    // 【仅保持 PDN_UART 空闲高电平，不做任何 UART 通讯】
    // 这里刻意用开漏输出而不是推挽：PDN_UART 是双向单线，驱动器和 MCU 共用一根线，
    // 推挽一旦和回包对上就是直接短路。开漏 + 驱动器内部上拉 = 真正的“释放总线”。
    // 全项目没有任何地方会把这个引脚切回推挽或配置 USART，即没有发波能力。
    void stepper_init_uart_gpio(gpio_type* port, uint16_t pin)
    {
        gpio_init_type gpio_init_struct;
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = pin;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(port, &gpio_init_struct);
        stepper_write_gpio(port, pin, true);
    }
}
