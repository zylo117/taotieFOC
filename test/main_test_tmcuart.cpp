/**
 * TMC2209 单线 UART（PDN_UART）通信测试 —— 纯 GPIO 软件位时序版
 *
 * 硬件连接（AT32F403A @ 250MHz）
 *   PA4  <────>  驱动模块 PDN_UART  单线半双工，MCU 与驱动直连即可
 *                                   （驱动器 PDN_UART 内部已有上拉，不需要外部上拉/串阻）
 *   PA9  ──TX──▶ USB-TTL            板载 PRINT_UART，115200 8N1，用于 printf 输出
 *
 * 时序原理
 *   DWT->CYCCNT 作为位定时基准：115200bps 时 1bit = 250MHz / 115200 ≈ 2170 个 CPU 周期
 *   TX：PA4 切推挽输出，用绝对时刻（t0 + n * 1bit）翻转电平，多位发送误差不累积
 *   RX：PA4 切输入（内部上拉），等起始位下降沿 → 1.5bit 后采样第一位 → 之后每 1bit 采一位
 *
 * TMC 数据报（同步字节 0x05，从机地址 0x00 = MS1/MS2 接地，应答地址 0xFF）
 *   写：05 00 (REG|0x80) D3 D2 D1 D0 CRC   共 8 字节
 *   读：05 00 REG CRC                      共 4 字节
 *   答：05 FF REG D3 D2 D1 D0 CRC          共 8 字节
 */
#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

#include <stdio.h>
#include <string.h>

namespace
{
    /* ---------------- 硬件连接 ---------------- */
    gpio_type *const k_uart_port = GPIOA;
    constexpr uint16_t k_uart_pin = GPIO_PINS_4;

    /* ---------------- 通信参数 ---------------- */
    constexpr uint32_t k_uart_baudrate = 115200UL;

    /* ---------------- TMC 协议常量 ---------------- */
    constexpr uint8_t k_sync_byte = 0x05U;      /* 主机与从机数据报的同步字节 */
    constexpr uint8_t k_slave_address = 0x00U;  /* MS1 = MS2 = GND */
    constexpr uint8_t k_master_address = 0xFFU; /* 驱动器应答时填的主机地址 */
    constexpr uint8_t k_write_flag = 0x80U;

    constexpr uint8_t k_reg_gconf = 0x00U;
    constexpr uint8_t k_reg_ifcnt = 0x02U;
    constexpr uint8_t k_reg_ioin = 0x06U;
    constexpr uint8_t k_reg_ihold_irun = 0x10U;
    constexpr uint8_t k_reg_tpowerdown = 0x11U;
    constexpr uint8_t k_reg_chopconf = 0x6CU;
    constexpr uint8_t k_reg_drv_status = 0x6FU;

    constexpr uint32_t k_read_error = 0xFFFFFFFFUL;
    constexpr uint8_t k_read_retry = 3U;

    /* 一个位的 CPU 周期数 */
    uint32_t g_bit_cycles = 0U;

    /* 数据报之间必须留够总线空闲：实测只留 8bit 时驱动器还停在发送态，
     * 会把紧接着的新请求整个丢掉（表现为 no reply），这里等连续静默 64bit 再发 */
    constexpr uint32_t k_bus_idle_bits = 64U;
    constexpr uint32_t k_bus_idle_timeout_bits = 600U;

    /* 是否打印应答原始字节：启动阶段打开便于查错，周期回读时关掉避免刷屏 */
    bool g_dump_raw = true;

    /* ---------------- 时间基准：DWT 周期计数器 ---------------- */
    inline void dwt_init()
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    inline uint32_t cycle_now()
    {
        return DWT->CYCCNT;
    }

    /* 阻塞到绝对时刻；deadline 采用 32 位回绕比较，只要等待时间 < 2^31 周期就安全 */
    inline void wait_until(uint32_t deadline)
    {
        while (static_cast<int32_t>(cycle_now() - deadline) < 0)
        {
        }
    }

    inline bool line_is_high();

    /* 等总线真正安静下来：连续 bits 个位时间都没有低电平（含驱动器自己做上拉释放的时间） */
    bool bus_wait_idle()
    {
        const uint32_t quiet_cycles = k_bus_idle_bits * g_bit_cycles;
        const uint32_t timeout_cycles = k_bus_idle_timeout_bits * g_bit_cycles;
        const uint32_t start = cycle_now();
        uint32_t high_since = start;

        while (static_cast<uint32_t>(cycle_now() - high_since) < quiet_cycles)
        {
            if (!line_is_high())
            {
                high_since = cycle_now();
            }
            if (static_cast<uint32_t>(cycle_now() - start) > timeout_cycles)
            {
                return false;    /* 线一直拉不下来或一直被拉低，报错让上层重试 */
            }
        }
        return true;
    }

    /* ---------------- 单线总线收发方向切换 ---------------- */
    inline bool line_is_high()
    {
        return gpio_input_data_bit_read(k_uart_port, k_uart_pin) != RESET;
    }

    inline void line_low()
    {
        gpio_bits_reset(k_uart_port, k_uart_pin);
    }

    inline void line_high()
    {
        gpio_bits_set(k_uart_port, k_uart_pin);
    }

    /* 空闲 / 接收态：引脚作输入并开内部上拉，总线交给驱动器 */
    void line_to_receive()
    {
        gpio_init_type gpio_init_struct;

        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_pins = k_uart_pin;
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_UP;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(k_uart_port, &gpio_init_struct);
    }

    /* 发送态：先置高输出锁存，再切推挽输出，避免切换瞬间拉出一个低电平毛刺 */
    void line_to_transmit()
    {
        gpio_init_type gpio_init_struct;

        line_high();
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_pins = k_uart_pin;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(k_uart_port, &gpio_init_struct);
    }

    /* ---------------- TMC CRC ---------------- */
    /* 多项式 x^8 + x^2 + x + 1，字节按 LSB 先处理 */
    uint8_t tmc_crc(const uint8_t *data, uint8_t length)
    {
        uint8_t crc = 0U;

        for (uint8_t index = 0U; index < length; ++index)
        {
            uint8_t current = data[index];
            for (uint8_t bit = 0U; bit < 8U; ++bit)
            {
                const bool mix = ((crc >> 7U) ^ (current & 0x01U)) != 0U;
                crc = static_cast<uint8_t>(crc << 1U);
                if (mix)
                {
                    crc ^= 0x07U;
                }
                current >>= 1U;
            }
        }
        return crc;
    }

    /* ---------------- 软件 UART 收发一个字节（8N1，LSB 先发） ---------------- */
    void uart_write_byte(uint8_t value)
    {
        const uint32_t bit = g_bit_cycles;

        __disable_irq();

        line_low();                                     /* 起始位 */
        const uint32_t t0 = cycle_now();

        for (uint8_t index = 0U; index < 8U; ++index)   /* 数据位，绝对时刻切换 */
        {
            wait_until(t0 + static_cast<uint32_t>(index + 1U) * bit);
            if ((value & (1U << index)) != 0U)
            {
                line_high();
            }
            else
            {
                line_low();
            }
        }

        wait_until(t0 + 9U * bit);
        line_high();                                    /* 停止位 */
        wait_until(t0 + 10U * bit);                     /* 等停止位结束再交还总线 */

        __enable_irq();
    }

    bool uart_read_byte(uint8_t *value)
    {
        const uint32_t bit = g_bit_cycles;
        const uint32_t timeout = cycle_now() + 200U * bit;   /* 约 1.7ms 等不到起始位就算超时 */

        __disable_irq();

        while (line_is_high())                          /* 等起始位的下降沿 */
        {
            if (static_cast<int32_t>(cycle_now() - timeout) >= 0)
            {
                __enable_irq();
                return false;
            }
        }

        const uint32_t t0 = cycle_now();
        uint32_t sample_at = t0 + bit + bit / 2U;        /* 起始位中点之后 1bit，即 1.5bit 处 */
        uint8_t received = 0U;

        for (uint8_t index = 0U; index < 8U; ++index)
        {
            wait_until(sample_at);
            if (line_is_high())
            {
                received |= static_cast<uint8_t>(1U << index);
            }
            sample_at += bit;
        }

        wait_until(t0 + 9U * bit + bit / 2U);            /* 停止位中点 */
        const bool stop_ok = line_is_high();

        __enable_irq();

        *value = received;
        return stop_ok;
    }

    /* ---------------- TMC 寄存器读写 ---------------- */
    void tmc_write_register(uint8_t reg, uint32_t value)
    {
        uint8_t frame[8];

        frame[0] = k_sync_byte;
        frame[1] = k_slave_address;
        frame[2] = static_cast<uint8_t>(reg | k_write_flag);
        frame[3] = static_cast<uint8_t>(value >> 24U);
        frame[4] = static_cast<uint8_t>(value >> 16U);
        frame[5] = static_cast<uint8_t>(value >> 8U);
        frame[6] = static_cast<uint8_t>(value);
        frame[7] = tmc_crc(frame, 7U);

        bus_wait_idle();
        line_to_transmit();
        for (uint8_t index = 0U; index < 8U; ++index)
        {
            uart_write_byte(frame[index]);
        }
        line_to_receive();                              /* 写报文不需要应答，直接放总线 */
    }

    uint32_t tmc_read_register(uint8_t reg)
    {
        uint8_t request[4];

        request[0] = k_sync_byte;
        request[1] = k_slave_address;
        request[2] = static_cast<uint8_t>(reg & 0x7FU);
        request[3] = tmc_crc(request, 3U);

        for (uint8_t attempt = 1U; attempt <= k_read_retry; ++attempt)
        {
            bus_wait_idle();
            line_to_transmit();
            for (uint8_t index = 0U; index < 4U; ++index)
            {
                uart_write_byte(request[index]);
            }
            line_to_receive();                          /* 同一根线，发完立刻切接收等应答 */

            uint8_t response[8] = {0};
            bool framing_ok = true;
            for (uint8_t index = 0U; index < 8U; ++index)
            {
                if (!uart_read_byte(&response[index]))
                {
                    framing_ok = false;
                    break;
                }
            }

            bus_wait_idle();

            if (!framing_ok)
            {
                printf("[TMC] reg 0x%02X: no reply (attempt %u)\r\n",
                       reg, static_cast<unsigned>(attempt));
                continue;
            }

            if (g_dump_raw)
            {
                printf("[TMC] reg 0x%02X raw: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                       reg, response[0], response[1], response[2], response[3],
                       response[4], response[5], response[6], response[7]);
            }

            if (response[0] != k_sync_byte || response[1] != k_master_address ||
                response[7] != tmc_crc(response, 7U))
            {
                printf("[TMC] reg 0x%02X: bad reply (sync/address/crc mismatch)\r\n", reg);
                continue;
            }

            return (static_cast<uint32_t>(response[3]) << 24U) |
                   (static_cast<uint32_t>(response[4]) << 16U) |
                   (static_cast<uint32_t>(response[5]) << 8U) |
                    static_cast<uint32_t>(response[6]);
        }

        return k_read_error;
    }

    /* ---------------- DRV_STATUS 位解析 ---------------- */
    void tmc_report_status()
    {
        static const struct
        {
            uint8_t bit;
            const char *text;
        } flags[] = {
            { 0,  "[WARN]  OTPW  : overtemperature pre-warning\r\n" },
            { 1,  "[ERROR] OT    : overtemperature, driver shut down\r\n" },
            { 2,  "[ERROR] S2GA  : phase A shorted to ground\r\n" },
            { 3,  "[ERROR] S2GB  : phase B shorted to ground\r\n" },
            { 4,  "[ERROR] S2VSA : phase A shorted to supply\r\n" },
            { 5,  "[ERROR] S2VSB : phase B shorted to supply\r\n" },
            { 6,  "[WARN]  OLA   : phase A open load\r\n" },
            { 7,  "[WARN]  OLB   : phase B open load\r\n" },
            { 8,  "[INFO]  T120  : temperature above 120 C\r\n" },
            { 9,  "[INFO]  T143  : temperature above 143 C\r\n" },
            { 10, "[INFO]  T150  : temperature above 150 C\r\n" },
            { 11, "[INFO]  T157  : temperature above 157 C\r\n" },
            { 30, "[INFO]  STEALTH: running in StealthChop\r\n" },
            { 31, "[INFO]  STST  : motor at standstill\r\n" },
        };
        constexpr uint32_t k_fault_mask = 0x000000FEUL;    /* OT / 短路 / 开路 */

        const uint32_t drv_status = tmc_read_register(k_reg_drv_status);
        if (drv_status == k_read_error)
        {
            return;
        }

        printf("\r\n=== DRV_STATUS: 0x%08lX ===\r\n", static_cast<unsigned long>(drv_status));

        for (uint32_t index = 0U; index < sizeof(flags) / sizeof(flags[0]); ++index)
        {
            if ((drv_status & (1UL << flags[index].bit)) != 0U)
            {
                printf("%s", flags[index].text);
            }
        }

        /* CS_ACTUAL：驱动器当前实际施加的电流档位 0..31 */
        printf("CS_ACTUAL     : %lu / 31\r\n",
               static_cast<unsigned long>((drv_status >> 16U) & 0x1FUL));

        if ((drv_status & k_fault_mask) == 0U)
        {
            printf("[OK] No faults\r\n");
        }
        printf("========================\r\n");
    }

    /* ---------------- 初始化 ---------------- */
    void tmc_uart_init()
    {
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

        dwt_init();

        g_bit_cycles = system_core_clock / k_uart_baudrate;
        if (g_bit_cycles == 0U)
        {
            g_bit_cycles = 1U;
        }

        /* 上电默认：引脚作输入，总线空闲高电平，不占用驱动器 */
        line_to_receive();
        line_high();
        delay_ms(2);
    }

    void print_register(const char *name, uint8_t reg)
    {
        const uint32_t value = tmc_read_register(reg);
        if (value == k_read_error)
        {
            printf("%s (0x%02X) read failed\r\n", name, reg);
            return;
        }
        printf("%s (0x%02X) = 0x%08lX\r\n", name, reg, static_cast<unsigned long>(value));
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    uart_print_init(115200);

    tmc_uart_init();

    printf("\r\n===== TMC single wire UART test (PA4 bit-bang) =====\r\n");
    printf("core clock  = %lu Hz\r\n", static_cast<unsigned long>(system_core_clock));
    printf("baudrate    = %lu bps, 1 bit = %lu cycles\r\n",
           static_cast<unsigned long>(k_uart_baudrate),
           static_cast<unsigned long>(g_bit_cycles));

    delay_ms(500);    /* 等驱动器供电稳定，内部上电复位完成 */

    /* 1) 链路自检：IOIN 能读回来就说明单线 UART 通了（VERSION 位应等于 0x21） */
    print_register("IOIN", k_reg_ioin);

    const uint32_t ifcnt_before = tmc_read_register(k_reg_ifcnt);

    /* 2) 基础配置
     *    GCONF    : pdn_disable = 1（PDN 脚当 UART 用）、mstep_reg_select = 1（细分由寄存器决定）
     *    CHOPCONF : MRES = 4（16 细分）、intpol = 1、TOFF = 3、HSTRT = 5
     *    IHOLD_IRUN: IRUN = 18/31、IHOLD = 5/31、IHOLDDELAY = 6
     *    TPOWERDOWN: 电机停转约 0.4s 后降到保持电流 */
    tmc_write_register(k_reg_gconf, (1UL << 6U) | (1UL << 7U));
    printf("GCONF written\r\n");

    tmc_write_register(k_reg_chopconf, (0x04UL << 24U) | 0x10000053UL);
    printf("CHOPCONF written\r\n");

    tmc_write_register(k_reg_ihold_irun, (6UL << 16U) | (18UL << 8U) | 5UL);
    printf("IHOLD_IRUN written: IRUN=18 IHOLD=5\r\n");

    tmc_write_register(k_reg_tpowerdown, 20UL);
    printf("TPOWERDOWN written\r\n");

    /* 3) 用 IFCNT 校验写报文是否被驱动器收下：每成功写一次 +1，共写 4 次 */
    delay_ms(10);
    const uint32_t ifcnt_after = tmc_read_register(k_reg_ifcnt);
    if (ifcnt_before != k_read_error && ifcnt_after != k_read_error)
    {
        printf("IFCNT: %lu -> %lu (delta = %lu, expected 4)\r\n",
               static_cast<unsigned long>(ifcnt_before & 0xFFU),
               static_cast<unsigned long>(ifcnt_after & 0xFFU),
               static_cast<unsigned long>((ifcnt_after - ifcnt_before) & 0xFFU));
    }

    /* 4) 回读确认配置真的落到了寄存器里
     *    注意：这块驱动模块回读 IHOLD_IRUN(0x10) 恒为 0（多次复位、多周期回读均如此），
     *    但 DRV_STATUS 的 CS_ACTUAL 恰好等于写入的 IHOLD(5)，说明写确实生效了，
     *    属于该芯片对 0x10 读取不回显的固有行为，不能据此判定写失败。
     *    要校验写是否生效，看 IFCNT 增量和 DRV_STATUS 的 CS_ACTUAL 更可靠。 */
    print_register("GCONF", k_reg_gconf);
    print_register("CHOPCONF", k_reg_chopconf);
    print_register("IHOLD_IRUN", k_reg_ihold_irun);

    printf("TMC2209 ready\r\n");

    /* 5) 周期性回读配置 + 状态，同时观察 IHOLD_IRUN 这类写寄存器回读是否稳定 */
    uint32_t cycle = 0U;
    while (1)
    {
        ++cycle;

        g_dump_raw = false;                                 /* 周期回读不打印原始字节 */
        const uint32_t gconf = tmc_read_register(k_reg_gconf);
        const uint32_t chopconf = tmc_read_register(k_reg_chopconf);
        const uint32_t ihold_irun = tmc_read_register(k_reg_ihold_irun);
        const uint32_t ifcnt = tmc_read_register(k_reg_ifcnt);
        g_dump_raw = true;

        printf("\r\n-- #%lu GCONF=0x%08lX CHOPCONF=0x%08lX IHOLD_IRUN=0x%08lX IFCNT=%lu\r\n",
               static_cast<unsigned long>(cycle),
               static_cast<unsigned long>(gconf),
               static_cast<unsigned long>(chopconf),
               static_cast<unsigned long>(ihold_irun),
               static_cast<unsigned long>(ifcnt & 0xFFU));

        tmc_report_status();
        delay_ms(1000);
    }
}
