#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include <stdio.h>
#include <string.h>

#define ENABLE_UART_DEBUG   1
#define MAIN_POLL_SPI_TEST  0

//==================== 编码器硬件引脚定义 ====================
#define KTH7823_SPI              SPI2
#define KTH7823_SCLK_PORT        GPIOB
#define KTH7823_SCLK_PIN         GPIO_PINS_13
#define KTH7823_MISO_PORT        GPIOB
#define KTH7823_MISO_PIN         GPIO_PINS_14
#define KTH7823_MOSI_PORT        GPIOB
#define KTH7823_MOSI_PIN         GPIO_PINS_15
#define KTH7823_CS_PORT          GPIOB
#define KTH7823_CS_PIN           GPIO_PINS_12

constexpr uint32_t ENCODER_SAMPLE_US = 10U;
constexpr uint32_t TMR3_MAX_16BIT_TICK = 65535U;
constexpr uint32_t tick_per_us = 125U;  // 1us包含多少个tick
constexpr uint32_t arr_val = (ENCODER_SAMPLE_US * tick_per_us) - 1U;
static_assert(arr_val <= TMR3_MAX_16BIT_TICK, "TMR3 is 16bit! sample_us too large, overflow!");

constexpr uint32_t ENC_RING_BUF_SIZE = 64U;
constexpr uint32_t SPI_WAIT_TIMEOUT_TICK = 10000U;

namespace
{
#if ENABLE_UART_DEBUG
void uart1_debug_init(void);
void uart_send_str(const char *str);
void uart_print_num(uint32_t val);
void uart_print_float(float f);
#endif

volatile uint16_t enc_ring_buf[ENC_RING_BUF_SIZE];
volatile uint32_t enc_head;
volatile uint32_t enc_tail;

volatile uint32_t g_systick_ms = 0U;
volatile uint32_t g_tmr3_irq_cnt = 0U;

// 你的统计变量，对应Kth7823Encoder类成员
volatile uint32_t read_count_ = 0U;
volatile uint32_t all_ones_count_ = 0U;
volatile uint32_t all_zeros_count_ = 0U;

/**
 * @brief CS GPIO控制
*/
void encoder_write_gpio(gpio_type* port, uint16_t pin, bool level)
{
    if(level)
        gpio_bits_set(port, pin);
    else
        gpio_bits_reset(port, pin);
}

/**
 * @brief SPI16bit读写，带超时保护
 * @retval 0xFFFF 表示超时错误
*/
    uint16_t encoder_spi2_rw16(uint16_t tx_data)
{
    constexpr uint32_t spi_timeout = 100000U;
    uint32_t timeout = spi_timeout;
    while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_TDBE_FLAG) == RESET && timeout > 0U)
    {
        --timeout;
    }
    if (timeout == 0U)
    {
        return 0xFFFFU;
    }
    spi_i2s_data_transmit(KTH7823_SPI, tx_data);

    timeout = spi_timeout;
    while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_RDBF_FLAG) == RESET && timeout > 0U)
    {
        --timeout;
    }
    if (timeout == 0U)
    {
        return 0xFFFFU;
    }
    return static_cast<uint16_t>(spi_i2s_data_receive(KTH7823_SPI));
}

/**
 * @brief 完全复刻你 Kth7823Encoder::readRawAngle() 的逻辑，ISR调用版本
*/
uint16_t readRawAngle_ISR(void)
{
    uint16_t raw = 0U;
    uint16_t last_tx_frame_ = 0x0000U;
    encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, false);
    raw = encoder_spi2_rw16(last_tx_frame_);
    encoder_write_gpio(KTH7823_CS_PORT, KTH7823_CS_PIN, true);

    read_count_++;
    if (raw == 0xFFFFU)
    {
        all_ones_count_++;
    }
    else if (raw == 0x0000U)
    {
        all_zeros_count_++;
    }
    return raw;
}

void spi2_encoder_config(void)
{
    gpio_init_type gpio_conf;
    gpio_default_para_init(&gpio_conf);

    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

    gpio_conf.gpio_pins = KTH7823_CS_PIN;
    gpio_conf.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_conf.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_conf.gpio_pull = GPIO_PULL_NONE;
    gpio_conf.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(KTH7823_CS_PORT, &gpio_conf);
    gpio_bits_set(KTH7823_CS_PORT, KTH7823_CS_PIN);

    gpio_conf.gpio_pins = KTH7823_SCLK_PIN | KTH7823_MISO_PIN | KTH7823_MOSI_PIN;
    gpio_conf.gpio_mode = GPIO_MODE_MUX;
    gpio_conf.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_conf.gpio_pull = GPIO_PULL_NONE;
    gpio_conf.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(KTH7823_SCLK_PORT, &gpio_conf);

    spi_i2s_reset(KTH7823_SPI);
    spi_init_type spi_conf;
    spi_default_para_init(&spi_conf);

    spi_conf.transmission_mode     = SPI_TRANSMIT_FULL_DUPLEX;
    spi_conf.master_slave_mode     = SPI_MODE_MASTER;
    spi_conf.clock_polarity        = SPI_CLOCK_POLARITY_HIGH;   // spi mode3，CPOL=1
    spi_conf.clock_phase           = SPI_CLOCK_PHASE_2EDGE;
    spi_conf.frame_bit_num         = SPI_FRAME_16BIT;
    spi_conf.cs_mode_selection     = SPI_CS_SOFTWARE_MODE;
    spi_conf.mclk_freq_division    = SPI_MCLK_DIV_16;  // 主频低就开高，否则数据异常（断一半，数据只有180-360度）,8分频就偶尔异常了
    spi_conf.first_bit_transmission= SPI_FIRST_BIT_MSB;

    spi_init(KTH7823_SPI, &spi_conf);
    spi_i2s_dma_transmitter_enable(KTH7823_SPI, FALSE);
    spi_i2s_dma_receiver_enable(KTH7823_SPI, FALSE);
    spi_enable(KTH7823_SPI, TRUE);
}


void tmr3_encoder_sample_init(void)
{
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE); //补上TMR3时钟
    tmr_base_init(TMR3, static_cast<uint16_t>(arr_val), 0U);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(TMR3, TRUE);

    tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR3_GLOBAL_IRQn, 2U, 0U);

    tmr_counter_value_set(TMR3, 0U);
    tmr_counter_enable(TMR3, TRUE);
}

    void enc_ring_push(uint16_t val)
{
    uint32_t next = (enc_head + 1U) % ENC_RING_BUF_SIZE;
    if(next != enc_tail)
    {
        enc_ring_buf[enc_head] = val;
        enc_head = next;
    }
    else
    {
        //缓冲区已满：丢弃最老数据，写入新样本，保证保存最新值
        enc_ring_buf[enc_head] = val;
        enc_head = next;
        enc_tail = (enc_tail + 1U) % ENC_RING_BUF_SIZE;
    }
}


bool enc_ring_pop(uint16_t *out_val)
{
    if(enc_head == enc_tail)
        return false;
    *out_val = enc_ring_buf[enc_tail];
    enc_tail = (enc_tail + 1U) % ENC_RING_BUF_SIZE;
    return true;
}

#if ENABLE_UART_DEBUG
void uart1_debug_init()
{
    gpio_init_type gpio_init_struct;
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);

    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_transmitter_enable(USART1, TRUE);
    usart_enable(USART1, TRUE);
}

void uart_send_str(const char *str)
{
    while(*str)
    {
        while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
        usart_data_transmit(USART1, static_cast<uint16_t>(*str++));
    }
}

void uart_print_num(uint32_t val)
{
    char buf[32];
    sprintf(buf, "%04X ", (unsigned int)val);
    uart_send_str(buf);
}

/**
 * 简易打印float，用于输出角度
*/
void uart_print_float(float f)
{
    char buf[40];
    sprintf(buf, "%.2f", f);
    uart_send_str(buf);
}
#endif
}

extern "C" void SysTick_Handler(void)
{
    g_systick_ms++;
}

extern "C" void TMR3_GLOBAL_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR3, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR3, TMR_OVF_FLAG);
        g_tmr3_irq_cnt++;
        uint16_t raw = readRawAngle_ISR(); //完全复用你的读取逻辑，包含错误统计
        enc_ring_push(raw);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    SysTick_Config(250000U);

    spi2_encoder_config();
#if (MAIN_POLL_SPI_TEST == 0)
    tmr3_encoder_sample_init();
#endif

#if ENABLE_UART_DEBUG
    uart1_debug_init();
    uart_send_str("\r\nKTH7823 start, raw*360/65536 decode angle\r\n");
#endif

    uint32_t last_print_ms = g_systick_ms;
    uint32_t old_irq_cnt = 0U;

    while (1)
    {
        if((g_systick_ms - last_print_ms) >= 1000U)
        {
            last_print_ms = g_systick_ms;
            uint32_t irq_delta = g_tmr3_irq_cnt - old_irq_cnt;
            old_irq_cnt = g_tmr3_irq_cnt;

#if ENABLE_UART_DEBUG
            uart_send_str("\r\n[stat] irq_delta:");
            uart_print_num(irq_delta);
            uart_send_str(" total_read:");
            uart_print_num(read_count_);
            uart_send_str(" all_ones(0xFFFF):");
            uart_print_num(all_ones_count_);
            uart_send_str(" all_zero(0x0000):");
            uart_print_num(all_zeros_count_);

            uart_send_str("\r\nLast 5 sample -> raw(hex) | angle(deg):\r\n");
#endif
            uint16_t temp_buf[5];
            uint32_t cnt =0;
            while(cnt <5U)
            {
                uint16_t v;
                if(enc_ring_pop(&v))
                {
                    temp_buf[cnt++] = v;
                }else{
                    break;
                }
            }
            for(uint32_t i=0;i<cnt;i++)
            {
                uint16_t raw = temp_buf[i];
                float angle_deg = static_cast<int32_t>(raw) * 360.0f / 65536.0f; //你的解码公式
#if ENABLE_UART_DEBUG
                uart_print_num(raw);
                uart_send_str(" | ");
                uart_print_float(angle_deg);
                uart_send_str(" deg\r\n");
#endif
            }
        }

#if (MAIN_POLL_SPI_TEST ==1)
        delay_ms(200);
        uint16_t v = readRawAngle_ISR();
        float angle_deg = static_cast<float>(v) * 360.0f / 65536.0f;
#if ENABLE_UART_DEBUG
        uart_send_str("Poll raw:");
        uart_print_num(v);
        uart_send_str(" | angle:");
        uart_print_float(angle_deg);
        uart_send_str(" deg\r\n");
#endif
#endif
    }
}
