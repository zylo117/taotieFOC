/**
 * Minimal GPIO-only proof test.
 *
 * PB10 / STEP: 64Hz pulses, 1us high width.
 * PB11 / DIR: fixed HIGH.
 * PA3 / EN: low active.
 */

#include "at32f403a_407.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

namespace
{
    constexpr uint16_t kStepPin = GPIO_PINS_10;
    constexpr uint16_t kDirPin = GPIO_PINS_11;
    constexpr uint16_t kEnPin = GPIO_PINS_3;
    constexpr uint32_t kPulseWidthUs = 1UL;
    constexpr uint32_t kBurstCount = 64UL;

    void gpio_configuration()
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);

        gpio_init_struct.gpio_pins = kStepPin;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(GPIOB, &gpio_init_struct);

        gpio_init_struct.gpio_pins = kDirPin;
        gpio_init(GPIOB, &gpio_init_struct);

        gpio_init_struct.gpio_pins = kEnPin;
        gpio_init(GPIOA, &gpio_init_struct);

        gpio_bits_reset(GPIOB, kStepPin);
        gpio_bits_set(GPIOB, kDirPin);
        gpio_bits_reset(GPIOA, kEnPin);

        gpio_init_struct.gpio_pins = LED5_PIN;
        gpio_init(GPIOA, &gpio_init_struct);
        gpio_bits_set(GPIOA, LED5_PIN);
    }

    void emit_step_burst(uint32_t count)
    {
        for (uint32_t i = 0U; i < count; ++i)
        {
            gpio_bits_set(GPIOB, kStepPin);
            delay_us(kPulseWidthUs);
            gpio_bits_reset(GPIOB, kStepPin);
            delay_us(kPulseWidthUs);
        }
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_configuration();

    while (1)
    {
        gpio_bits_reset(GPIOA, LED5_PIN);
        delay_ms(100);
        gpio_bits_set(GPIOA, LED5_PIN);
        delay_ms(100);

        emit_step_burst(kBurstCount);
        delay_ms(250);
    }
}
