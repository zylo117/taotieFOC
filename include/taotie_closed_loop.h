#ifndef TAOTIE_CLOSED_LOOP_H
#define TAOTIE_CLOSED_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "at32f403a_407.h"

#define TAOTIE_STEP_INPUT_PORT          GPIOC
#define TAOTIE_STEP_IN_PIN              GPIO_PINS_15
#define TAOTIE_DIR_INPUT_PORT           GPIOC
#define TAOTIE_DIR_IN_PIN               GPIO_PINS_14
#define TAOTIE_EN_INPUT_PORT            GPIOC
#define TAOTIE_EN_IN_PIN                GPIO_PINS_13

#define TAOTIE_STEP_OUTPUT_PORT         GPIOA
#define TAOTIE_STEP_OUT_PIN             GPIO_PINS_15
#define TAOTIE_DIR_OUTPUT_PORT          GPIOA
#define TAOTIE_DIR_OUT_PIN              GPIO_PINS_14
#define TAOTIE_EN_OUTPUT_PORT           GPIOA
#define TAOTIE_EN_OUT_PIN               GPIO_PINS_13

#define TAOTIE_TMC2209_UART_GPIO        GPIOA
#define TAOTIE_TMC2209_UART_PIN         GPIO_PINS_4

#define TAOTIE_KTH7823_SPI              SPI2
#define TAOTIE_KTH7823_SCLK_PORT        GPIOB
#define TAOTIE_KTH7823_SCLK_PIN         GPIO_PINS_11
#define TAOTIE_KTH7823_MISO_PORT        GPIOB
#define TAOTIE_KTH7823_MISO_PIN         GPIO_PINS_2
#define TAOTIE_KTH7823_MOSI_PORT        GPIOB
#define TAOTIE_KTH7823_MOSI_PIN         GPIO_PINS_1
#define TAOTIE_KTH7823_CS_PORT          GPIOB
#define TAOTIE_KTH7823_CS_PIN           GPIO_PINS_10
#define TAOTIE_KTH7823_MGH_PORT         GPIOB
#define TAOTIE_KTH7823_MGH_PIN          GPIO_PINS_13
#define TAOTIE_KTH7823_MGL_PORT         GPIOB
#define TAOTIE_KTH7823_MGL_PIN          GPIO_PINS_12

#define TAOTIE_STEP_PERIOD_US_DEFAULT   5000U
#define TAOTIE_MAX_PID_OUTPUT           2000.0f
#define TAOTIE_MAX_I_TERM               100.0f

void taotie_closed_loop_init(void);
void taotie_closed_loop_sync_step_dir(void);
void taotie_closed_loop_process(uint32_t time_us);
void taotie_closed_loop_set_target_step(int32_t target_step);
void taotie_closed_loop_set_target_velocity(float rps);
void taotie_closed_loop_set_pid(float kp, float ki, float kd);
int32_t taotie_closed_loop_get_position_steps(void);
float taotie_closed_loop_get_follow_error(void);
uint16_t taotie_kth7823_read_angle_raw(void);
void taotie_kth7823_set_zero(uint16_t zero_angle);
bool taotie_tmc2209_write_reg(uint8_t reg, uint32_t value);
bool taotie_tmc2209_uart_init(void);

#ifdef __cplusplus
}
#endif

#endif
