#include "taotie_closed_loop.h"
#include "at32f403a_407_board.h"

#include <math.h>
#include <string.h>

#define TAOTIE_STEP_EDGE_TIMEOUT_US  200U
#define TAOTIE_TMC2209_UART_BAUD     115200U

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float max_integral;
  float max_output;
} taotie_pid_t;

static taotie_pid_t g_position_pid;
static taotie_pid_t g_velocity_pid;

static volatile int32_t g_target_step = 0;
static volatile int32_t g_actual_step = 0;
static volatile int32_t g_last_actual_step = 0;
static volatile int32_t g_command_step = 0;
static volatile float g_follow_error = 0.0f;
static volatile float g_measured_velocity_rps = 0.0f;
static volatile float g_target_velocity_rps = 0.0f;
static volatile uint32_t g_step_period_us = TAOTIE_STEP_PERIOD_US_DEFAULT;
static volatile uint16_t g_encoder_zero = 0U;
static volatile uint16_t g_last_encoder_raw = 0U;
static volatile uint32_t g_last_process_time_us = 0U;
static volatile uint8_t g_last_step_state = 0U;
static volatile uint8_t g_last_dir_state = 0U;
static volatile uint8_t g_last_en_state = 0U;

static void taotie_gpio_config_input(gpio_type *port, uint16_t pin)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
}

static void taotie_gpio_config_output(gpio_type *port, uint16_t pin)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
}

static void taotie_write_gpio(gpio_type *port, uint16_t pin, bool state)
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

static uint16_t taotie_spi2_rw16(uint16_t tx_data)
{
  while (spi_i2s_flag_get(TAOTIE_KTH7823_SPI, SPI_I2S_TDBE_FLAG) == RESET)
  {
  }
  spi_i2s_data_transmit(TAOTIE_KTH7823_SPI, tx_data);

  while (spi_i2s_flag_get(TAOTIE_KTH7823_SPI, SPI_I2S_RDBF_FLAG) == RESET)
  {
  }

  return spi_i2s_data_receive(TAOTIE_KTH7823_SPI);
}

static void taotie_tmc2209_uart_bit_bang_byte(uint8_t byte)
{
  uint8_t bit_index;
  uint16_t pin = TAOTIE_TMC2209_UART_PIN;
  gpio_type *port = TAOTIE_TMC2209_UART_GPIO;

  taotie_write_gpio(port, pin, 0);
  for (bit_index = 0; bit_index < 8U; ++bit_index)
  {
    delay_us(1000U / (TAOTIE_TMC2209_UART_BAUD / 1000U));
    taotie_write_gpio(port, pin, (byte & 0x01U) != 0U);
    byte >>= 1U;
  }
  delay_us(1000U / (TAOTIE_TMC2209_UART_BAUD / 1000U));
  taotie_write_gpio(port, pin, 1);
}

static float taotie_pid_update(taotie_pid_t *pid, float error, float dt)
{
  float derivative;
  float output;

  if (dt <= 0.0f)
  {
    derivative = 0.0f;
  }
  else
  {
    derivative = (error - pid->last_error) / dt;
  }

  pid->integral += error * dt;
  if (pid->integral > pid->max_integral)
  {
    pid->integral = pid->max_integral;
  }
  else if (pid->integral < -pid->max_integral)
  {
    pid->integral = -pid->max_integral;
  }

  output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  if (output > pid->max_output)
  {
    output = pid->max_output;
  }
  else if (output < -pid->max_output)
  {
    output = -pid->max_output;
  }

  pid->last_error = error;
  return output;
}

static void taotie_kth7823_spi_init(void)
{
  spi_init_type spi_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

  taotie_gpio_config_output(TAOTIE_KTH7823_CS_PORT, TAOTIE_KTH7823_CS_PIN);
  taotie_gpio_config_output(TAOTIE_KTH7823_SCLK_PORT, TAOTIE_KTH7823_SCLK_PIN);
  taotie_gpio_config_output(TAOTIE_KTH7823_MOSI_PORT, TAOTIE_KTH7823_MOSI_PIN);
  taotie_gpio_config_input(TAOTIE_KTH7823_MISO_PORT, TAOTIE_KTH7823_MISO_PIN);

  taotie_gpio_config_input(TAOTIE_KTH7823_MGH_PORT, TAOTIE_KTH7823_MGH_PIN);
  taotie_gpio_config_input(TAOTIE_KTH7823_MGL_PORT, TAOTIE_KTH7823_MGL_PIN);

  taotie_write_gpio(TAOTIE_KTH7823_CS_PORT, TAOTIE_KTH7823_CS_PIN, 1);

  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(TAOTIE_KTH7823_SPI, &spi_init_struct);
  spi_enable(TAOTIE_KTH7823_SPI, TRUE);
}

static void taotie_step_dir_io_init(void)
{
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

  taotie_gpio_config_input(TAOTIE_STEP_INPUT_PORT, TAOTIE_STEP_IN_PIN);
  taotie_gpio_config_input(TAOTIE_DIR_INPUT_PORT, TAOTIE_DIR_IN_PIN);
  taotie_gpio_config_input(TAOTIE_EN_INPUT_PORT, TAOTIE_EN_IN_PIN);

  taotie_gpio_config_output(TAOTIE_STEP_OUTPUT_PORT, TAOTIE_STEP_OUT_PIN);
  taotie_gpio_config_output(TAOTIE_DIR_OUTPUT_PORT, TAOTIE_DIR_OUT_PIN);
  taotie_gpio_config_output(TAOTIE_EN_OUTPUT_PORT, TAOTIE_EN_OUT_PIN);

  taotie_write_gpio(TAOTIE_EN_OUTPUT_PORT, TAOTIE_EN_OUT_PIN, 1);
  taotie_write_gpio(TAOTIE_DIR_OUTPUT_PORT, TAOTIE_DIR_OUT_PIN, 0);
  taotie_write_gpio(TAOTIE_STEP_OUTPUT_PORT, TAOTIE_STEP_OUT_PIN, 0);
}

void taotie_closed_loop_init(void)
{
  g_position_pid.kp = 1.0f;
  g_position_pid.ki = 0.04f;
  g_position_pid.kd = 0.02f;
  g_position_pid.max_integral = TAOTIE_MAX_I_TERM;
  g_position_pid.max_output = TAOTIE_MAX_PID_OUTPUT;
  g_position_pid.integral = 0.0f;
  g_position_pid.last_error = 0.0f;

  g_velocity_pid.kp = 0.7f;
  g_velocity_pid.ki = 0.06f;
  g_velocity_pid.kd = 0.01f;
  g_velocity_pid.max_integral = TAOTIE_MAX_I_TERM;
  g_velocity_pid.max_output = TAOTIE_MAX_PID_OUTPUT;
  g_velocity_pid.integral = 0.0f;
  g_velocity_pid.last_error = 0.0f;

  taotie_step_dir_io_init();
  taotie_kth7823_spi_init();

  g_encoder_zero = taotie_kth7823_read_angle_raw();
  g_last_encoder_raw = g_encoder_zero;
  g_last_process_time_us = 0U;

  taotie_tmc2209_uart_init();
  taotie_tmc2209_write_reg(0x00U, 0x00000000UL);
}

void taotie_closed_loop_set_pid(float kp, float ki, float kd)
{
  g_position_pid.kp = kp;
  g_position_pid.ki = ki;
  g_position_pid.kd = kd;
  g_velocity_pid.kp = kp * 0.5f;
  g_velocity_pid.ki = ki * 0.5f;
  g_velocity_pid.kd = kd * 0.5f;
}

void taotie_closed_loop_set_target_step(int32_t target_step)
{
  g_target_step = target_step;
}

void taotie_closed_loop_set_target_velocity(float rps)
{
  g_target_velocity_rps = rps;
}

int32_t taotie_closed_loop_get_position_steps(void)
{
  return g_actual_step;
}

float taotie_closed_loop_get_follow_error(void)
{
  return g_follow_error;
}

uint16_t taotie_kth7823_read_angle_raw(void)
{
  uint16_t raw = 0U;

  taotie_write_gpio(TAOTIE_KTH7823_CS_PORT, TAOTIE_KTH7823_CS_PIN, 0);
  raw = taotie_spi2_rw16(0x0000U);
  taotie_write_gpio(TAOTIE_KTH7823_CS_PORT, TAOTIE_KTH7823_CS_PIN, 1);

  return raw;
}

void taotie_kth7823_set_zero(uint16_t zero_angle)
{
  g_encoder_zero = zero_angle;
}

bool taotie_tmc2209_uart_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = TAOTIE_TMC2209_UART_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(TAOTIE_TMC2209_UART_GPIO, &gpio_init_struct);

  taotie_write_gpio(TAOTIE_TMC2209_UART_GPIO, TAOTIE_TMC2209_UART_PIN, 1);
  return true;
}

bool taotie_tmc2209_write_reg(uint8_t reg, uint32_t value)
{
  uint32_t frame = ((uint32_t)reg << 16U) | (value & 0xFFFFUL);
  uint8_t index;

  (void)frame;

  for (index = 0U; index < 4U; ++index)
  {
    taotie_tmc2209_uart_bit_bang_byte((uint8_t)((frame >> (index * 8U)) & 0xFFU));
  }

  return true;
}

void taotie_closed_loop_sync_step_dir(void)
{
  uint8_t step_state = gpio_input_data_bit_read(TAOTIE_STEP_INPUT_PORT, TAOTIE_STEP_IN_PIN);
  uint8_t dir_state = gpio_input_data_bit_read(TAOTIE_DIR_INPUT_PORT, TAOTIE_DIR_IN_PIN);
  uint8_t en_state = gpio_input_data_bit_read(TAOTIE_EN_INPUT_PORT, TAOTIE_EN_IN_PIN);

  if (step_state != g_last_step_state)
  {
    if (step_state != 0U)
    {
      g_command_step += (dir_state != 0U) ? 1 : -1;
      g_target_step = g_command_step;
      taotie_write_gpio(TAOTIE_STEP_OUTPUT_PORT, TAOTIE_STEP_OUT_PIN, 1);
      delay_us(TAOTIE_STEP_EDGE_TIMEOUT_US);
      taotie_write_gpio(TAOTIE_STEP_OUTPUT_PORT, TAOTIE_STEP_OUT_PIN, 0);
    }
    g_last_step_state = step_state;
  }

  if (dir_state != g_last_dir_state)
  {
    taotie_write_gpio(TAOTIE_DIR_OUTPUT_PORT, TAOTIE_DIR_OUT_PIN, dir_state);
    g_last_dir_state = dir_state;
  }

  if (en_state != g_last_en_state)
  {
    taotie_write_gpio(TAOTIE_EN_OUTPUT_PORT, TAOTIE_EN_OUT_PIN, en_state);
    g_last_en_state = en_state;
  }
}

void taotie_closed_loop_process(uint32_t time_us)
{
  uint16_t encoder_raw = taotie_kth7823_read_angle_raw();
  int32_t actual_step = 0;
  float position_error;
  float velocity_reference;
  float velocity_error;
  float position_correction;
  float velocity_correction;
  float correction_gain = 0.0f;

  if (g_last_process_time_us == 0U)
  {
    g_last_process_time_us = time_us;
  }

  if (encoder_raw > g_encoder_zero)
  {
    actual_step = (int32_t)(encoder_raw - g_encoder_zero);
  }
  else
  {
    actual_step = -(int32_t)(g_encoder_zero - encoder_raw);
  }

  g_actual_step = actual_step;
  g_follow_error = (float)(g_target_step - g_actual_step);

  if (g_last_process_time_us != 0U && time_us > g_last_process_time_us)
  {
    uint32_t dt_us = time_us - g_last_process_time_us;
    float dt = (float)dt_us * 1.0e-6f;
    if (dt > 0.0f)
    {
      g_measured_velocity_rps = (float)(g_actual_step - g_last_actual_step) / dt / 200.0f;
    }
  }

  position_error = (float)(g_target_step - g_actual_step);
  position_correction = taotie_pid_update(&g_position_pid, position_error, 0.001f);

  velocity_reference = position_correction;
  velocity_error = velocity_reference - g_measured_velocity_rps;
  velocity_correction = taotie_pid_update(&g_velocity_pid, velocity_error, 0.001f);

  correction_gain = velocity_correction / TAOTIE_MAX_PID_OUTPUT;
  correction_gain = fminf(1.0f, fmaxf(-1.0f, correction_gain));

  if (g_step_period_us > 10U)
  {
    uint32_t compensated_step_period = (uint32_t)((float)g_step_period_us * (1.0f - 0.25f * correction_gain));
    if (compensated_step_period < 10U)
    {
      compensated_step_period = 10U;
    }
    g_step_period_us = compensated_step_period;
  }

  if (fabsf(position_error) < 0.25f)
  {
    g_position_pid.integral = 0.0f;
    g_velocity_pid.integral = 0.0f;
  }

  if (fabsf(position_error) < 0.1f)
  {
    g_position_pid.last_error = 0.0f;
    g_velocity_pid.last_error = 0.0f;
  }

  g_last_encoder_raw = encoder_raw;
  g_last_actual_step = g_actual_step;
  g_last_process_time_us = time_us;

  taotie_closed_loop_sync_step_dir();
}
