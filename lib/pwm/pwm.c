#include "pwm.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES ((ledc_timer_bit_t)PWM_RES_BIT)
#define LEDC_FREQUENCY (PWM_FREQ_HZ)

static const int motor_gpios[PWM_MOTOR_COUNT] = {
    PWM_MOTOR_1_GPIO, PWM_MOTOR_2_GPIO, PWM_MOTOR_3_GPIO, PWM_MOTOR_4_GPIO};
static const ledc_channel_t motor_channels[PWM_MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

void pwm_init(void) {
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  for (int i = 0; i < PWM_MOTOR_COUNT; i++) {
    ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                          .channel = motor_channels[i],
                                          .timer_sel = LEDC_TIMER,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .gpio_num = motor_gpios[i],
                                          .duty = 0,
                                          .hpoint = 0};
    ledc_channel_config(&ledc_channel);
    pwm_set_motor(i, PWM_MIN_PULSE_US);
  }
}

void pwm_set_motor(int motor_index, uint32_t pulse_width_us) {
  if (motor_index < 0 || motor_index >= PWM_MOTOR_COUNT)
    return;
  if (pulse_width_us < PWM_MIN_PULSE_US)
    pulse_width_us = PWM_MIN_PULSE_US;
  else if (pulse_width_us > PWM_MAX_PULSE_US)
    pulse_width_us = PWM_MAX_PULSE_US;

  uint32_t max_duty = (1 << PWM_RES_BIT) - 1;
  uint32_t duty = (uint32_t)(((uint64_t)pulse_width_us * (uint64_t)max_duty *
                              (uint64_t)PWM_FREQ_HZ) /
                             1000000ULL);

  ledc_set_duty(LEDC_MODE, motor_channels[motor_index], duty);
  ledc_update_duty(LEDC_MODE, motor_channels[motor_index]);
}
