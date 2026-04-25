/**
 * @file config.h
 * @brief System configuration structure and NVS storage
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  // Rate PID gains (inner loop)
  float roll_kp, roll_ki, roll_kd;
  float pitch_kp, pitch_ki, pitch_kd;
  float yaw_kp, yaw_ki, yaw_kd;

  // Rate loop limits
  float rate_output_limit;
  float rate_integral_limit;

  // Angle mode (outer loop)
  float angle_kp;   // Angle error → target rate
  float angle_ki;   // Steady-state drift correction
  float angle_max;  // Max tilt angle (degrees)

  // Safety
  uint16_t low_bat_threshold;  // Battery voltage in mV
} system_config_t;

extern system_config_t sys_cfg;

void config_load_defaults(void);
void config_save_to_nvs(void);
bool config_load_from_nvs(void);

#endif // CONFIG_H
