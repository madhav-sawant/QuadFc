#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  // Rate PID Gains
  float roll_kp;
  float roll_ki;
  float roll_kd;

  float pitch_kp;
  float pitch_ki;
  float pitch_kd;

  float yaw_kp;
  float yaw_ki;
  float yaw_kd;

  // Limits
  float rate_output_limit;
  float rate_integral_limit;

  // Angle Mode PID (Outer Loop)
  float angle_kp;  // Angle P gain (deg error -> deg/s rate)
  float angle_ki;  // Angle I gain (optional, usually 0)
  float angle_max; // Max allowed angle in degrees (e.g., 45)

  // Safety
  uint16_t low_bat_threshold;
} system_config_t;

// Global configuration instance
extern system_config_t sys_cfg;

// Load defaults into the config structure
void config_load_defaults(void);

// NVS Functions
void config_save_to_nvs(void);
bool config_load_from_nvs(void);

#endif // CONFIG_H
