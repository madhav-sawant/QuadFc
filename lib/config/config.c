/**
 * @file config.c
 * @brief System configuration with NVS persistence
 */

#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_tune_final"

void config_load_defaults(void) {
  // SYSTEMATIC PID TUNING - PROFILE 17 "Golden Tune - DJI Stability"
  // 
  // FLIGHT ANALYSIS (Blackbox 5):
  // - Rate Loop: LOCKED. No oscillation, holds angle perfectly.
  // - Yaw: Strong hold, counters torque.
  // - Angle: Calculated P=3.5 for 70dps correction at max tilt.
  //   I=0.20 adds position hold "memory" to fight steady-state drift (M1).
  
  // BALANCED TUNE (After 3 test flights - smooth AND stable)
  // Log analysis: D=0.005 too weak (45° flip), D=0.008 jerky, D=0.007 = sweet spot
  sys_cfg.roll_kp = 0.40f;   // Smooth response, good correction speed
  sys_cfg.roll_ki = 0.12f;   // Reduced to minimize I-term windup on ground
  sys_cfg.roll_kd = 0.012f;  // Conservative damping increase from data analysis

  sys_cfg.pitch_kp = 0.40f;  // Smooth response, good correction speed  
  sys_cfg.pitch_ki = 0.12f;  // Reduced to minimize I-term windup on ground
  sys_cfg.pitch_kd = 0.012f; // Conservative damping increase from data analysis

  sys_cfg.yaw_kp = 10.0f; // OPTIMIZED: From simulation (63 configs tested, best = 0.48 deg/s error)
  sys_cfg.yaw_ki = 0.5f;  // Balanced integral for real-world drift correction
  sys_cfg.yaw_kd = 0.0f;  // Not needed for yaw (gyro directly measures rate)

  sys_cfg.rate_output_limit = 400.0f;
  sys_cfg.rate_integral_limit = 60.0f;

  // ANGLE MODE
  sys_cfg.angle_kp =  3.20f;
  sys_cfg.angle_ki =  0.20f;
  sys_cfg.angle_max = 20.0f;

  // Safety
  sys_cfg.low_bat_threshold = 10500; 
}

void config_save_to_nvs(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
    return;

  nvs_set_blob(h, "roll_kp", &sys_cfg.roll_kp, sizeof(float));
  nvs_set_blob(h, "roll_ki", &sys_cfg.roll_ki, sizeof(float));
  nvs_set_blob(h, "roll_kd", &sys_cfg.roll_kd, sizeof(float));
  nvs_set_blob(h, "pitch_kp", &sys_cfg.pitch_kp, sizeof(float));
  nvs_set_blob(h, "pitch_ki", &sys_cfg.pitch_ki, sizeof(float));
  nvs_set_blob(h, "pitch_kd", &sys_cfg.pitch_kd, sizeof(float));
  nvs_set_blob(h, "yaw_kp", &sys_cfg.yaw_kp, sizeof(float));
  nvs_set_blob(h, "yaw_ki", &sys_cfg.yaw_ki, sizeof(float));
  nvs_set_blob(h, "yaw_kd", &sys_cfg.yaw_kd, sizeof(float));
  nvs_set_blob(h, "angle_kp", &sys_cfg.angle_kp, sizeof(float));

  nvs_commit(h);
  nvs_close(h);
}

static bool load_float(nvs_handle_t h, const char *key, float *out) {
  size_t len = sizeof(float);
  float val;
  if (nvs_get_blob(h, key, &val, &len) != ESP_OK)
    return false;
  if (val < -1000.0f || val > 1000.0f)
    return false; // Sanity check
  *out = val;
  return true;
}

bool config_load_from_nvs(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
    return false;

  load_float(h, "roll_kp", &sys_cfg.roll_kp);
  load_float(h, "roll_ki", &sys_cfg.roll_ki);
  load_float(h, "roll_kd", &sys_cfg.roll_kd);
  load_float(h, "pitch_kp", &sys_cfg.pitch_kp);
  load_float(h, "pitch_ki", &sys_cfg.pitch_ki);
  load_float(h, "pitch_kd", &sys_cfg.pitch_kd);
  load_float(h, "yaw_kp", &sys_cfg.yaw_kp);
  load_float(h, "yaw_ki", &sys_cfg.yaw_ki);
  load_float(h, "yaw_kd", &sys_cfg.yaw_kd);
  load_float(h, "angle_kp", &sys_cfg.angle_kp);

  nvs_close(h);
  return true;
}
