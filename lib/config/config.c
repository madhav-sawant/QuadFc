/**
 * @file config.c
 * @brief System configuration with NVS persistence
 */

#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_v3"  // BB100 data-driven tune

void config_load_defaults(void) {
  // -----------------------------------------------------------------------
  // RATE PID (inner loop)
  // -----------------------------------------------------------------------
  // Roll/Pitch: tuned to damp ~7.5Hz oscillation and reduce windup.
  sys_cfg.roll_kp  = 0.30f;
  sys_cfg.roll_ki  = 0.08f;
  sys_cfg.roll_kd  = 0.016f;

  sys_cfg.pitch_kp = 0.30f;
  sys_cfg.pitch_ki = 0.08f;
  sys_cfg.pitch_kd = 0.016f;

  // Yaw: Kp and Ki lowered to reduce vibration bias impact.
  // Prevents yaw from dominating motor authority during flight.
  sys_cfg.yaw_kp   = 2.5f;
  sys_cfg.yaw_ki   = 0.15f;
  sys_cfg.yaw_kd   = 0.0f;

  sys_cfg.rate_output_limit   = 400.0f;
  sys_cfg.rate_integral_limit =  50.0f;  // tightened from 60 → less windup headroom

  // -----------------------------------------------------------------------
  // ANGLE PID (outer loop)
  // -----------------------------------------------------------------------
  // Lowered Kp and Ki for smoother angle-to-rate conversion.
  sys_cfg.angle_kp  = 3.5f;
  sys_cfg.angle_ki  = 0.15f;
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
  nvs_set_blob(h, "angle_ki", &sys_cfg.angle_ki, sizeof(float));

  nvs_commit(h);
  nvs_close(h);
}

static bool load_float(nvs_handle_t h, const char *key, float *out) {
  size_t len = sizeof(float);
  float val;
  if (nvs_get_blob(h, key, &val, &len) != ESP_OK)
    return false;
  if (val < -1000.0f || val > 1000.0f)
    return false;  // Sanity check
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
  load_float(h, "angle_ki", &sys_cfg.angle_ki);

  nvs_close(h);
  return true;
}
