/**
 * @file rate_control.c
 * @brief Rate PID controller for roll, pitch, and yaw (inner loop)
 */

#include "rate_control.h"
#include "../pid/pid.h"

static pid_controller_t pid_roll;
static pid_controller_t pid_pitch;
static pid_controller_t pid_yaw;
static rate_output_t output;

void rate_control_init(void) {
  pid_init(&pid_roll, sys_cfg.roll_kp, sys_cfg.roll_ki, sys_cfg.roll_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  pid_init(&pid_pitch, sys_cfg.pitch_kp, sys_cfg.pitch_ki, sys_cfg.pitch_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  pid_init(&pid_yaw, sys_cfg.yaw_kp, sys_cfg.yaw_ki, sys_cfg.yaw_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  output.roll = 0.0f;
  output.pitch = 0.0f;
  output.yaw = 0.0f;
}

void rate_control_update(float target_roll, float target_pitch,
                         float target_yaw, float gyro_roll, float gyro_pitch,
                         float gyro_yaw, uint16_t throttle) {
  // Smooth PID authority ramp: 0% (≤1100µs) to 100% (≥1300µs)
  float pid_authority = 0.0f;
  if (throttle >= 1300) {
    pid_authority = 1.0f;
  } else if (throttle > 1100) {
    pid_authority = (float)(throttle - 1100) / 200.0f;
  }

  // Below ramp zone: reset everything cleanly
  if (pid_authority <= 0.0f) {
    pid_roll.integral = 0.0f;
    pid_pitch.integral = 0.0f;
    pid_yaw.integral = 0.0f;
    pid_roll.prev_measurement = gyro_roll;
    pid_pitch.prev_measurement = gyro_pitch;
    pid_yaw.prev_measurement = gyro_yaw;
    pid_roll.filtered_derivative = 0.0f;
    pid_pitch.filtered_derivative = 0.0f;
    pid_yaw.filtered_derivative = 0.0f;
    output.roll = 0.0f;
    output.pitch = 0.0f;
    output.yaw = 0.0f;
    return;
  }

  // Calculate raw PID outputs
  float raw_roll =
      pid_calculate(&pid_roll, target_roll, gyro_roll, RATE_LOOP_DT_SEC);
  float raw_pitch =
      pid_calculate(&pid_pitch, target_pitch, gyro_pitch, RATE_LOOP_DT_SEC);
  float raw_yaw = 
      pid_calculate(&pid_yaw, target_yaw, gyro_yaw, RATE_LOOP_DT_SEC);

  // Apply authority ramp to outputs
  output.roll  = raw_roll  * pid_authority;
  output.pitch = raw_pitch * pid_authority;

  // Yaw uses same authority ramp as roll/pitch.
  // Mixer limits yaw headroom to 50% to reserve motor authority.
  output.yaw = raw_yaw * pid_authority;
}

const rate_output_t *rate_control_get_output(void) { return &output; }
