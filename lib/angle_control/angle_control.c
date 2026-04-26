/**
 * @file angle_control.c
 * @brief Angle PI controller (outer loop)
 *
 * Converts angle error to target rate for inner PID loop.
 * I-term accumulates only when airborne.
 */

#include "angle_control.h"
#include "../config/config.h"

static float i_roll_accum = 0.0f;
static float i_pitch_accum = 0.0f;
static angle_output_t output;

#define MAX_ANGLE_I 30.0f            // Max I-term contribution (deg/s)
#define MAX_ANGLE_RATE 250.0f        // Max output rate command (deg/s)
#define ANGLE_I_THROTTLE_MIN 1200    // I-term minimum throttle

void angle_control_init(void) {
  i_roll_accum = 0.0f;
  i_pitch_accum = 0.0f;
  output.target_roll_rate = 0.0f;
  output.target_pitch_rate = 0.0f;
}

void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle) {
  float raw_roll_error = target_roll - actual_roll;
float raw_pitch_error = target_pitch - actual_pitch;

  // Smooth throttle authority ramp (1100-1300µs)
  // Fades corrections to zero at idle to avoid steps.
  float throttle_factor = 0.0f;
  if (throttle >= 1300) {
    throttle_factor = 1.0f;
  } else if (throttle > 1100) {
    throttle_factor = (float)(throttle - 1100) / 200.0f;
  }

  // Scale errors smoothly without hard deadband
 float roll_error  = raw_roll_error  * throttle_factor;
float pitch_error = raw_pitch_error * throttle_factor;
  // I-term accumulation logic
  if (!armed) {
    i_roll_accum = 0.0f;
    i_pitch_accum = 0.0f;
  } else if (sys_cfg.angle_ki > 0.0f && throttle > ANGLE_I_THROTTLE_MIN) {
    float max_i_accum = MAX_ANGLE_I / sys_cfg.angle_ki;

    i_roll_accum += raw_roll_error * dt_sec;
    if (i_roll_accum > max_i_accum) i_roll_accum = max_i_accum;
    if (i_roll_accum < -max_i_accum) i_roll_accum = -max_i_accum;

    i_pitch_accum += raw_pitch_error * dt_sec;
    if (i_pitch_accum > max_i_accum) i_pitch_accum = max_i_accum;
    if (i_pitch_accum < -max_i_accum) i_pitch_accum = -max_i_accum;
  }

  // Output: target rate = P * error + I * accumulated
  output.target_roll_rate =
      sys_cfg.angle_kp * roll_error + sys_cfg.angle_ki * i_roll_accum;
  output.target_pitch_rate =
      sys_cfg.angle_kp * pitch_error + sys_cfg.angle_ki * i_pitch_accum;

  // Clamp output rates proportional to throttle transition
  // Prevents huge rate commands while on the ground.
  float rate_limit = MAX_ANGLE_RATE * throttle_factor;
  if (rate_limit < 30.0f) rate_limit = 30.0f;  // Minimum so corrections aren't zero

  if (output.target_roll_rate > rate_limit)
    output.target_roll_rate = rate_limit;
  if (output.target_roll_rate < -rate_limit)
    output.target_roll_rate = -rate_limit;
  if (output.target_pitch_rate > rate_limit)
    output.target_pitch_rate = rate_limit;
  if (output.target_pitch_rate < -rate_limit)
    output.target_pitch_rate = -rate_limit;
}

const angle_output_t *angle_control_get_output(void) { return &output; }
