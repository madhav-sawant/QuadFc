/**
 * @file angle_control.c
 * @brief Angle PI controller implementation
 */

#include "angle_control.h"
#include "../config/config.h"


// Internal state
static float i_roll_accum = 0.0f;
static float i_pitch_accum = 0.0f;
static angle_output_t output;

// Constants from main.c
#define MAX_ANGLE_I 30.0f
#define MAX_ANGLE_RATE 150.0f
#define ANGLE_I_THROTTLE_MIN 1100  // Single threshold for all PIDs
#define ANGLE_I_TILT_THRESHOLD 2.0f  // Only accumulate if tilted >2°

void angle_control_init(void) {
  i_roll_accum = 0.0f;
  i_pitch_accum = 0.0f;
  output.target_roll_rate = 0.0f;
  output.target_pitch_rate = 0.0f;
}

void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle) {
  float roll_error = target_roll - actual_roll;
  float pitch_error = target_pitch - actual_pitch;

  // I-term logic: Optimal approach
  if (!armed) {
    // Reset when disarmed
    i_roll_accum = 0.0f;
    i_pitch_accum = 0.0f;
  } else if (sys_cfg.angle_ki > 0.0f && throttle > ANGLE_I_THROTTLE_MIN) {
    // Smart accumulation strategy:
    // 1. If tilted >2° → definitely accumulate (drone is clearly off-level)
    // 2. If throttle >1300us → accumulate (likely airborne, even if level)
    // 3. Otherwise → don't accumulate (sitting on ground at low throttle)
    
    float abs_roll = actual_roll < 0 ? -actual_roll : actual_roll;
    float abs_pitch = actual_pitch < 0 ? -actual_pitch : actual_pitch;
    
    bool drone_is_tilted = (abs_roll > ANGLE_I_TILT_THRESHOLD || 
                           abs_pitch > ANGLE_I_TILT_THRESHOLD);
    bool throttle_high = (throttle > 1300);  // Likely airborne at >1300us
    
    if (drone_is_tilted || throttle_high) {
      // Accumulate I-term (drone is flying or needs correction)
      float max_i_accum = MAX_ANGLE_I / sys_cfg.angle_ki;

      i_roll_accum += roll_error * dt_sec;
      if (i_roll_accum > max_i_accum)
        i_roll_accum = max_i_accum;
      if (i_roll_accum < -max_i_accum)
        i_roll_accum = -max_i_accum;

      i_pitch_accum += pitch_error * dt_sec;
      if (i_pitch_accum > max_i_accum)
        i_pitch_accum = max_i_accum;
      if (i_pitch_accum < -max_i_accum)
        i_pitch_accum = -max_i_accum;
    }
    // else: drone is flat (on ground), don't accumulate
  } else {
    // Hold I-term when throttle is low (prevent windup on ground, but don't lose trim in flight)
    // Do nothing - just don't add to accumulator
  }

  // Calculate Target Rate = P + I
  output.target_roll_rate =
      sys_cfg.angle_kp * roll_error + sys_cfg.angle_ki * i_roll_accum;
  output.target_pitch_rate =
      sys_cfg.angle_kp * pitch_error + sys_cfg.angle_ki * i_pitch_accum;

  // Clamp Output Rates
  if (output.target_roll_rate > MAX_ANGLE_RATE)
    output.target_roll_rate = MAX_ANGLE_RATE;
  if (output.target_roll_rate < -MAX_ANGLE_RATE)
    output.target_roll_rate = -MAX_ANGLE_RATE;

  if (output.target_pitch_rate > MAX_ANGLE_RATE)
    output.target_pitch_rate = MAX_ANGLE_RATE;
  if (output.target_pitch_rate < -MAX_ANGLE_RATE)
    output.target_pitch_rate = -MAX_ANGLE_RATE;
}

const angle_output_t *angle_control_get_output(void) { return &output; }
/**
 * @file angle_control.c
 * @brief Angle PI controller implementation
 */

#include "angle_control.h"
#include "../config/config.h"


// Internal state
static float i_roll_accum = 0.0f;
static float i_pitch_accum = 0.0f;
static angle_output_t output;

// Constants from main.c
#define MAX_ANGLE_I 30.0f
#define MAX_ANGLE_RATE 150.0f
#define ANGLE_I_THROTTLE_MIN 1100  // Single threshold for all PIDs
#define ANGLE_I_TILT_THRESHOLD 2.0f  // Only accumulate if tilted >2°

void angle_control_init(void) {
  i_roll_accum = 0.0f;
  i_pitch_accum = 0.0f;
  output.target_roll_rate = 0.0f;
  output.target_pitch_rate = 0.0f;
}

void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle) {
  float roll_error = target_roll - actual_roll;
  float pitch_error = target_pitch - actual_pitch;

  // I-term logic: Optimal approach
  if (!armed) {
    // Reset when disarmed
    i_roll_accum = 0.0f;
    i_pitch_accum = 0.0f;
  } else if (sys_cfg.angle_ki > 0.0f && throttle > ANGLE_I_THROTTLE_MIN) {
    // Smart accumulation strategy:
    // 1. If tilted >2° → definitely accumulate (drone is clearly off-level)
    // 2. If throttle >1300us → accumulate (likely airborne, even if level)
    // 3. Otherwise → don't accumulate (sitting on ground at low throttle)
    
    float abs_roll = actual_roll < 0 ? -actual_roll : actual_roll;
    float abs_pitch = actual_pitch < 0 ? -actual_pitch : actual_pitch;
    
    bool drone_is_tilted = (abs_roll > ANGLE_I_TILT_THRESHOLD || 
                           abs_pitch > ANGLE_I_TILT_THRESHOLD);
    bool throttle_high = (throttle > 1300);  // Likely airborne at >1300us
    
    if (drone_is_tilted || throttle_high) {
      // Accumulate I-term (drone is flying or needs correction)
      float max_i_accum = MAX_ANGLE_I / sys_cfg.angle_ki;

      i_roll_accum += roll_error * dt_sec;
      if (i_roll_accum > max_i_accum)
        i_roll_accum = max_i_accum;
      if (i_roll_accum < -max_i_accum)
        i_roll_accum = -max_i_accum;

      i_pitch_accum += pitch_error * dt_sec;
      if (i_pitch_accum > max_i_accum)
        i_pitch_accum = max_i_accum;
      if (i_pitch_accum < -max_i_accum)
        i_pitch_accum = -max_i_accum;
    }
    // else: drone is flat (on ground), don't accumulate
  } else {
    // Hold I-term when throttle is low (prevent windup on ground, but don't lose trim in flight)
    // Do nothing - just don't add to accumulator
  }

  // Calculate Target Rate = P + I
  output.target_roll_rate =
      sys_cfg.angle_kp * roll_error + sys_cfg.angle_ki * i_roll_accum;
  output.target_pitch_rate =
      sys_cfg.angle_kp * pitch_error + sys_cfg.angle_ki * i_pitch_accum;

  // Clamp Output Rates
  if (output.target_roll_rate > MAX_ANGLE_RATE)
    output.target_roll_rate = MAX_ANGLE_RATE;
  if (output.target_roll_rate < -MAX_ANGLE_RATE)
    output.target_roll_rate = -MAX_ANGLE_RATE;

  if (output.target_pitch_rate > MAX_ANGLE_RATE)
    output.target_pitch_rate = MAX_ANGLE_RATE;
  if (output.target_pitch_rate < -MAX_ANGLE_RATE)
    output.target_pitch_rate = -MAX_ANGLE_RATE;
}

const angle_output_t *angle_control_get_output(void) { return &output; }
