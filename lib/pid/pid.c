/**
 * @file pid.c
 * @brief Generic PID controller — simple and proven
 *
 * Clean PID with:
 * - D-term on measurement (avoids derivative kick on setpoint change)
 * - D-term EMA low-pass filter (reduces high-frequency noise)
 * - I-term anti-windup (hard clamp)
 * - I-term freeze (external control for ground/takeoff)
 */

#include "pid.h"
#include <math.h>

// D-term LPF: 70% new + 30% old — smooths noisy derivative
#define D_TERM_LPF_ALPHA 0.70f

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
  pid->integral = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->filtered_derivative = 0.0f;
  pid->integral_frozen = false;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec) {
  float error = setpoint - measurement;

  // P-term
  float p_out = pid->kp * error;

  // I-term with anti-windup and freeze
  if (!pid->integral_frozen) {
    pid->integral += error * dt_sec;
    if (pid->integral > pid->integral_limit)
      pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
      pid->integral = -pid->integral_limit;
  }
  float i_out = pid->ki * pid->integral;

  // D-term on measurement (not error — avoids derivative kick)
  float raw_d = -(measurement - pid->prev_measurement) / dt_sec;
  pid->prev_measurement = measurement;

  // EMA low-pass filter on derivative
  pid->filtered_derivative =
      D_TERM_LPF_ALPHA * raw_d +
      (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
  float d_out = pid->kd * pid->filtered_derivative;

  // Sum and clamp output
  float output = p_out + i_out + d_out;
  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}

void pid_freeze_integral(pid_controller_t *pid, bool freeze) {
  pid->integral_frozen = freeze;
}
