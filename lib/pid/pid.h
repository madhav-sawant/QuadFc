/**
 * @file pid.h
 * @brief Generic PID controller with D-term low-pass filter
 *
 * Simple, proven PID:
 * - P: proportional to error
 * - I: with anti-windup clamp and freeze capability
 * - D: on measurement (not error), with EMA low-pass filter
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
  float kp, ki, kd;
  float output_limit;
  float integral_limit;
  float integral;
  float prev_measurement;
  float filtered_derivative;
  bool integral_frozen;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit);

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec);

void pid_freeze_integral(pid_controller_t *pid, bool freeze);

#endif // PID_H
