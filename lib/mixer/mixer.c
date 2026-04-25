/**
 * @file mixer.c
 * @brief Quad-X motor mixer with voltage compensation
 *
 * Motor layout (props-out):
 *   M4(CCW)──────M2(CW)     FRONT
 *       ╲      ╱
 *         ╲  ╱
 *         ╱  ╲
 *       ╱      ╲
 *   M3(CW)──────M1(CCW)     REAR
 */

#include "mixer.h"
#include "pwm.h"

static volatile bool armed = false;
static uint16_t motors[4] = {1000, 1000, 1000, 1000};

// Voltage compensation (defaults to baseline = no compensation at startup)
static float vbat_volts = 11.7f;
static float last_compensation = 1.0f;

// Motor output smoothing (Betaflight motor_output_limit concept)
// PT1 filter on motor commands prevents jittery PWM from amplifying frame vibration.
// Alpha 0.7 = 70% new + 30% previous. At 250Hz this gives ~55Hz cutoff.
// This is critical: jerky motor commands excite frame resonance → IMU sees vibration
// → PID generates more jitter → positive feedback loop.
#define MOTOR_SMOOTH_ALPHA 0.70f
static float motor_smooth[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

#define VCOMP_REF_VOLTAGE   11.7f   // Tuning baseline voltage
#define VCOMP_MIN_VOLTAGE   11.5f   // Safety floor (prevents overcurrent)
#define VCOMP_MIN_RATIO     0.844f  // At 12.7V: (11.7/12.7)² = 0.844
#define VCOMP_MAX_RATIO     1.04f   // At 11.5V: (11.7/11.5)² = 1.035

static uint16_t clamp(int32_t val) {
  if (val < MIXER_IDLE_THROTTLE)
    return MIXER_IDLE_THROTTLE;
  if (val > MIXER_MAX_THROTTLE)
    return MIXER_MAX_THROTTLE;
  return (uint16_t)val;
}

void mixer_init(void) {
  armed = false;
  for (int i = 0; i < 4; i++) {
    motors[i] = MIXER_STOP_CMD;
  }
}

void mixer_update(uint16_t throttle, float roll, float pitch, float yaw) {
  if (!armed) {
    for (int i = 0; i < 4; i++) {
      motors[i] = MIXER_STOP_CMD;
    }
    pwm_stop_all();
    return;
  }

  // PID mixing hysteresis: ON at 1100µs, OFF at 1050µs
  static bool pid_active = false;
  if (throttle >= 1100)
    pid_active = true;
  else if (throttle < 1050)
    pid_active = false;

  if (!pid_active) {
    for (int i = 0; i < 4; i++) {
      motors[i] = MIXER_IDLE_THROTTLE;
      pwm_set_motor(i, MIXER_IDLE_THROTTLE);
    }
    return;
  }

  int32_t t = throttle;
  if (t > MIXER_MAX_THROTTLE)
    t = MIXER_MAX_THROTTLE;

  // Voltage compensation: Thrust ∝ V², scale Output linearly to keep Thrust constant
  // Formula: Output_new = Output_old * (V_ref / V_bat)
  float v_clamped = vbat_volts;
  if (v_clamped < VCOMP_MIN_VOLTAGE) v_clamped = VCOMP_MIN_VOLTAGE;
  float ratio = VCOMP_REF_VOLTAGE / v_clamped;
  float compensation = ratio; // Linear scaling is correct for constant thrust
  if (compensation < VCOMP_MIN_RATIO) compensation = VCOMP_MIN_RATIO;
  if (compensation > VCOMP_MAX_RATIO) compensation = VCOMP_MAX_RATIO;
  last_compensation = compensation;

  // Scale PID and Throttle to maintain consistent authority and hover point
  roll  *= compensation;
  pitch *= compensation;
  yaw   *= compensation;
  
  // Scale throttle range (1000-2000)
  t = (int32_t)((t - 1000) * compensation) + 1000;
  if (t > MIXER_MAX_THROTTLE) t = MIXER_MAX_THROTTLE;

  // Throttle-proportional PID authority in mixer
  // At low throttle, limit how much any motor can deviate from base throttle.
  // This prevents one motor at 2000 and another at 1100 during takeoff.
  // Max deviation scales from ±100 at 1100µs to ±400 at 1600µs+
  float mix_headroom = (float)(t - 1000) * 0.67f;  // ~67% of throttle above idle
  if (mix_headroom < 100.0f) mix_headroom = 100.0f;
  if (mix_headroom > 400.0f) mix_headroom = 400.0f;

  // Clamp PID contributions to available headroom
  if (roll  >  mix_headroom) roll  =  mix_headroom;
  if (roll  < -mix_headroom) roll  = -mix_headroom;
  if (pitch >  mix_headroom) pitch =  mix_headroom;
  if (pitch < -mix_headroom) pitch = -mix_headroom;

  // Yaw gets less headroom than roll/pitch — yaw saturation steals
  // motor authority and causes roll/pitch to lose control
  float yaw_headroom = mix_headroom * 0.5f;
  if (yaw_headroom < 50.0f) yaw_headroom = 50.0f;
  if (yaw_headroom > 200.0f) yaw_headroom = 200.0f;
  if (yaw   >  yaw_headroom) yaw   =  yaw_headroom;
  if (yaw   < -yaw_headroom) yaw   = -yaw_headroom;

  // Quad-X mixing matrix
  int32_t m1 = t - (int32_t)roll + (int32_t)pitch - (int32_t)yaw; // Rear Right (CCW)
  int32_t m2 = t - (int32_t)roll - (int32_t)pitch + (int32_t)yaw; // Front Right (CW)
  int32_t m3 = t + (int32_t)roll + (int32_t)pitch + (int32_t)yaw; // Rear Left (CW)
  int32_t m4 = t + (int32_t)roll - (int32_t)pitch - (int32_t)yaw; // Front Left (CCW)

  motors[0] = clamp(m1);
  motors[1] = clamp(m2);
  motors[2] = clamp(m3);
  motors[3] = clamp(m4);

  // Motor output smoothing (Betaflight-style PT1)
  // Prevents rapid PWM changes from exciting frame vibration
  uint16_t raw_motors[4] = {motors[0], motors[1], motors[2], motors[3]};
  for (int i = 0; i < 4; i++) {
    motor_smooth[i] = MOTOR_SMOOTH_ALPHA * (float)raw_motors[i] + 
                      (1.0f - MOTOR_SMOOTH_ALPHA) * motor_smooth[i];
    motors[i] = (uint16_t)(motor_smooth[i] + 0.5f);
    // Ensure smoothing doesn't push below idle
    if (motors[i] < MIXER_IDLE_THROTTLE) motors[i] = MIXER_IDLE_THROTTLE;
  }

  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, motors[i]);
  }
}

void mixer_arm(bool arm) {
  armed = arm;
  if (!arm) {
    // Reset motor smoothing filter state
    for (int i = 0; i < 4; i++) {
      motor_smooth[i] = 1000.0f;
    }
    mixer_update(1000, 0, 0, 0);
  }
}

void mixer_get_outputs(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4) {
  *m1 = motors[0];
  *m2 = motors[1];
  *m3 = motors[2];
  *m4 = motors[3];
}

void mixer_set_battery_voltage(float volts) {
  vbat_volts = volts;
}

float mixer_get_vbat_compensation(void) {
  return last_compensation;
}
