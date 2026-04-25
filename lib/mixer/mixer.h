/**
 * @file mixer.h
 * @brief Quad-X motor mixer with voltage compensation
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include <stdint.h>

#define MIXER_IDLE_THROTTLE 1100 // Minimum motor spin when armed
#define MIXER_MAX_THROTTLE  2000
#define MIXER_STOP_CMD      1000  // Motor off

void mixer_init(void);
void mixer_update(uint16_t throttle, float roll, float pitch, float yaw);
void mixer_arm(bool arm);
void mixer_get_outputs(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);
void mixer_set_battery_voltage(float volts);
float mixer_get_vbat_compensation(void);

#endif // MIXER_H
