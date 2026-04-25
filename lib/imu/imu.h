/**
 * @file imu.h
 * @brief MPU6050 IMU driver interface
 */

#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float accel_x_g, accel_y_g, accel_z_g;  // Acceleration in g
  float gyro_x_dps, gyro_y_dps, gyro_z_dps;  // Angular rate in deg/s
  float roll_deg, pitch_deg;  // Fused angles in degrees
} imu_data_t;


// Core functions
esp_err_t imu_init(void);
void imu_read(float dt_sec);
const imu_data_t *imu_get_data(void);

// Calibration
void imu_calibrate_gyro(void);
void imu_calibrate_accel(void);
bool imu_calibration_load_from_nvs(void);
void imu_calibration_save_to_nvs(void);
void imu_set_level_on_arm(void);

// Runtime yaw bias correction (vibration-induced DC offset)
void imu_update_yaw_bias(uint16_t rc_yaw, uint16_t rc_yaw_center,
                         uint16_t throttle);
void imu_reset_yaw_bias(void);
float imu_get_yaw_bias(void);

// Diagnostics
uint32_t imu_get_i2c_errors(void);

#endif // IMU_H
