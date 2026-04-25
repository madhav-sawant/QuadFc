/**
 * @file imu.c
 * @brief MPU6050 IMU driver with complementary filter for angle estimation
 *
 * Simplified signal chain:
 *   Raw I2C → Bias subtract → EMA low-pass → Complementary filter
 *
 * No deadbands, no biquad, no dynamic alpha, no runtime bias estimator.
 * Clean signals for the PID to work with.
 */

#include "imu.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>

// MPU6050 register addresses
#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_USER_CTRL 0x6A
#define REG_INT_PIN_CFG 0x37
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43
#define REG_WHO_AM_I 0x75

// I2C configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

// MPU6050 settings
// DLPF_CFG=0x04 → 20Hz HW lowpass (targets frame resonance at ~20Hz)
#define DLPF_CFG_20HZ 0x04
#define FS_SEL_2000 0x18
#define AFS_SEL_2G 0x00
#define GYRO_SCALE_FACTOR 16.4f      // LSB/(deg/s) for ±2000°/s
#define ACCEL_SCALE_FACTOR 16384.0f  // LSB/g for ±2g

// ==========================================================================
// CASCADED 2nd-ORDER BUTTERWORTH GYRO LPF
// ==========================================================================
// Two EMA stages in series (-40dB/decade). Effective cutoff: ~10Hz.
// Provides strong attenuation at 20Hz with acceptable phase lag for PID.
#define GYRO_LPF_ALPHA1 0.28f   // Stage 1
#define GYRO_LPF_ALPHA2 0.28f   // Stage 2

// Accel EMA: filtering for long-term drift correction in comp filter
#define ACCEL_LPF_ALPHA 0.10f

// ==========================================================================
// ADAPTIVE COMPLEMENTARY FILTER
// ==========================================================================
// Adjusts gyro/accel blend dynamically based on noise and |a| magnitude
// to prevent vibration-induced drift while retaining fast response.
#define COMP_ALPHA_MAX    0.99f   // Calm: almost pure gyro (fast response)
#define COMP_ALPHA_MIN    0.85f   // Noisy: heavy accel pull (kills drift)
#define GYRO_VAR_CALM     1.0f    // Below this variance → max alpha
#define GYRO_VAR_NOISY    25.0f   // Above this variance → min alpha
#define GYRO_VAR_EMA      0.01f   // Variance tracker time constant (slow)

// Accel magnitude gating: reject accel correction if |a| ≠ 1g
#define ACCEL_MAG_GATE_THRESHOLD 0.15f  // Reject if | |a| - 1g | > this

// Gyro spike rejection: clamp sample-to-sample change rate to reject glitches
#define GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE 3.2f

// Spike filter: limits max angle change rate (safety net)
#define MAX_ANGLE_RATE_DPS 500.0f

// ==========================================================================
// NOTCH FILTER AT 20Hz (F450 frame resonance)
// ==========================================================================
// IIR biquad notch (fs=250Hz, f0=20Hz, BW=5Hz).
// Attenuates 20Hz by ~40dB with minimal passband ripple.
#define NOTCH_B0  0.9286f
#define NOTCH_B1 (-1.6180f)
#define NOTCH_B2  0.9286f
#define NOTCH_A1 (-1.6180f)
#define NOTCH_A2  0.8573f

// ==========================================================================
// RUNTIME YAW GYRO BIAS ESTIMATOR
// ==========================================================================
// Tracks and subtracts vibration-induced DC offset on gyro_z.
// Adapts slowly when motors are loaded and yaw stick is centered.
#define YAW_BIAS_ALPHA 0.02f
#define YAW_BIAS_THROTTLE_MIN 1200  // Minimum throttle to adapt
#define YAW_BIAS_STICK_DEADBAND 20  // Threshold for centered stick

#define RAD_TO_DEG 57.2957795f

// State
static imu_data_t imu_state;

// Calibration offsets (stored in NVS)
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float accel_offset_pitch = 0.0f;
static float accel_offset_roll = 0.0f;
static float accel_scale_factor = 1.0f;

// Cascaded 2-stage EMA state (stage 1 raw, stage 2 smoothed)
static float gyro_s1_x = 0.0f, gyro_s2_x = 0.0f;  // Roll
static float gyro_s1_y = 0.0f, gyro_s2_y = 0.0f;  // Pitch
static float gyro_s1_z = 0.0f, gyro_s2_z = 0.0f;  // Yaw

// Notch filter state (x=input history, y=output history)
static float notch_x1_x = 0.0f, notch_x2_x = 0.0f, notch_y1_x = 0.0f, notch_y2_x = 0.0f;
static float notch_x1_y = 0.0f, notch_x2_y = 0.0f, notch_y1_y = 0.0f, notch_y2_y = 0.0f;
static float notch_x1_z = 0.0f, notch_x2_z = 0.0f, notch_y1_z = 0.0f, notch_y2_z = 0.0f;

// Accel filter state
static float accel_filtered_x = 0.0f;
static float accel_filtered_y = 0.0f;
static float accel_filtered_z = 0.0f;

static bool filter_initialized = false;

// Adaptive complementary filter state
static float gyro_variance_est = 0.0f;  // Running variance estimate (EMA)
static float gyro_prev_x = 0.0f;        // Previous sample for spike rejection
static float gyro_prev_y = 0.0f;
static float gyro_prev_z = 0.0f;

// Runtime yaw bias (adapts in-flight to compensate vibration-induced DC offset)
static float runtime_yaw_bias = 0.0f;
static bool yaw_bias_active = false;

// I2C error counter
static uint32_t i2c_error_count = 0;

/* I2C helpers */
static esp_err_t write_register(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) i2c_error_count++;
  return ret;
}

static esp_err_t read_registers(uint8_t start_reg, uint8_t *buffer,
                                size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, start_reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) i2c_error_count++;
  return ret;
}

/* Initialization */
esp_err_t imu_init(void) {
  uint8_t who_am_i;
  esp_err_t ret = read_registers(REG_WHO_AM_I, &who_am_i, 1);

  // Init I2C if not already done
  if (ret != ESP_OK) {
    printf("IMU: Initializing I2C...\n");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  }

  // Verify MPU6050 presence
  if (read_registers(REG_WHO_AM_I, &who_am_i, 1) != ESP_OK ||
      who_am_i != MPU6050_ADDR) {
    printf("IMU: MPU6050 not found!\n");
    return ESP_FAIL;
  }
  printf("IMU: MPU6050 detected\n");

  // Configure MPU6050
  write_register(REG_PWR_MGMT_1, 0x01);   // PLL with X gyro reference
  write_register(REG_USER_CTRL, 0x00);
  write_register(REG_INT_PIN_CFG, 0x02);   // I2C bypass for aux sensors
  write_register(REG_CONFIG, DLPF_CFG_20HZ);  // 20Hz HW filter — targets measured F450 resonance
  write_register(REG_GYRO_CONFIG, FS_SEL_2000);
  write_register(REG_ACCEL_CONFIG, AFS_SEL_2G);

  return ESP_OK;
}

/* Calibration */
void imu_calibrate_gyro(void) {
  const int samples = 2000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  uint8_t buffer[6];

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_GYRO_XOUT_H, buffer, 6) == ESP_OK) {
      sum_x += (int16_t)((buffer[0] << 8) | buffer[1]);
      sum_y += (int16_t)((buffer[2] << 8) | buffer[3]);
      sum_z += (int16_t)((buffer[4] << 8) | buffer[5]);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  gyro_bias_x = (sum_x / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_y = (sum_y / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_z = (sum_z / samples) / GYRO_SCALE_FACTOR;
}

void imu_calibrate_accel(void) {
  const int samples = 2000;
  float sum_pitch = 0, sum_roll = 0, sum_z = 0;
  uint8_t buffer[6];

  printf("IMU: Calibrating accelerometer...\n");

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
      float ax = (int16_t)((buffer[0] << 8) | buffer[1]) / ACCEL_SCALE_FACTOR;
      float ay = (int16_t)((buffer[2] << 8) | buffer[3]) / ACCEL_SCALE_FACTOR;
      float az = (int16_t)((buffer[4] << 8) | buffer[5]) / ACCEL_SCALE_FACTOR;

      sum_pitch += atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      sum_roll += atan2f(ay, az) * RAD_TO_DEG;
      sum_z += az;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  accel_offset_pitch = sum_pitch / samples;
  accel_offset_roll = sum_roll / samples;

  // Z-axis should read 1.0g when level
  float avg_z = sum_z / samples;
  if (avg_z > 0.5f && avg_z < 1.5f) {
    accel_scale_factor = 1.0f / avg_z;
  }
  printf("IMU: Accel calibration complete (scale=%.3f)\n", accel_scale_factor);
}

/* Main read function - called at 250Hz */
void imu_read(float dt_sec) {
  static bool first_read = true;
  uint8_t buffer[14];

  if (read_registers(REG_ACCEL_XOUT_H, buffer, 14) != ESP_OK) {
    return;
  }

  // Parse raw data (accel + gyro in one burst read)
  int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
  int16_t gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

  // Convert to physical units
  float accel_raw_x = (ax_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_y = (ay_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_z = (az_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float gyro_raw_x = (gx_raw / GYRO_SCALE_FACTOR) - gyro_bias_x;
  float gyro_raw_y = (gy_raw / GYRO_SCALE_FACTOR) - gyro_bias_y;
  float gyro_raw_z = (gz_raw / GYRO_SCALE_FACTOR) - gyro_bias_z;

  // -------------------------------------------------------------------------
  // TIER 1: Seed filters on first call to avoid startup transient
  // -------------------------------------------------------------------------
  if (!filter_initialized) {
    accel_filtered_x = accel_raw_x;
    accel_filtered_y = accel_raw_y;
    accel_filtered_z = accel_raw_z;
    // Seed all gyro filter stages with first reading
    gyro_s1_x = gyro_s2_x = gyro_raw_x;
    gyro_s1_y = gyro_s2_y = gyro_raw_y;
    gyro_s1_z = gyro_s2_z = gyro_raw_z;
    // Seed notch filter state too (avoids 1-sample transient)
    notch_x1_x = notch_x2_x = notch_y1_x = notch_y2_x = gyro_raw_x;
    notch_x1_y = notch_x2_y = notch_y1_y = notch_y2_y = gyro_raw_y;
    notch_x1_z = notch_x2_z = notch_y1_z = notch_y2_z = gyro_raw_z;
    filter_initialized = true;
  } else {
    // Accel: heavy EMA (only used for long-term drift correction in comp filter)
    accel_filtered_x = ACCEL_LPF_ALPHA * accel_raw_x +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_x;
    accel_filtered_y = ACCEL_LPF_ALPHA * accel_raw_y +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_y;
    accel_filtered_z = ACCEL_LPF_ALPHA * accel_raw_z +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_z;

    // -----------------------------------------------------------------------
    // TIER 2: Cascaded 2-stage EMA (2nd-order Butterworth equivalent)
    // Effective -3dB at ~13.5Hz. Kills the 20Hz frame resonance by ~18dB.
    // -----------------------------------------------------------------------
    // Stage 1
    gyro_s1_x = GYRO_LPF_ALPHA1 * gyro_raw_x + (1.0f - GYRO_LPF_ALPHA1) * gyro_s1_x;
    gyro_s1_y = GYRO_LPF_ALPHA1 * gyro_raw_y + (1.0f - GYRO_LPF_ALPHA1) * gyro_s1_y;
    gyro_s1_z = GYRO_LPF_ALPHA1 * gyro_raw_z + (1.0f - GYRO_LPF_ALPHA1) * gyro_s1_z;
    // Stage 2 (feeds notch)
    gyro_s2_x = GYRO_LPF_ALPHA2 * gyro_s1_x + (1.0f - GYRO_LPF_ALPHA2) * gyro_s2_x;
    gyro_s2_y = GYRO_LPF_ALPHA2 * gyro_s1_y + (1.0f - GYRO_LPF_ALPHA2) * gyro_s2_y;
    gyro_s2_z = GYRO_LPF_ALPHA2 * gyro_s1_z + (1.0f - GYRO_LPF_ALPHA2) * gyro_s2_z;
  }

  // -------------------------------------------------------------------------
  // TIER 3: Biquad notch at 20Hz — surgically removes the F450 frame resonance
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  // -------------------------------------------------------------------------
  float nx, ny, nz;
  nx = NOTCH_B0*gyro_s2_x + NOTCH_B1*notch_x1_x + NOTCH_B2*notch_x2_x
                           - NOTCH_A1*notch_y1_x  - NOTCH_A2*notch_y2_x;
  notch_x2_x = notch_x1_x; notch_x1_x = gyro_s2_x;
  notch_y2_x = notch_y1_x; notch_y1_x = nx;

  ny = NOTCH_B0*gyro_s2_y + NOTCH_B1*notch_x1_y + NOTCH_B2*notch_x2_y
                           - NOTCH_A1*notch_y1_y  - NOTCH_A2*notch_y2_y;
  notch_x2_y = notch_x1_y; notch_x1_y = gyro_s2_y;
  notch_y2_y = notch_y1_y; notch_y1_y = ny;

  nz = NOTCH_B0*gyro_s2_z + NOTCH_B1*notch_x1_z + NOTCH_B2*notch_x2_z
                           - NOTCH_A1*notch_y1_z  - NOTCH_A2*notch_y2_z;
  notch_x2_z = notch_x1_z; notch_x1_z = gyro_s2_z;
  notch_y2_z = notch_y1_z; notch_y1_z = nz;

  // -------------------------------------------------------------------------
  // TIER 4: Gyro spike rejection (per-sample delta clamp)
  // -------------------------------------------------------------------------
  // Clamps impossible sample-to-sample transients without lag.
  float gx_delta = nx - gyro_prev_x;
  float gy_delta = ny - gyro_prev_y;
  float gz_delta = nz - gyro_prev_z;

  if (gx_delta >  GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) nx = gyro_prev_x + GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;
  if (gx_delta < -GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) nx = gyro_prev_x - GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;
  if (gy_delta >  GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) ny = gyro_prev_y + GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;
  if (gy_delta < -GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) ny = gyro_prev_y - GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;
  if (gz_delta >  GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) nz = gyro_prev_z + GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;
  if (gz_delta < -GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE) nz = gyro_prev_z - GYRO_SPIKE_LIMIT_DPS_PER_SAMPLE;

  gyro_prev_x = nx;
  gyro_prev_y = ny;
  gyro_prev_z = nz;

  // Store final, clean gyro values
  imu_state.accel_x_g = accel_filtered_x;
  imu_state.accel_y_g = accel_filtered_y;
  imu_state.accel_z_g = accel_filtered_z;
  // Gyro: hardware DLPF → cascaded EMA → notch → spike clamp → yaw bias → PID
  imu_state.gyro_x_dps = nx;
  imu_state.gyro_y_dps = ny;
  imu_state.gyro_z_dps = nz - runtime_yaw_bias;  // Subtract vibration-induced DC offset

  // -------------------------------------------------------------------------
  // TIER 5: Adaptive complementary filter
  // -------------------------------------------------------------------------

  // 5a: Gyro variance tracker
  // Measures current gyro noise level to drive adaptive alpha.
  float gyro_energy = nx * nx + ny * ny;  // Only roll/pitch axes matter for angle
  gyro_variance_est = GYRO_VAR_EMA * gyro_energy +
                      (1.0f - GYRO_VAR_EMA) * gyro_variance_est;

  // 5b: Map variance to adaptive alpha
  // Linear interpolation: calm → COMP_ALPHA_MAX, noisy → COMP_ALPHA_MIN
  float alpha;
  if (gyro_variance_est <= GYRO_VAR_CALM) {
    alpha = COMP_ALPHA_MAX;   // Calm: trust gyro fully (fast, no lag)
  } else if (gyro_variance_est >= GYRO_VAR_NOISY) {
    alpha = COMP_ALPHA_MIN;   // Noisy: trust accel more (kills drift)
  } else {
    // Linear blend between calm and noisy
    float t = (gyro_variance_est - GYRO_VAR_CALM) /
              (GYRO_VAR_NOISY - GYRO_VAR_CALM);
    alpha = COMP_ALPHA_MAX - t * (COMP_ALPHA_MAX - COMP_ALPHA_MIN);
  }

  // 5c: Accel magnitude gating
  // Discard accel angles during heavy vibration/acceleration.
  float accel_mag = sqrtf(accel_filtered_x * accel_filtered_x +
                          accel_filtered_y * accel_filtered_y +
                          accel_filtered_z * accel_filtered_z);
  bool accel_valid = (fabsf(accel_mag - 1.0f) < ACCEL_MAG_GATE_THRESHOLD);

  // If accel is invalid, force alpha to 1.0 (pure gyro, no accel correction)
  if (!accel_valid) {
    alpha = 1.0f;
  }

  // Accelerometer angle calculation
  float accel_pitch =
      atan2f(accel_filtered_x, sqrtf(accel_filtered_y * accel_filtered_y +
                                     accel_filtered_z * accel_filtered_z)) *
      RAD_TO_DEG;
  float accel_roll = atan2f(accel_filtered_y, accel_filtered_z) * RAD_TO_DEG;

  // Remove calibration offsets
  accel_pitch -= accel_offset_pitch;
  accel_roll -= accel_offset_roll;

  // 5d: Complementary filter with adaptive alpha
  if (first_read) {
    imu_state.pitch_deg = accel_pitch;
    imu_state.roll_deg = accel_roll;
    first_read = false;
    printf("IMU: Initialized (Roll=%.1f, Pitch=%.1f)\n", imu_state.roll_deg,
           imu_state.pitch_deg);
  } else {
    float prev_pitch = imu_state.pitch_deg;
    float prev_roll = imu_state.roll_deg;

    float new_pitch =
        alpha * (prev_pitch + imu_state.gyro_y_dps * dt_sec) +
        (1.0f - alpha) * accel_pitch;
    float new_roll =
        alpha * (prev_roll + imu_state.gyro_x_dps * dt_sec) +
        (1.0f - alpha) * accel_roll;

    // Spike filter: limit max angle rate of change (safety net)
    float max_delta = MAX_ANGLE_RATE_DPS * dt_sec;
    float pitch_delta = new_pitch - prev_pitch;
    float roll_delta = new_roll - prev_roll;

    if (pitch_delta > max_delta)
      pitch_delta = max_delta;
    if (pitch_delta < -max_delta)
      pitch_delta = -max_delta;
    if (roll_delta > max_delta)
      roll_delta = max_delta;
    if (roll_delta < -max_delta)
      roll_delta = -max_delta;

    imu_state.pitch_deg = prev_pitch + pitch_delta;
    imu_state.roll_deg = prev_roll + roll_delta;
  }
}

const imu_data_t *imu_get_data(void) { return &imu_state; }

/* NVS calibration storage */
#define NVS_NAMESPACE "imu_cal"
#define NVS_MAGIC 0xCAFE1234

bool imu_calibration_load_from_nvs(void) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
    printf("IMU: No saved calibration\n");
    return false;
  }

  uint32_t magic = 0;
  size_t len = sizeof(uint32_t);
  if (nvs_get_blob(handle, "magic", &magic, &len) != ESP_OK ||
      magic != NVS_MAGIC) {
    nvs_close(handle);
    return false;
  }

  len = sizeof(float);
  nvs_get_blob(handle, "gyro_bias_x", &gyro_bias_x, &len);
  nvs_get_blob(handle, "gyro_bias_y", &gyro_bias_y, &len);
  nvs_get_blob(handle, "gyro_bias_z", &gyro_bias_z, &len);
  nvs_get_blob(handle, "accel_off_p", &accel_offset_pitch, &len);
  nvs_get_blob(handle, "accel_off_r", &accel_offset_roll, &len);
  nvs_get_blob(handle, "accel_scale", &accel_scale_factor, &len);

  nvs_close(handle);
  printf("IMU: Calibration loaded from NVS\n");
  return true;
}

void imu_calibration_save_to_nvs(void) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    return;
  }

  nvs_set_blob(handle, "gyro_bias_x", &gyro_bias_x, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_y", &gyro_bias_y, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_z", &gyro_bias_z, sizeof(float));
  nvs_set_blob(handle, "accel_off_p", &accel_offset_pitch, sizeof(float));
  nvs_set_blob(handle, "accel_off_r", &accel_offset_roll, sizeof(float));
  nvs_set_blob(handle, "accel_scale", &accel_scale_factor, sizeof(float));

  uint32_t magic = NVS_MAGIC;
  nvs_set_blob(handle, "magic", &magic, sizeof(uint32_t));

  nvs_commit(handle);
  nvs_close(handle);
  printf("IMU: Calibration saved to NVS\n");
}

/* Utility functions */

uint32_t imu_get_i2c_errors(void) {
  return i2c_error_count;
}

/**
 * Auto-level on arm: captures current surface as "level"
 * Allows takeoff from any surface without drift
 */
void imu_set_level_on_arm(void) {
  uint8_t buffer[6];
  if (read_registers(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
    float ax = (int16_t)((buffer[0] << 8) | buffer[1]) / ACCEL_SCALE_FACTOR;
    float ay = (int16_t)((buffer[2] << 8) | buffer[3]) / ACCEL_SCALE_FACTOR;
    float az = (int16_t)((buffer[4] << 8) | buffer[5]) / ACCEL_SCALE_FACTOR;
    
    // Current raw angles become the new zero reference
    accel_offset_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    accel_offset_roll = atan2f(ay, az) * RAD_TO_DEG;
    
    imu_state.pitch_deg = 0.0f;
    imu_state.roll_deg = 0.0f;
  }
}

/**
 * Runtime yaw gyro bias estimator.
 * Tracks and subtracts vibration-induced DC offset on gyro_z.
 * Slowly adapts when motors are loaded and yaw stick is centered.
 */
void imu_update_yaw_bias(uint16_t rc_yaw, uint16_t rc_yaw_center,
                         uint16_t throttle) {
  // Only adapt when conditions are safe:
  // 1. Motors spinning with load (vibration present)
  // 2. Pilot not commanding yaw (so the non-zero reading IS the bias)
  if (throttle >= YAW_BIAS_THROTTLE_MIN) {
    int16_t yaw_stick = (int16_t)rc_yaw - (int16_t)rc_yaw_center;
    if (yaw_stick > -YAW_BIAS_STICK_DEADBAND &&
        yaw_stick <  YAW_BIAS_STICK_DEADBAND) {
      // Read the UNCOMPENSATED notch output (add back current bias to get raw)
      float raw_gz = imu_state.gyro_z_dps + runtime_yaw_bias;

      // Slow EMA adaptation: α=0.005, τ ≈ 200 samples ≈ 0.8s
      // Tracks DC offset but won't follow real yaw maneuvers
      runtime_yaw_bias = YAW_BIAS_ALPHA * raw_gz +
                         (1.0f - YAW_BIAS_ALPHA) * runtime_yaw_bias;

      // Safety clamp: vibration bias shouldn't exceed ±15°/s
      // If it does, something is very wrong mechanically
      if (runtime_yaw_bias > 15.0f) runtime_yaw_bias = 15.0f;
      if (runtime_yaw_bias < -15.0f) runtime_yaw_bias = -15.0f;

      yaw_bias_active = true;
    }
  }
}

/**
 * Reset yaw bias on disarm — start fresh each flight.
 * Prevents stale bias from a previous flight causing issues.
 */
void imu_reset_yaw_bias(void) {
  runtime_yaw_bias = 0.0f;
  yaw_bias_active = false;
}

/**
 * Get current runtime yaw bias (for blackbox logging/debug).
 */
float imu_get_yaw_bias(void) {
  return runtime_yaw_bias;
}
