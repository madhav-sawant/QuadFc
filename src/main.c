/**
 * @file main.c
 * @brief Quadcopter Flight Controller - Final Year BE Project
 *
 * @author Madhav Kapse
 * @date 2025
 * @version 2.0
 *
 * Hardware: ESP32 + MPU6050 + BMP280 + F450 frame
 * Control: Cascaded PID - Angle PI (outer) + Rate PID (inner)
 * RTOS: FreeRTOS - Control loop at 250Hz on Core 1
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "adc.h"
#include "blackbox.h"
#include "config.h"
#include "imu.h"
#include "mixer.h"
#include "pwm.h"
#include "rate_control.h"
#include "angle_control.h"
#include "rx.h"
#include "webserver.h"

// Hardware pins
#define LED_PIN 2
#define BUTTON_PIN 0

// Control parameters
#define CONTROL_LOOP_FREQ_HZ 250
#define TUNING_THROTTLE_LIMIT 1800

// RC input
#define RC_DEADBAND_US 10
#define RC_MAX_YAW_RATE_DPS 180.0f

// Crash detection thresholds
#define CRASH_ANGLE_DEG 50.0f
#define CRASH_GYRO_RATE_DPS 2000.0f

// System state
bool system_armed = false;
static bool error_state = false;

// RC auto-centering (captures stick position at arming)
static uint16_t rc_center_roll = 1500;
static uint16_t rc_center_pitch = 1500;
static uint16_t rc_center_yaw = 1500;

// Debug/logging
static uint16_t debug_motors[4];
static uint16_t debug_vbat = 0;

/**
 * Control loop task - runs at 250Hz on Core 1
 * Reads sensors → runs PID → updates motors
 */
static void control_loop_task(void *arg) {
  (void)arg;

  const int64_t cycle_us = 1000000 / CONTROL_LOOP_FREQ_HZ;
  int64_t next_cycle = esp_timer_get_time();
  int debug_div = 0;
  int bb_div = 0;

  printf("Control loop started on Core %d\n", xPortGetCoreID());
  esp_task_wdt_add(NULL);

  while (1) {
    esp_task_wdt_reset();

    // Step 1: Read sensors
    imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
    const imu_data_t *imu = imu_get_data();

    // Step 1b: Update yaw bias estimator
    imu_update_yaw_bias(rx_get_channel(3), rc_center_yaw, rx_get_channel(2));

    // Step 2: Read RC channels (0=Roll, 1=Pitch, 2=Throttle, 3=Yaw)
    uint16_t rx_roll = rx_get_channel(0);
    uint16_t rx_pitch = rx_get_channel(1);
    uint16_t rx_thr = rx_get_channel(2);
    uint16_t rx_yaw = rx_get_channel(3);

    // Step 3: Crash detection (active after takeoff to prevent false triggers)
    if (rx_thr > 1400) {
      if (fabs(imu->roll_deg) > CRASH_ANGLE_DEG ||
          fabs(imu->pitch_deg) > CRASH_ANGLE_DEG) {
        mixer_arm(false);
        system_armed = false;
        error_state = true;
      }
      if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
          fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
          fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
        mixer_arm(false);
        system_armed = false;
        error_state = true;
      }
    }

    // Step 4: Outer loop (Angle PI) - converts stick angle to target rate
    float target_roll_angle = 0.0f;
    float target_pitch_angle = 0.0f;

    int16_t roll_trim = rx_roll - rc_center_roll;
    int16_t pitch_trim = rx_pitch - rc_center_pitch;
    
    if (abs(roll_trim) > RC_DEADBAND_US) {
      target_roll_angle = (float)(roll_trim) / 500.0f * sys_cfg.angle_max;
    }
    if (abs(pitch_trim) > RC_DEADBAND_US) {
      target_pitch_angle = (float)(pitch_trim) / 500.0f * sys_cfg.angle_max;
    }

    angle_control_update(target_roll_angle, target_pitch_angle, 
                         imu->roll_deg, imu->pitch_deg, 
                         1.0f / CONTROL_LOOP_FREQ_HZ, 
                         system_armed, rx_thr);
    
    const angle_output_t *angle_out = angle_control_get_output();
    float target_roll_rate = angle_out->target_roll_rate;
    float target_pitch_rate = angle_out->target_pitch_rate;

    // Yaw: rate-only control (no magnetometer for heading hold)
    float target_yaw_rate = 0.0f;
    int16_t yaw_trim = rx_yaw - rc_center_yaw;
    if (abs(yaw_trim) > RC_DEADBAND_US) {
      target_yaw_rate = (float)(yaw_trim) / 500.0f * RC_MAX_YAW_RATE_DPS;
    }

    // Step 5: Motor mixing prep
    uint16_t throttle = system_armed ? rx_thr : 1000;
    if (throttle < 1000) throttle = 1000;
    if (throttle > TUNING_THROTTLE_LIMIT) throttle = TUNING_THROTTLE_LIMIT;

    // Step 6: Inner loop (Rate PID)
    rate_control_update(target_roll_rate, target_pitch_rate, target_yaw_rate,
                        imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps,
                        throttle);
    const rate_output_t *pid = rate_control_get_output();

    // Update voltage compensation
    float vbat = adc_get_battery_voltage_filtered();
    mixer_set_battery_voltage(vbat);

    mixer_update(throttle, pid->roll, pid->pitch, pid->yaw);

    // Debug output at 10Hz
    if (++debug_div >= 25) {
      debug_div = 0;
      mixer_get_outputs(&debug_motors[0], &debug_motors[1], 
                        &debug_motors[2], &debug_motors[3]);
    }

    // Blackbox logging at ~83Hz
    if (system_armed && ++bb_div >= BLACKBOX_LOG_DIVIDER) {
      bb_div = 0;
      // Get CURRENT motor values (not stale debug values from 10Hz)
      uint16_t bb_motors[4];
      mixer_get_outputs(&bb_motors[0], &bb_motors[1], &bb_motors[2], &bb_motors[3]);
      blackbox_entry_t entry = {
          .angle_roll = imu->roll_deg,
          .angle_pitch = imu->pitch_deg,
          .gyro_x = imu->gyro_x_dps,
          .gyro_y = imu->gyro_y_dps,
          .gyro_z = imu->gyro_z_dps,
          .pid_roll = pid->roll,
          .pid_pitch = pid->pitch,
          .pid_yaw = pid->yaw,
          .motor = {bb_motors[0], bb_motors[1], bb_motors[2], bb_motors[3]},
          .rc_roll = rx_roll,
          .rc_pitch = rx_pitch,
          .rc_yaw = rx_yaw,
          .rc_throttle = throttle,
          .battery_mv = (uint16_t)(vbat * 1000.0f),
          .i2c_errors = imu_get_i2c_errors(),
          .accel_z_raw = imu->accel_z_g,
          .vbat_comp = mixer_get_vbat_compensation(),
      };
      blackbox_log(&entry);
    } else if (!system_armed) {
      bb_div = 0;
    }

    // Precise cycle timing: bulk sleep + busy-wait near deadline
    next_cycle += cycle_us;
    while (esp_timer_get_time() < next_cycle) {
      if ((next_cycle - esp_timer_get_time()) > 2000) {
        vTaskDelay(1);
      } else {
        taskYIELD();
      }
    }
  }
}

/**
 * IMU calibration - must be done on a level surface
 */
static void perform_calibration(bool save_to_nvs) {
  gpio_set_level(LED_PIN, 1);  // LED ON = calibrating
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  imu_calibrate_accel();
  imu_calibrate_gyro();
  
  if (save_to_nvs) {
    imu_calibration_save_to_nvs();
  }
  
  gpio_set_level(LED_PIN, 0);
  
  // 5 fast blinks = calibration done
  for (int i = 0; i < 5; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * Main entry point - initializes hardware and starts control loop
 */
void app_main(void) {
  // Hold motor GPIOs LOW during boot to prevent ESC false-arming
  gpio_reset_pin(PWM_MOTOR_1_GPIO);
  gpio_reset_pin(PWM_MOTOR_2_GPIO);
  gpio_reset_pin(PWM_MOTOR_3_GPIO);
  gpio_reset_pin(PWM_MOTOR_4_GPIO);
  gpio_set_direction(PWM_MOTOR_1_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_2_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_3_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_4_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  gpio_set_level(PWM_MOTOR_2_GPIO, 0);
  gpio_set_level(PWM_MOTOR_3_GPIO, 0);
  gpio_set_level(PWM_MOTOR_4_GPIO, 0);

  // Init PWM and set motors to minimum
  pwm_init();
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000);
  }

  // Init NVS (stores PID gains and calibration)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    printf("NVS: Erasing and reinitializing...\n");
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Load PID config: defaults first, then NVS overrides
  config_load_defaults();
  config_load_from_nvs();

  // GPIO setup
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // Init subsystems
  adc_init();
  rx_init();
  mixer_init();
  blackbox_init();
  
  // WiFi AP for PID tuning and blackbox download
  webserver_init();

  // IMU init - halt if not detected
  if (imu_init() != ESP_OK) {
    printf("\n!!! CRITICAL: IMU not detected (SDA=21, SCL=22) !!!\n");
    error_state = true;
    while(1) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  // 3 slow blinks = "Hold BOOT button to calibrate"
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  
  bool button_pressed = (gpio_get_level(BUTTON_PIN) == 0);
  
  if (button_pressed) {
    // Calibration mode
    while (gpio_get_level(BUTTON_PIN) == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    perform_calibration(true);
  } else {
    // Normal boot - load saved calibration
    bool accel_cal_loaded = imu_calibration_load_from_nvs();
    if (!accel_cal_loaded) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(2000));
      gpio_set_level(LED_PIN, 0);
      imu_calibrate_accel();
    }
    imu_calibrate_gyro();  // Gyro always calibrates at boot
  }

  // 3 quick blinks = boot OK
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Level check - verify calibration is reasonable
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    float roll_sum = 0, pitch_sum = 0;
    for (int i = 0; i < 100; i++) {
        imu_read(0.004f);
        const imu_data_t *imu = imu_get_data();
        roll_sum += imu->roll_deg;
        pitch_sum += imu->pitch_deg;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float avg_roll = roll_sum / 100.0f;
    float avg_pitch = pitch_sum / 100.0f;

    if (fabsf(avg_roll) < 2.0f && fabsf(avg_pitch) < 2.0f) {
        for(int i=0; i<2; i++) {  // 2 blinks = level OK
          gpio_set_level(LED_PIN, 1);
          vTaskDelay(pdMS_TO_TICKS(100));
          gpio_set_level(LED_PIN, 0);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else {
        for(int i=0; i<5; i++) {  // 5 slow blinks = level warning
          gpio_set_level(LED_PIN, 1);
          vTaskDelay(pdMS_TO_TICKS(300));
          gpio_set_level(LED_PIN, 0);
          vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
  }

  // Start control loop on Core 1
  rate_control_init();
  angle_control_init();
  xTaskCreatePinnedToCore(control_loop_task, "control", 4096, NULL, 5, NULL, 1);

  // Housekeeping loop - ~100Hz on Core 0
  static int rx_fail = 0;
  static int bat_div = 0;
  static bool low_bat_latched = false;
  static uint8_t bat_invalid_count = 0;
  static bool bat_ever_valid = false;

  while (1) {
    // RX connection check (200ms timeout)
    bool rx_instant = rx_is_connected();
    rx_fail = rx_instant ? 0 : rx_fail + 1;
    bool rx_ok = (rx_fail < 20);

    uint16_t rx_aux1 = rx_get_channel(4);
    bool arm_sw = (rx_aux1 > 1600);

    // Arming: requires RX + arm switch + no errors + low throttle
    if (rx_ok && arm_sw && !error_state && !low_bat_latched) {
      if (!system_armed && rx_get_channel(2) < 1150) {
        system_armed = true;
        rate_control_init();
        angle_control_init();
        imu_reset_yaw_bias();  // Fresh start each flight
        
        rc_center_roll = rx_get_channel(0);
        rc_center_pitch = rx_get_channel(1);
        rc_center_yaw = rx_get_channel(3);
        
        mixer_arm(true);
        blackbox_clear();
        blackbox_start();
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        blackbox_stop();
        imu_reset_yaw_bias();  // Clean state for next arm
      }
      if (error_state && !arm_sw) {
        error_state = false;
      }
    }

    // Battery monitoring (1Hz)
    if (++bat_div >= 100) {
      bat_div = 0;
    adc_read_battery_voltage();  
    debug_vbat=(uint16_t)(adc_get_battery_voltage_filtered()*1000.0f);

      // Plausibility check for 3S pack wiring/sensor faults
      // Invalid for this system: <6.0V or >14.0V
      bool bat_invalid_sample = (debug_vbat < 6000 || debug_vbat > 14000);
      if (bat_invalid_sample) {
        if (bat_invalid_count < 255) {
          bat_invalid_count++;
        }
      } else {
        bat_invalid_count = 0;
        bat_ever_valid = true;
      }
    }

    // LED status: ON=armed, low battery, or battery sensor fault
    bool low_bat = (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold);
    bool bat_sensor_fault = (bat_invalid_count >= 3) || !bat_ever_valid;
    if (low_bat || bat_sensor_fault) {
      low_bat_latched = true;
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        blackbox_stop();
        imu_reset_yaw_bias();
      }
      gpio_set_level(LED_PIN, 1);
    } else {
      if (!arm_sw) {
        low_bat_latched = false;
      }
      gpio_set_level(LED_PIN, system_armed ? 1 : 0);
    }

    // Emergency stop via BOOT button
    if (system_armed && gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      error_state = true;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
