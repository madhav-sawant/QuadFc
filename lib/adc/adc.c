/**
 * @file adc.c
 * @brief Battery voltage monitoring implementation
 * 
 * Reads battery voltage using ADC with voltage divider
 * - Uses ESP32 calibration for accuracy
 * - Averages 64 samples to reduce noise
 * - Applies IIR filter for stable readings
 */

#include "adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <stdio.h>

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool cali_enabled = false;

void adc_init(void) {
  // 1. Init ADC Unit
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  // 2. Config Channel
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_WIDTH,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

  // 3. Init ADC Calibration (uses factory eFuse data)
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT,
      .atten = ADC_ATTEN,
      .bitwidth = ADC_WIDTH,
  };
  esp_err_t ret =
      adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
  if (ret == ESP_OK) {
    cali_enabled = true;
    printf("[ADC] Calibration enabled (eFuse line fitting)\n");
  } else {
    cali_enabled = false;
    printf("[ADC] WARNING: Calibration not available, using raw conversion\n");
  }
}

// IIR filter state
static uint16_t adc_filtered_raw = 0;

static uint16_t adc_read_raw(void) {
  int raw_val = 0;
  uint32_t sum = 0;

  // Average 64 samples to reduce noise
  for (int i = 0; i < 64; i++) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_val));
    sum += raw_val;
  }

  uint16_t current = (uint16_t)(sum / 64);

  // Apply IIR low-pass filter for stability
  // New reading contributes 1/16, old value contributes 15/16
  if (adc_filtered_raw == 0) {
    adc_filtered_raw = current;
  } else {
    adc_filtered_raw =
        (current >> 4) + (adc_filtered_raw - (adc_filtered_raw >> 4));
  }

  return adc_filtered_raw;
}

static uint16_t adc_read_voltage(uint16_t raw) {
  if (cali_enabled) {
    // Use ESP32 calibration for accurate voltage
    int voltage_mv = 0;
    adc_cali_raw_to_voltage(adc_cali_handle, (int)raw, &voltage_mv);
    return (uint16_t)voltage_mv;
  }

  // Fallback: simple linear conversion
  uint16_t voltage = (raw * 3300) / 4095;
  return voltage;
}

uint16_t adc_read_battery_voltg(void) {
  uint16_t raw = adc_read_raw();
  uint16_t adc_mv = adc_read_voltage(raw);

  // Scale up using voltage divider ratio
  uint32_t battery_mv =
      ((uint32_t)adc_mv * VOLTAGE_SCALE_MV) / VOLTAGE_SCALE_DIV;

  return (uint16_t)battery_mv;
}
