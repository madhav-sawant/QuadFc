/**
 * @file adc.c
 * @brief Battery voltage monitoring via ESP32 ADC with calibration
 */

#include "adc.h"
#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t adc_chars;

// IIR filter for raw ADC readings
static uint32_t filtered_adc = 0;
#define IIR_ALPHA_NUM 1
#define IIR_ALPHA_DEN 8

void adc_init(void) {
  adc1_config_width(ADC_BITWIDTH);
  adc1_config_channel_atten(ADC_CHAN, ADC_ATTEN);

  esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_BITWIDTH, 1100,
                           &adc_chars);
  filtered_adc = 0;
}

// Read raw ADC with 64-sample averaging + IIR filter
static uint16_t adc_read_raw(void) {
  uint32_t sum = 0;
  for (int i = 0; i < 64; i++) {
    sum += adc1_get_raw(ADC_CHAN);
  }
  uint32_t avg = sum / 64;

  if (filtered_adc == 0) {
    filtered_adc = avg;
  } else {
    filtered_adc =
        filtered_adc + (IIR_ALPHA_NUM * ((int32_t)avg - (int32_t)filtered_adc)) /
                            IIR_ALPHA_DEN;
  }
  return (uint16_t)filtered_adc;
}

// Convert raw ADC to millivolts using ESP-IDF calibration
static uint16_t adc_read_voltage(uint16_t raw) {
  uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  return (uint16_t)voltage;
}

// Float EMA for smooth voltage output (used by voltage compensation)
static float battery_voltage_filtered = 0.0f;
#define VBAT_LPF_ALPHA 0.2f  // ~5s time constant at 1Hz update

uint16_t adc_read_battery_voltage(void) {
  uint16_t raw = adc_read_raw();
  uint16_t adc_mv = adc_read_voltage(raw);

  // Scale by voltage divider ratio
  uint32_t battery_mv =
      ((uint32_t)adc_mv * VOLTAGE_SCALE_MV) / VOLTAGE_SCALE_DIV;

  // Update float EMA for compensation output
  float volts = (float)battery_mv / 1000.0f;
  if (battery_voltage_filtered < 1.0f) {
    battery_voltage_filtered = volts;  // Seed on first read
  } else {
    battery_voltage_filtered = VBAT_LPF_ALPHA * volts +
                               (1.0f - VBAT_LPF_ALPHA) * battery_voltage_filtered;
  }

  return (uint16_t)battery_mv;
}

float adc_get_battery_voltage_filtered(void) {
  return battery_voltage_filtered;
}
