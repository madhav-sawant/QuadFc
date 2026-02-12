/**
 * @file adc.h
 * @brief Battery voltage monitoring using ESP32 ADC
 * 
 * Reads battery voltage through a voltage divider (R1=9.6k, R2=3.3k)
 * Uses ESP32 calibration for accurate readings
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

// ADC Configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_4  // GPIO 32
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_BITWIDTH_12

// Voltage Divider Calibration
// R1 = 9.6k, R2 = 3.3k
// Ratio = (9.6 + 3.3) / 3.3 = 3.9454 (tuned with multimeter)
#define VOLTAGE_SCALE_MV 39454
#define VOLTAGE_SCALE_DIV 10000

void adc_init(void);
uint16_t adc_read_battery_voltg(void);

#endif // ADC_H
