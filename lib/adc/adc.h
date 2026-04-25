/**
 * @file adc.h
 * @brief Battery voltage monitoring via ESP32 ADC
 */

#ifndef ADC_H
#define ADC_H

#include "driver/adc.h"
#include <stdint.h>

// ADC configuration
#define ADC_UNIT    ADC_UNIT_1
#define ADC_CHAN    ADC1_CHANNEL_4  // GPIO32
#define ADC_ATTEN  ADC_ATTEN_DB_11
#define ADC_BITWIDTH ADC_WIDTH_BIT_12

// Voltage divider calibration: Vbat = Vadc * (SCALE_MV / SCALE_DIV)
#define VOLTAGE_SCALE_MV 3969
#define VOLTAGE_SCALE_DIV 1000

void adc_init(void);
uint16_t adc_read_battery_voltage(void);         // Returns battery voltage in mV
float adc_get_battery_voltage_filtered(void);    // Returns smoothed voltage in V

#endif // ADC_H
