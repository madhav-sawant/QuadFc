/**
 * @file rx.h
 * @brief PPM receiver driver
 */

#ifndef RX_H
#define RX_H

#include <stdbool.h>
#include <stdint.h>

#define RX_PIN 33
#define RX_CHANNEL_COUNT 8

#define RX_MIN_US 900       // Minimum valid pulse width
#define RX_MAX_US 2100      // Maximum valid pulse width
#define RX_SYNC_MIN_US 2100 // Sync pulse threshold

void rx_init(void);
bool rx_is_connected(void);
uint16_t rx_get_channel(uint8_t channel_index);

#endif // RX_H
