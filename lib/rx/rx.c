/**
 * @file rx.c
 * @brief PPM receiver driver with median filter for noise rejection
 */

#include "rx.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile uint16_t rx_channels[RX_CHANNEL_COUNT];
static volatile uint8_t current_channel = 0;
static volatile int64_t last_time_us = 0;
static volatile bool connected = false;
static volatile int64_t last_frame_time_us = 0;

// 3-sample median filter
#define MEDIAN_SAMPLES 3
static volatile uint16_t rx_buffer[RX_CHANNEL_COUNT][MEDIAN_SAMPLES];
static volatile uint8_t rx_buffer_idx[RX_CHANNEL_COUNT] = {0};

static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    if (b > c) return b;
    else if (a > c) return c;
    else return a;
  } else {
    if (a > c) return a;
    else if (b > c) return c;
    else return b;
  }
}

// PPM ISR: measures time between rising edges
static void IRAM_ATTR rx_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  int64_t dt = now - last_time_us;
  last_time_us = now;

  if (dt > RX_SYNC_MIN_US) {
    // Sync pulse detected (gap between frames)
    current_channel = 0;
    connected = true;
    last_frame_time_us = now;
  } else if (dt >= RX_MIN_US && dt <= RX_MAX_US) {
    if (current_channel < RX_CHANNEL_COUNT) {
      // Store in circular buffer and apply median filter
      uint8_t idx = rx_buffer_idx[current_channel];
      rx_buffer[current_channel][idx] = (uint16_t)dt;
      rx_buffer_idx[current_channel] = (idx + 1) % MEDIAN_SAMPLES;

      rx_channels[current_channel] = median3(
          rx_buffer[current_channel][0],
          rx_buffer[current_channel][1],
          rx_buffer[current_channel][2]);

      current_channel++;
    }
  }
}

void rx_init(void) {
  // Safe defaults
  for (int i = 0; i < RX_CHANNEL_COUNT; i++) {
    rx_channels[i] = 1500;
  }
  rx_channels[2] = 1000;  // Throttle to minimum
  rx_channels[4] = 1000;  // Arm switch off
  rx_channels[5] = 1000;

  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << RX_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE  // PPM: rising-to-rising edge timing
  };

  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(RX_PIN, rx_isr_handler, NULL);
}

bool rx_is_connected(void) {
  if (esp_timer_get_time() - last_frame_time_us > 100000) {
    connected = false;  // 100ms timeout
  }
  return connected;
}

uint16_t rx_get_channel(uint8_t channel_index) {
  if (channel_index >= RX_CHANNEL_COUNT)
    return 0;
  // Atomic read to prevent torn reads during ISR update
  portDISABLE_INTERRUPTS();
  uint16_t val = rx_channels[channel_index];
  portENABLE_INTERRUPTS();
  return val;
}
