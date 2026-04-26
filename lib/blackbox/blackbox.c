/**
 * @file blackbox.c
 * @brief Flight data recorder using ring buffer + FreeRTOS queue
 *
 * Runs on Core 0 to avoid blocking the control loop on Core 1.
 * Data is stored in RAM and can be downloaded as CSV via the webserver.
 */

#include "blackbox.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "blackbox";

// Ring buffer
static blackbox_entry_t buffer[BLACKBOX_MAX_ENTRIES];
static volatile uint16_t write_index = 0;
static volatile uint16_t entry_count = 0;
static volatile bool recording = false;


#define BLACKBOX_QUEUE_SIZE 16
static QueueHandle_t log_queue = NULL;
static TaskHandle_t blackbox_task_handle = NULL;

// Task: consumes entries from queue and writes to ring buffer
static void blackbox_task(void *arg) {
  (void)arg;
  blackbox_entry_t entry;

  ESP_LOGI(TAG, "Blackbox task started on Core %d", xPortGetCoreID());

  while (1) {
    if (xQueueReceive(log_queue, &entry, portMAX_DELAY) == pdTRUE) {
      if (recording) {
        memcpy(&buffer[write_index], &entry, sizeof(blackbox_entry_t));
        write_index = (write_index + 1) % BLACKBOX_MAX_ENTRIES;
        if (entry_count < BLACKBOX_MAX_ENTRIES) {
          entry_count++;
        }
      }
    }
  }
}

void blackbox_init(void) {
  log_queue = xQueueCreate(BLACKBOX_QUEUE_SIZE, sizeof(blackbox_entry_t));
  if (log_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create queue!");
    return;
  }

  memset(buffer, 0, sizeof(buffer));
  write_index = 0;
  entry_count = 0;
  recording = false;

  // Pin to Core 0 (control loop runs on Core 1)
  xTaskCreatePinnedToCore(blackbox_task, "blackbox", 2048, NULL, 5,
                          &blackbox_task_handle, 0);

  ESP_LOGI(TAG, "Blackbox initialized. Buffer: %d entries, %d bytes",
           BLACKBOX_MAX_ENTRIES,
           (int)(BLACKBOX_MAX_ENTRIES * sizeof(blackbox_entry_t)));
}

void blackbox_log(const blackbox_entry_t *entry) {
  if (log_queue == NULL || entry == NULL || !recording)
    return;// ← exit function immediately, don't log anything
  xQueueSend(log_queue, entry, 0);
  
}

void blackbox_clear(void) {
 if (blackbox_task_handle!=NULL)
 {
  vTaskSuspend(blackbox_task_handle); //stop before touching the buffer 
 }
 if(log_queue!=NULL)
 {
  xQueueReset(log_queue);
 }
 write_index=0;
 entry_count=0;
 recording=false;

 if (blackbox_task_handle!=NULL)
 {
  vTaskResume(blackbox_task_handle);
 }
  ESP_LOGI(TAG, "Blackbox cleared");
}

uint16_t blackbox_get_count(void) { return entry_count; }

const blackbox_entry_t *blackbox_get_entry(uint16_t index) {
  if (index >= entry_count)
    return NULL;

  // Handle ring buffer wraparound
  uint16_t actual_index;
  if (entry_count >= BLACKBOX_MAX_ENTRIES) {
    actual_index = (write_index + index) % BLACKBOX_MAX_ENTRIES;
  } else {
    actual_index = index;
  }
  return &buffer[actual_index];
}

void blackbox_start(void) {
  recording = true;
  ESP_LOGI(TAG, "Recording started");
}

void blackbox_stop(void) {
  recording = false;
  ESP_LOGI(TAG, "Recording stopped");
}
