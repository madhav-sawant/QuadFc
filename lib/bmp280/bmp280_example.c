/**
 * @file bmp280_example.c
 * @brief Example showing how to use BMP280 library
 * 
 * This is NOT part of the main firmware yet!
 * It's just an example to show you how the library works
 * 
 * TO TEST THIS:
 * 1. Connect BMP280 to ESP32 I2C pins (SDA=GPIO21, SCL=GPIO22)
 * 2. Compile and upload
 * 3. Open serial monitor
 * 4. You should see pressure and temperature readings every second
 */

#include "bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP280_EXAMPLE";

/**
 * STEP 1: Initialize I2C bus
 * 
 * WHY: BMP280 uses I2C to communicate with ESP32
 * We need to configure I2C before using the sensor
 * 
 * I2C PINS (default ESP32):
 * - SDA (data): GPIO 21
 * - SCL (clock): GPIO 22
 * - Pull-ups: Internal (enabled)
 * 
 * SPEED: 100kHz (standard mode) or 400kHz (fast mode)
 * BMP280 supports up to 3.4MHz, but 100kHz is reliable
 */
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,              // ESP32 is master, sensor is slave
        .sda_io_num = GPIO_NUM_21,            // SDA pin
        .scl_io_num = GPIO_NUM_22,            // SCL pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up
        .master.clk_speed = 100000,           // 100kHz (standard mode)
    };
    
    // Configure I2C
    i2c_param_config(I2C_NUM_0, &conf);
    
    // Install I2C driver
    // No RX/TX buffers needed for master mode
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    
    ESP_LOGI(TAG, "I2C initialized");
}

/**
 * STEP 2: Main application
 * 
 * WHAT IT DOES:
 * 1. Initialize I2C
 * 2. Initialize BMP280
 * 3. Read pressure/temperature in loop
 * 4. Print values every second
 */
void app_main(void) {
    ESP_LOGI(TAG, "BMP280 Example Starting");
    
    // Initialize I2C bus
    i2c_master_init();
    
    // Create sensor structure
    bmp280_t sensor;
    
    // Initialize sensor with default settings
    // Try address 0x76 first (most common)
    if (!bmp280_init(&sensor, BMP280_I2C_ADDR_PRIMARY)) {
        ESP_LOGE(TAG, "Failed to initialize BMP280 at 0x76");
        
        // Try alternate address 0x77
        ESP_LOGI(TAG, "Trying alternate address 0x77...");
        if (!bmp280_init(&sensor, BMP280_I2C_ADDR_SECONDARY)) {
            ESP_LOGE(TAG, "Failed to initialize BMP280 at 0x77");
            ESP_LOGE(TAG, "Check wiring and I2C address!");
            return;
        }
    }
    
    ESP_LOGI(TAG, "BMP280 initialized successfully!");
    ESP_LOGI(TAG, "Starting measurements...\n");
    
    // Variables to store readings
    float pressure, temperature;
    float initial_pressure = 0;
    bool first_reading = true;
    
    // Main loop
    while (1) {
        // Read sensor
        if (bmp280_read(&sensor, &pressure, &temperature)) {
            // Save first reading as reference
            if (first_reading) {
                initial_pressure = pressure;
                first_reading = false;
                ESP_LOGI(TAG, "Reference pressure: %.2f Pa", initial_pressure);
            }
            
            // Calculate relative altitude (from first reading)
            float altitude_change = bmp280_pressure_to_altitude(pressure, initial_pressure);
            
            // Print readings
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
            ESP_LOGI(TAG, "Pressure:    %.2f Pa (%.2f hPa)", pressure, pressure / 100.0f);
            ESP_LOGI(TAG, "Altitude:    %.2f m (relative to start)", altitude_change);
            ESP_LOGI(TAG, "---");
            
            // WHAT THESE VALUES MEAN:
            // - Temperature: Ambient temperature in Celsius
            // - Pressure: Atmospheric pressure in Pascals (Pa)
            //   * 1 hPa = 100 Pa (hectopascal, common weather unit)
            //   * Sea level ≈ 101325 Pa = 1013.25 hPa
            // - Altitude: Height change from first reading
            //   * Move sensor up → altitude increases
            //   * Move sensor down → altitude decreases
            //   * 1 Pa change ≈ 8cm altitude change
            
        } else {
            ESP_LOGE(TAG, "Failed to read sensor");
        }
        
        // Wait 1 second before next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * EXPECTED OUTPUT (example):
 * 
 * I (123) BMP280_EXAMPLE: BMP280 Example Starting
 * I (145) BMP280_EXAMPLE: I2C initialized
 * I (167) BMP280: Initializing BMP280 at address 0x76
 * I (189) BMP280: BMP280 detected (chip ID: 0x58)
 * I (201) BMP280: Calibration data loaded:
 * I (203) BMP280:   T1=27758, T2=26453, T3=-1000
 * I (205) BMP280:   P1=36477, P2=-10685, P3=3024
 * I (207) BMP280: BMP280 initialized successfully
 * I (209) BMP280_EXAMPLE: BMP280 initialized successfully!
 * I (211) BMP280_EXAMPLE: Starting measurements...
 * 
 * I (1213) BMP280_EXAMPLE: Reference pressure: 101325.50 Pa
 * I (1215) BMP280_EXAMPLE: Temperature: 25.34 °C
 * I (1217) BMP280_EXAMPLE: Pressure:    101325.50 Pa (1013.26 hPa)
 * I (1219) BMP280_EXAMPLE: Altitude:    0.00 m (relative to start)
 * I (1221) BMP280_EXAMPLE: ---
 * 
 * I (2223) BMP280_EXAMPLE: Temperature: 25.35 °C
 * I (2225) BMP280_EXAMPLE: Pressure:    101324.25 Pa (1013.24 hPa)
 * I (2227) BMP280_EXAMPLE: Altitude:    0.10 m (relative to start)
 * I (2229) BMP280_EXAMPLE: ---
 * 
 * ... (continues every second)
 * 
 * TRY THIS:
 * - Lift sensor up → altitude increases
 * - Lower sensor → altitude decreases
 * - Blow on sensor → temperature increases slightly
 * - Cover sensor with hand → pressure changes (body heat + air pressure)
 */
