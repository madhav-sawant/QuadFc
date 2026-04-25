/**
 * @file bmp280.c
 * @brief BMP280 Barometric Pressure Sensor Driver Implementation
 * 
 * This file implements all the functions declared in bmp280.h
 * Every function is explained in detail so you understand exactly what it does
 */

#include "bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

// Logging tag for debugging
static const char *TAG = "BMP280";

//=============================================================================
// I2C COMMUNICATION FUNCTIONS
//=============================================================================

/**
 * @brief Write a single byte to BMP280 register
 * 
 * HOW I2C WORKS:
 * 1. Send START condition
 * 2. Send device address + WRITE bit
 * 3. Send register address
 * 4. Send data byte
 * 5. Send STOP condition
 * 
 * WHY STATIC: This is a helper function only used inside this file
 * 
 * @param i2c_addr Sensor I2C address
 * @param reg_addr Register address to write to
 * @param data Data byte to write
 * @return ESP_OK if successful, error code otherwise
 */
static esp_err_t bmp280_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
    // Create I2C command link (sequence of I2C operations)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Build the command sequence:
    i2c_master_start(cmd);                                    // START condition
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);  // Address + WRITE
    i2c_master_write_byte(cmd, reg_addr, true);               // Register address
    i2c_master_write_byte(cmd, data, true);                   // Data to write
    i2c_master_stop(cmd);                                     // STOP condition
    
    // Execute the command on I2C bus
    // I2C_NUM_0 = I2C port 0 (ESP32 has 2 I2C ports)
    // 1000 / portTICK_PERIOD_MS = 1 second timeout
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    // Clean up (free memory used by command link)
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Read multiple bytes from BMP280 registers
 * 
 * HOW IT WORKS:
 * 1. Write register address (where to start reading)
 * 2. Read N bytes sequentially from that address
 * 
 * WHY SEQUENTIAL: BMP280 auto-increments register address after each read
 * So reading from 0xF7 gives you 0xF7, 0xF8, 0xF9, etc.
 * 
 * @param i2c_addr Sensor I2C address
 * @param reg_addr Starting register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return ESP_OK if successful, error code otherwise
 */
static esp_err_t bmp280_read_registers(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    // STEP 1: Write register address (tell sensor where to read from)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // STEP 2: Repeated START + read data
    i2c_master_start(cmd);  // Repeated START (no STOP between write and read)
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    
    // Read len bytes
    // Last byte: send NACK (tells sensor "this is the last byte")
    // Other bytes: send ACK (tells sensor "send more")
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

//=============================================================================
// CALIBRATION FUNCTIONS
//=============================================================================

/**
 * @brief Read factory calibration data from sensor
 * 
 * WHY: Each BMP280 is calibrated at factory
 * Calibration coefficients are stored in EEPROM (registers 0x88-0x9F)
 * We need these to convert raw sensor data to real pressure
 * 
 * HOW DATA IS STORED:
 * - Little-endian format (LSB first, then MSB)
 * - Some values are unsigned (dig_T1, dig_P1)
 * - Most are signed (can be negative)
 * 
 * EXAMPLE:
 * Register 0x88 = 0x6E, Register 0x89 = 0x6C
 * dig_T1 = 0x6C6E = 27758 (unsigned 16-bit)
 * 
 * @param sensor Pointer to sensor structure
 * @return true if successful, false if failed
 */
static bool bmp280_read_calibration(bmp280_t *sensor) {
    uint8_t calib_data[BMP280_CALIB_DATA_SIZE];  // 24 bytes
    
    // Read all calibration data in one go (faster than reading individually)
    if (bmp280_read_registers(sensor->config.i2c_addr, BMP280_REG_CALIB_START, 
                              calib_data, BMP280_CALIB_DATA_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return false;
    }
    
    // Parse calibration data (combine bytes into 16-bit values)
    // Little-endian: LSB at lower address, MSB at higher address
    
    // Temperature calibration (3 coefficients)
    sensor->calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];   // Unsigned
    sensor->calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];   // Signed
    sensor->calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];   // Signed
    
    // Pressure calibration (9 coefficients)
    sensor->calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];   // Unsigned
    sensor->calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];   // Signed
    sensor->calib.dig_P3 = (calib_data[11] << 8) | calib_data[10]; // Signed
    sensor->calib.dig_P4 = (calib_data[13] << 8) | calib_data[12]; // Signed
    sensor->calib.dig_P5 = (calib_data[15] << 8) | calib_data[14]; // Signed
    sensor->calib.dig_P6 = (calib_data[17] << 8) | calib_data[16]; // Signed
    sensor->calib.dig_P7 = (calib_data[19] << 8) | calib_data[18]; // Signed
    sensor->calib.dig_P8 = (calib_data[21] << 8) | calib_data[20]; // Signed
    sensor->calib.dig_P9 = (calib_data[23] << 8) | calib_data[22]; // Signed
    
    // Log calibration values for debugging
    ESP_LOGI(TAG, "Calibration data loaded:");
    ESP_LOGI(TAG, "  T1=%u, T2=%d, T3=%d", sensor->calib.dig_T1, sensor->calib.dig_T2, sensor->calib.dig_T3);
    ESP_LOGI(TAG, "  P1=%u, P2=%d, P3=%d", sensor->calib.dig_P1, sensor->calib.dig_P2, sensor->calib.dig_P3);
    
    return true;
}

//=============================================================================
// COMPENSATION FUNCTIONS (Convert raw data to real values)
//=============================================================================

/**
 * @brief Compensate raw temperature reading
 * 
 * WHY: Raw ADC value from sensor is meaningless without calibration
 * This formula converts raw value to actual temperature in Celsius
 * 
 * FORMULA: From BMP280 datasheet section 3.11.3
 * This is a 2nd-order polynomial compensation
 * 
 * SIDE EFFECT: Updates sensor->t_fine
 * t_fine is used in pressure compensation (pressure depends on temperature)
 * 
 * @param sensor Pointer to sensor structure
 * @param adc_T Raw temperature ADC value (20-bit)
 * @return Temperature in Celsius
 */
static float bmp280_compensate_temperature(bmp280_t *sensor, int32_t adc_T) {
    // Formula from datasheet (using integer math for speed)
    int32_t var1, var2;
    
    // var1 = ((adc_T/8 - dig_T1*2) * dig_T2) / 2048
    var1 = ((((adc_T >> 3) - ((int32_t)sensor->calib.dig_T1 << 1))) * 
            ((int32_t)sensor->calib.dig_T2)) >> 11;
    
    // var2 = (((adc_T/16 - dig_T1) * (adc_T/16 - dig_T1)) / 4096 * dig_T3) / 16384
    var2 = (((((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1))) >> 12) * 
            ((int32_t)sensor->calib.dig_T3)) >> 14;
    
    // t_fine is used in pressure calculation
    sensor->t_fine = var1 + var2;
    
    // Temperature in Celsius = t_fine / 5120
    float T = (sensor->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

/**
 * @brief Compensate raw pressure reading
 * 
 * WHY: Converts raw ADC value to actual pressure in Pascals
 * 
 * FORMULA: From BMP280 datasheet section 3.11.3
 * This is a complex polynomial that corrects for:
 * - Temperature effects
 * - Non-linearity of sensor
 * - Manufacturing variations
 * 
 * REQUIRES: t_fine must be calculated first (call compensate_temperature first)
 * 
 * @param sensor Pointer to sensor structure
 * @param adc_P Raw pressure ADC value (20-bit)
 * @return Pressure in Pascals (Pa)
 */
static float bmp280_compensate_pressure(bmp280_t *sensor, int32_t adc_P) {
    // Formula from datasheet (64-bit math to prevent overflow)
    int64_t var1, var2, p;
    
    // First part of compensation
    var1 = ((int64_t)sensor->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)sensor->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)sensor->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)sensor->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)sensor->calib.dig_P3) >> 8) + 
           ((var1 * (int64_t)sensor->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)sensor->calib.dig_P1) >> 33;
    
    // Avoid division by zero
    if (var1 == 0) {
        return 0;  // Invalid pressure
    }
    
    // Second part of compensation
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)sensor->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)sensor->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)sensor->calib.dig_P7) << 4);
    
    // Convert to Pascals (result is in Q24.8 format, divide by 256)
    return (float)p / 256.0f;
}

//=============================================================================
// PUBLIC API FUNCTIONS
//=============================================================================

/**
 * @brief Initialize BMP280 with default settings
 * 
 * WHAT IT DOES:
 * 1. Verify sensor is present (check chip ID)
 * 2. Soft reset sensor
 * 3. Read calibration data
 * 4. Configure sensor for quadcopter use
 * 5. Start continuous measurements
 * 
 * DEFAULT SETTINGS (optimized for drones):
 * - Pressure oversampling: x4 (good noise vs speed balance)
 * - Temperature oversampling: x1 (only needed for pressure compensation)
 * - IIR filter: x4 (removes vibration noise)
 * - Mode: NORMAL (continuous measurements)
 * - Standby: 0.5ms (fastest update rate)
 */
bool bmp280_init(bmp280_t *sensor, uint8_t i2c_addr) {
    ESP_LOGI(TAG, "Initializing BMP280 at address 0x%02X", i2c_addr);
    
    // Set configuration
    sensor->config.i2c_addr = i2c_addr;
    sensor->config.temp_oversampling = BMP280_OVERSAMPLING_X1;   // x1 for temp (fast)
    sensor->config.press_oversampling = BMP280_OVERSAMPLING_X4;  // x4 for pressure (balanced)
    sensor->config.mode = BMP280_MODE_NORMAL;                    // Continuous mode
    sensor->config.filter = BMP280_FILTER_4;                     // IIR filter x4
    sensor->config.standby_time = BMP280_STANDBY_0_5_MS;         // Fastest updates
    sensor->initialized = false;
    
    // STEP 1: Check if sensor is present
    uint8_t chip_id;
    if (bmp280_read_registers(i2c_addr, BMP280_REG_CHIP_ID, &chip_id, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID - sensor not found");
        return false;
    }
    
    if (chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "Wrong chip ID: 0x%02X (expected 0x%02X)", chip_id, BMP280_CHIP_ID);
        return false;
    }
    ESP_LOGI(TAG, "BMP280 detected (chip ID: 0x%02X)", chip_id);
    
    // STEP 2: Soft reset (ensures clean state)
    if (!bmp280_reset(sensor)) {
        ESP_LOGE(TAG, "Failed to reset sensor");
        return false;
    }
    
    // Wait for reset to complete (2ms typical)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // STEP 3: Read calibration data
    if (!bmp280_read_calibration(sensor)) {
        return false;
    }
    
    // STEP 4: Configure sensor
    // 
    // IMPORTANT (from datasheet Section 4.3.5):
    // "Writes to the 'config' register in normal mode may be IGNORED."
    // "In sleep mode writes are not ignored."
    //
    // So we MUST write config register FIRST (sensor is in sleep mode after reset)
    // THEN write ctrl_meas (which sets normal mode and starts measurements)
    
    // Build config register value (datasheet Table 23):
    // [7:5] = t_sb (standby time between measurements)
    // [4:2] = filter (IIR filter coefficient)
    // [1]   = reserved
    // [0]   = spi3w_en (0 for I2C mode)
    uint8_t config = (sensor->config.standby_time << 5) |
                     (sensor->config.filter << 2);
    
    if (bmp280_write_register(i2c_addr, BMP280_REG_CONFIG, config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config register");
        return false;
    }
    
    // Build ctrl_meas register value (datasheet Table 20):
    // [7:5] = osrs_t (temperature oversampling)
    // [4:2] = osrs_p (pressure oversampling)
    // [1:0] = mode  (00=sleep, 01/10=forced, 11=normal)
    //
    // Writing this LAST because it sets normal mode which starts measurements
    uint8_t ctrl_meas = (sensor->config.temp_oversampling << 5) |
                        (sensor->config.press_oversampling << 2) |
                        sensor->config.mode;
    
    if (bmp280_write_register(i2c_addr, BMP280_REG_CTRL_MEAS, ctrl_meas) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ctrl_meas register");
        return false;
    }
    
    sensor->initialized = true;
    ESP_LOGI(TAG, "BMP280 initialized successfully");
    
    return true;
}

/**
 * @brief Read pressure and temperature from sensor
 * 
 * WHAT IT DOES:
 * 1. Read 6 bytes from sensor (3 pressure + 3 temperature)
 * 2. Combine bytes into 20-bit values
 * 3. Apply compensation formulas
 * 4. Return calibrated values
 * 
 * WHY 20-BIT: BMP280 ADC is 20-bit resolution
 * Data format: [MSB] [LSB] [XLSB]
 * Value = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
 */
bool bmp280_read(bmp280_t *sensor, float *pressure, float *temperature) {
    if (!sensor->initialized) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return false;
    }
    
    // Read 6 bytes starting from pressure MSB register
    // Order: PRESS_MSB, PRESS_LSB, PRESS_XLSB, TEMP_MSB, TEMP_LSB, TEMP_XLSB
    uint8_t data[6];
    if (bmp280_read_registers(sensor->config.i2c_addr, BMP280_REG_PRESS_MSB, data, 6) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return false;
    }
    
    // Combine bytes into 20-bit values
    // Shift MSB left 12 bits, LSB left 4 bits, XLSB right 4 bits
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    
    // Apply compensation formulas
    // IMPORTANT: Temperature MUST be compensated first (sets t_fine for pressure)
    *temperature = bmp280_compensate_temperature(sensor, adc_T);
    *pressure = bmp280_compensate_pressure(sensor, adc_P);
    
    return true;
}

/**
 * @brief Read only pressure (faster if you don't need temperature)
 */
bool bmp280_read_pressure(bmp280_t *sensor, float *pressure) {
    float temp;  // Dummy variable (we need to calculate it but don't return it)
    return bmp280_read(sensor, pressure, &temp);
}

/**
 * @brief Convert pressure to altitude
 * 
 * FORMULA: International Standard Atmosphere (ISA) barometric formula
 * altitude = 44330 × (1 - (P/P0)^0.1903)
 * 
 * WHERE:
 * - 44330 = scaling constant (meters)
 * - P = current pressure
 * - P0 = sea level pressure (101325 Pa standard)
 * - 0.1903 = 1/5.255 (temperature lapse rate constant)
 * 
 * ACCURACY: ±1m at sea level, degrades with altitude
 * 
 * NOTE: For altitude hold, use RELATIVE altitude (pressure difference)
 * not absolute altitude from this function
 */
float bmp280_pressure_to_altitude(float pressure, float sea_level_pressure) {
    // Avoid division by zero
    if (sea_level_pressure == 0) {
        sea_level_pressure = 101325.0f;  // Standard sea level pressure
    }
    
    // Barometric formula
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

/**
 * @brief Soft reset sensor
 * 
 * HOW: Write 0xB6 to reset register
 * WHY: Resets all registers to default values
 * WHEN: If sensor seems stuck or readings are wrong
 */
bool bmp280_reset(bmp280_t *sensor) {
    // Magic value 0xB6 triggers soft reset
    if (bmp280_write_register(sensor->config.i2c_addr, BMP280_REG_RESET, 0xB6) != ESP_OK) {
        return false;
    }
    
    sensor->initialized = false;
    return true;
}

/**
 * @brief Check if sensor is currently measuring
 * 
 * HOW: Read status register bit 3
 * WHY: Sensor takes ~5.5ms to measure (for x4 oversampling)
 * Reading during measurement gives old data
 * 
 * WHEN TO USE: If you need to know when fresh data is ready
 * (Not usually needed in NORMAL mode - sensor updates continuously)
 */
bool bmp280_is_measuring(bmp280_t *sensor) {
    uint8_t status;
    if (bmp280_read_registers(sensor->config.i2c_addr, BMP280_REG_STATUS, &status, 1) != ESP_OK) {
        return false;
    }
    
    // Bit 3: measuring (1 = measuring, 0 = results ready)
    return (status & 0x08) != 0;
}
