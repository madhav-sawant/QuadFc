/**
 * @file bmp280.h
 * @brief BMP280 Barometric Pressure Sensor Driver
 * 
 * WHAT THIS LIBRARY DOES:
 * - Reads pressure and temperature from BMP280 sensor
 * - Converts raw sensor data to calibrated pressure (Pascals)
 * - Provides filtered pressure for stable altitude readings
 * 
 * WHY WE NEED THIS:
 * - For altitude hold feature (measures height above ground)
 * - BMP280 measures atmospheric pressure (pressure decreases with altitude)
 * - 1 Pascal change ≈ 8cm altitude change at sea level
 * 
 * HOW IT WORKS:
 * 1. Initialize sensor (configure oversampling, filter)
 * 2. Read raw pressure/temperature (20-bit values)
 * 3. Apply factory calibration (12 coefficients stored in sensor)
 * 4. Filter pressure (remove noise from vibrations)
 * 5. Output stable pressure reading
 */

#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>

//=============================================================================
// SENSOR CONFIGURATION
//=============================================================================

/**
 * I2C ADDRESS
 * 
 * WHY: BMP280 can have 2 addresses depending on SDO pin connection
 * - 0x76 if SDO connected to GND (most common)
 * - 0x77 if SDO connected to VCC
 * 
 * HOW TO CHECK: Run I2C scanner to detect which address your sensor uses
 */
#define BMP280_I2C_ADDR_PRIMARY   0x76  // SDO to GND
#define BMP280_I2C_ADDR_SECONDARY 0x77  // SDO to VCC

/**
 * REGISTER ADDRESSES
 * 
 * WHY: These are memory locations inside the BMP280 chip
 * We read/write these addresses to control the sensor and get data
 * 
 * Reference: BMP280 Datasheet Table 18 (Register Map)
 */
#define BMP280_REG_CHIP_ID        0xD0  // Chip ID (should read 0x58)
#define BMP280_REG_RESET          0xE0  // Soft reset register
#define BMP280_REG_STATUS         0xF3  // Status (measuring, updating)
#define BMP280_REG_CTRL_MEAS      0xF4  // Control measurement settings
#define BMP280_REG_CONFIG         0xF5  // Configuration (filter, standby)
#define BMP280_REG_PRESS_MSB      0xF7  // Pressure data (most significant byte)
#define BMP280_REG_PRESS_LSB      0xF8  // Pressure data (least significant byte)
#define BMP280_REG_PRESS_XLSB     0xF9  // Pressure data (extra bits)
#define BMP280_REG_TEMP_MSB       0xFA  // Temperature data (MSB)
#define BMP280_REG_TEMP_LSB       0xFB  // Temperature data (LSB)
#define BMP280_REG_TEMP_XLSB      0xFC  // Temperature data (extra bits)

/**
 * CALIBRATION DATA REGISTERS
 * 
 * WHY: Each BMP280 sensor is slightly different due to manufacturing
 * Factory calibration coefficients correct these differences
 * These 12 values are programmed at factory and stored in sensor EEPROM
 * 
 * WHAT THEY DO: Used in compensation formulas to convert raw data to real pressure
 */
#define BMP280_REG_CALIB_START    0x88  // Start of calibration data (dig_T1)
#define BMP280_CALIB_DATA_SIZE    24    // 24 bytes = 12 coefficients × 2 bytes each

/**
 * CHIP ID VALUE
 * 
 * WHY: Used to verify we're talking to a BMP280 (not BMP180 or other sensor)
 * Reading register 0xD0 should return 0x58 for BMP280
 */
#define BMP280_CHIP_ID            0x58

/**
 * OVERSAMPLING SETTINGS
 * 
 * WHAT: How many times sensor measures before outputting value
 * WHY: More samples = less noise, but slower reading
 * 
 * TRADE-OFF:
 * - x1 (skip):   Fastest, noisiest (±2 Pa noise)
 * - x2:          Fast, moderate noise (±1 Pa)
 * - x4:          Balanced (±0.5 Pa) ← RECOMMENDED
 * - x8:          Slow, low noise (±0.3 Pa)
 * - x16:         Slowest, lowest noise (±0.2 Pa)
 * 
 * FOR QUADCOPTER: x4 is best balance (5.5ms read time, ±0.5 Pa noise)
 */
#define BMP280_OVERSAMPLING_SKIP  0x00  // No measurement
#define BMP280_OVERSAMPLING_X1    0x01  // 1 sample
#define BMP280_OVERSAMPLING_X2    0x02  // 2 samples
#define BMP280_OVERSAMPLING_X4    0x03  // 4 samples (recommended)
#define BMP280_OVERSAMPLING_X8    0x04  // 8 samples
#define BMP280_OVERSAMPLING_X16   0x05  // 16 samples

/**
 * POWER MODES
 * 
 * SLEEP:  Sensor off, no measurements (lowest power)
 * FORCED: Take one measurement, then sleep (for low-power applications)
 * NORMAL: Continuous measurements (for quadcopter - we need constant updates)
 */
#define BMP280_MODE_SLEEP         0x00
#define BMP280_MODE_FORCED        0x01
#define BMP280_MODE_NORMAL        0x03  // Use this for flight controller

/**
 * FILTER SETTINGS
 * 
 * WHAT: Hardware IIR (Infinite Impulse Response) filter inside BMP280
 * WHY: Removes high-frequency noise from vibrations
 * 
 * HOW IT WORKS: New_output = (Old_output × (filter-1) + New_reading) / filter
 * 
 * FILTER_OFF:  No filtering (fastest response, noisiest)
 * FILTER_2:    Light filtering
 * FILTER_4:    Moderate filtering (good for drones)
 * FILTER_8:    Heavy filtering
 * FILTER_16:   Maximum filtering (slowest response)
 * 
 * FOR QUADCOPTER: FILTER_4 balances noise reduction and response time
 */
#define BMP280_FILTER_OFF         0x00
#define BMP280_FILTER_2           0x01
#define BMP280_FILTER_4           0x02  // Recommended for drones
#define BMP280_FILTER_8           0x03
#define BMP280_FILTER_16          0x04

/**
 * STANDBY TIME (for NORMAL mode)
 * 
 * WHAT: Time between measurements in NORMAL mode
 * WHY: Controls update rate and power consumption
 * 
 * FOR QUADCOPTER: Use 0.5ms (fastest) for 250Hz control loop
 * Sensor updates faster than we read it, so we always get fresh data
 */
#define BMP280_STANDBY_0_5_MS     0x00  // 0.5ms (2000 Hz max) ← Use this
#define BMP280_STANDBY_62_5_MS    0x01  // 62.5ms (16 Hz)
#define BMP280_STANDBY_125_MS     0x02  // 125ms (8 Hz)
#define BMP280_STANDBY_250_MS     0x03  // 250ms (4 Hz)
#define BMP280_STANDBY_500_MS     0x04  // 500ms (2 Hz)
#define BMP280_STANDBY_1000_MS    0x05  // 1000ms (1 Hz)
#define BMP280_STANDBY_2000_MS    0x06  // 2000ms (0.5 Hz)
#define BMP280_STANDBY_4000_MS    0x07  // 4000ms (0.25 Hz)

//=============================================================================
// DATA STRUCTURES
//=============================================================================

/**
 * CALIBRATION COEFFICIENTS
 * 
 * WHY: These correct for manufacturing variations in each sensor
 * WHERE: Read from sensor EEPROM at startup (registers 0x88-0x9F)
 * 
 * NAMING: "dig" = digital calibration, "T" = temperature, "P" = pressure
 * 
 * HOW USED: In compensation formulas (see BMP280 datasheet section 3.11.3)
 */
typedef struct {
    uint16_t dig_T1;  // Temperature calibration 1 (unsigned)
    int16_t  dig_T2;  // Temperature calibration 2 (signed)
    int16_t  dig_T3;  // Temperature calibration 3 (signed)
    
    uint16_t dig_P1;  // Pressure calibration 1 (unsigned)
    int16_t  dig_P2;  // Pressure calibration 2 (signed)
    int16_t  dig_P3;  // Pressure calibration 3 (signed)
    int16_t  dig_P4;  // Pressure calibration 4 (signed)
    int16_t  dig_P5;  // Pressure calibration 5 (signed)
    int16_t  dig_P6;  // Pressure calibration 6 (signed)
    int16_t  dig_P7;  // Pressure calibration 7 (signed)
    int16_t  dig_P8;  // Pressure calibration 8 (signed)
    int16_t  dig_P9;  // Pressure calibration 9 (signed)
} bmp280_calib_data_t;

/**
 * SENSOR CONFIGURATION
 * 
 * WHY: Stores how we want the sensor to operate
 * WHEN: Set once during initialization
 */
typedef struct {
    uint8_t i2c_addr;              // I2C address (0x76 or 0x77)
    uint8_t temp_oversampling;     // Temperature oversampling (x1, x2, x4, etc.)
    uint8_t press_oversampling;    // Pressure oversampling (x1, x2, x4, etc.)
    uint8_t mode;                  // Power mode (SLEEP, FORCED, NORMAL)
    uint8_t filter;                // IIR filter coefficient (OFF, 2, 4, 8, 16)
    uint8_t standby_time;          // Standby time between measurements
} bmp280_config_t;

/**
 * SENSOR STATE
 * 
 * WHY: Stores everything we need to know about the sensor
 * WHAT: Calibration data, configuration, and current readings
 */
typedef struct {
    bmp280_config_t config;        // Sensor configuration
    bmp280_calib_data_t calib;     // Factory calibration coefficients
    int32_t t_fine;                // Fine temperature value (used in pressure calculation)
    bool initialized;              // Is sensor initialized and ready?
} bmp280_t;

//=============================================================================
// FUNCTION PROTOTYPES
//=============================================================================

/**
 * INITIALIZATION FUNCTIONS
 */

/**
 * @brief Initialize BMP280 sensor with default settings
 * 
 * WHAT IT DOES:
 * 1. Checks if sensor is present (reads chip ID)
 * 2. Reads factory calibration data from sensor
 * 3. Configures sensor (oversampling, filter, mode)
 * 4. Starts continuous measurements
 * 
 * DEFAULT SETTINGS (optimized for quadcopter):
 * - Pressure oversampling: x4 (±0.5 Pa noise)
 * - Temperature oversampling: x1 (we only need temp for pressure compensation)
 * - IIR filter: x4 (balances noise and response)
 * - Mode: NORMAL (continuous measurements)
 * - Standby: 0.5ms (fastest updates)
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @param i2c_addr I2C address (0x76 or 0x77)
 * @return true if initialization successful, false if sensor not found
 */
bool bmp280_init(bmp280_t *sensor, uint8_t i2c_addr);

/**
 * @brief Initialize BMP280 with custom configuration
 * 
 * WHY: For advanced users who want to tune sensor settings
 * WHEN: Use this if default settings don't work for your application
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @param config Pointer to custom configuration
 * @return true if successful, false if failed
 */
bool bmp280_init_with_config(bmp280_t *sensor, const bmp280_config_t *config);

/**
 * READING FUNCTIONS
 */

/**
 * @brief Read raw pressure and temperature from sensor
 * 
 * WHAT IT DOES:
 * 1. Reads 6 bytes from sensor (3 for pressure, 3 for temperature)
 * 2. Combines bytes into 20-bit values
 * 3. Applies factory calibration formulas
 * 4. Returns compensated pressure in Pascals
 * 
 * WHY 2 VALUES: Temperature is needed to compensate pressure reading
 * (pressure sensor is temperature-sensitive)
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @param pressure Pointer to store pressure (Pascals)
 * @param temperature Pointer to store temperature (Celsius)
 * @return true if read successful, false if failed
 */
bool bmp280_read(bmp280_t *sensor, float *pressure, float *temperature);

/**
 * @brief Read only pressure (faster than reading both)
 * 
 * WHY: If you don't need temperature, this saves time
 * NOTE: Still reads temperature internally (needed for compensation)
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @param pressure Pointer to store pressure (Pascals)
 * @return true if successful, false if failed
 */
bool bmp280_read_pressure(bmp280_t *sensor, float *pressure);

/**
 * UTILITY FUNCTIONS
 */

/**
 * @brief Convert pressure to altitude
 * 
 * WHAT: Uses barometric formula to estimate altitude from pressure
 * 
 * FORMULA: altitude = 44330 × (1 - (P/P0)^0.1903)
 * WHERE:
 * - P = current pressure (Pascals)
 * - P0 = sea level pressure (101325 Pa standard)
 * - 44330 = constant from ISA (International Standard Atmosphere)
 * - 0.1903 = 1/5.255 (adiabatic index for air)
 * 
 * ACCURACY: ±1 meter at sea level, ±3 meters at 1000m altitude
 * 
 * NOTE: This gives altitude above sea level, not above ground!
 * For altitude hold, use pressure difference, not absolute altitude
 * 
 * @param pressure Current pressure (Pascals)
 * @param sea_level_pressure Reference pressure at sea level (default 101325 Pa)
 * @return Altitude in meters
 */
float bmp280_pressure_to_altitude(float pressure, float sea_level_pressure);

/**
 * @brief Soft reset sensor
 * 
 * WHY: Resets sensor to power-on state without power cycling
 * WHEN: Use if sensor gets stuck or readings seem wrong
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @return true if successful, false if failed
 */
bool bmp280_reset(bmp280_t *sensor);

/**
 * @brief Check if sensor is measuring
 * 
 * WHY: Sensor takes time to measure (5.5ms for x4 oversampling)
 * Reading too early gives old data
 * 
 * @param sensor Pointer to BMP280 sensor structure
 * @return true if measurement in progress, false if ready to read
 */
bool bmp280_is_measuring(bmp280_t *sensor);

#endif // BMP280_H
