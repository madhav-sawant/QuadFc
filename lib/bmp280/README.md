# BMP280 Barometric Pressure Sensor Library

## 📋 What This Library Does

This library allows you to read atmospheric pressure and temperature from a BMP280 sensor. This is used for **altitude hold** - making your drone hover at a fixed height automatically.

---

## 🔧 Hardware Setup

### Wiring (I2C Connection)

```
BMP280          ESP32
------          -----
VCC     →       3.3V
GND     →       GND
SCL     →       GPIO 22 (SCL)
SDA     →       GPIO 21 (SDA)
SDO     →       GND (for address 0x76)
CSB     →       3.3V (enables I2C mode)
```

**Important:**
- Use **3.3V**, NOT 5V (BMP280 is 3.3V only!)
- SDO pin sets I2C address:
  - SDO → GND = address **0x76** (most common)
  - SDO → 3.3V = address **0x77**

---

## 📚 How It Works

### 1. Pressure and Altitude

**Physics:**
- Atmospheric pressure decreases as you go higher
- At sea level: ~101325 Pa (1013.25 hPa)
- Every 8cm up = ~1 Pa decrease
- Every 100m up = ~1200 Pa decrease

**For Altitude Hold:**
- We DON'T care about absolute altitude
- We only care about **pressure difference**
- Example:
  - Start pressure: 101325 Pa
  - Current pressure: 101320 Pa
  - Difference: -5 Pa
  - Altitude change: +40cm (drone went up 40cm)

### 2. Why We Need Temperature

**Problem:** Pressure sensor is temperature-sensitive
**Solution:** BMP280 also measures temperature and uses it to compensate pressure reading

**You get:**
- Accurate pressure (compensated for temperature)
- Bonus: ambient temperature reading

---

## 💻 How to Use

### Step 1: Initialize I2C

```c
#include "driver/i2c.h"

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,  // 100kHz
};
i2c_param_config(I2C_NUM_0, &conf);
i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
```

### Step 2: Initialize BMP280

```c
#include "bmp280.h"

bmp280_t sensor;

// Try address 0x76 (most common)
if (!bmp280_init(&sensor, BMP280_I2C_ADDR_PRIMARY)) {
    printf("Sensor not found at 0x76, trying 0x77...\n");
    
    // Try alternate address 0x77
    if (!bmp280_init(&sensor, BMP280_I2C_ADDR_SECONDARY)) {
        printf("ERROR: BMP280 not found!\n");
        return;
    }
}

printf("BMP280 initialized!\n");
```

### Step 3: Read Pressure and Temperature

```c
float pressure, temperature;

if (bmp280_read(&sensor, &pressure, &temperature)) {
    printf("Pressure: %.2f Pa\n", pressure);
    printf("Temperature: %.2f °C\n", temperature);
} else {
    printf("Read failed!\n");
}
```

### Step 4: Calculate Altitude Change

```c
// Save initial pressure (when drone is on ground)
float ground_pressure = 101325.0f;  // Read this at startup

// During flight, calculate altitude change
float current_pressure = 101320.0f;  // Read from sensor
float altitude = bmp280_pressure_to_altitude(current_pressure, ground_pressure);

printf("Altitude: %.2f meters above ground\n", altitude);
```

---

## 🎯 Configuration Options

### Default Settings (Optimized for Drones)

```c
bmp280_init(&sensor, 0x76);  // Uses these defaults:
```

- **Pressure oversampling:** x4 (±0.5 Pa noise, 5.5ms read time)
- **Temperature oversampling:** x1 (fast, only needed for compensation)
- **IIR filter:** x4 (removes vibration noise)
- **Mode:** NORMAL (continuous measurements)
- **Standby time:** 0.5ms (fastest updates)

### Custom Settings

```c
bmp280_config_t config = {
    .i2c_addr = 0x76,
    .press_oversampling = BMP280_OVERSAMPLING_X8,  // More accurate
    .temp_oversampling = BMP280_OVERSAMPLING_X1,
    .filter = BMP280_FILTER_8,                     // More filtering
    .mode = BMP280_MODE_NORMAL,
    .standby_time = BMP280_STANDBY_0_5_MS,
};

bmp280_init_with_config(&sensor, &config);
```

---

## 📊 Understanding the Numbers

### Pressure

- **Unit:** Pascals (Pa)
- **Range:** 30000 - 110000 Pa
- **Sea level:** ~101325 Pa (1013.25 hPa)
- **Noise:** ±0.5 Pa with x4 oversampling
- **Resolution:** 0.16 Pa (≈1.3cm altitude)

**Conversions:**
- 1 hPa = 100 Pa (hectopascal, weather reports)
- 1 bar = 100000 Pa
- 1 atm = 101325 Pa

### Temperature

- **Unit:** Celsius (°C)
- **Range:** -40°C to +85°C
- **Accuracy:** ±1°C
- **Resolution:** 0.01°C

### Altitude

- **Accuracy:** ±1 meter (at sea level)
- **Resolution:** ~8cm per Pascal
- **Note:** Relative altitude is more accurate than absolute

---

## 🔍 Troubleshooting

### Sensor Not Found

**Problem:** `bmp280_init()` returns false

**Solutions:**
1. Check wiring (especially VCC and GND)
2. Verify I2C address (try both 0x76 and 0x77)
3. Run I2C scanner to detect devices
4. Check if SDO pin is connected correctly
5. Verify CSB pin is HIGH (3.3V) for I2C mode

### Noisy Readings

**Problem:** Pressure jumps around ±10 Pa

**Solutions:**
1. Increase oversampling: `BMP280_OVERSAMPLING_X8`
2. Increase filter: `BMP280_FILTER_8` or `BMP280_FILTER_16`
3. Add foam over sensor (dampen vibrations)
4. Mount sensor away from motors
5. Use software filtering (see altitude_control library)

### Wrong Altitude

**Problem:** Altitude reading is way off

**Solutions:**
1. Use **relative** altitude, not absolute
2. Save ground pressure at startup
3. Don't use `bmp280_pressure_to_altitude()` with standard sea level pressure
4. Account for weather changes (pressure changes throughout day)

### Slow Updates

**Problem:** Readings don't update fast enough

**Solutions:**
1. Use `BMP280_MODE_NORMAL` (continuous mode)
2. Set `BMP280_STANDBY_0_5_MS` (fastest)
3. Lower oversampling (x2 or x1 for speed)
4. Check if `bmp280_is_measuring()` before reading

---

## 📐 Technical Details

### Measurement Time

| Oversampling | Pressure | Temperature | Total Time |
|--------------|----------|-------------|------------|
| x1           | 2.3 ms   | 2.3 ms      | 4.6 ms     |
| x2           | 3.6 ms   | 2.3 ms      | 5.9 ms     |
| x4           | 6.2 ms   | 2.3 ms      | 8.5 ms     |
| x8           | 11.4 ms  | 2.3 ms      | 13.7 ms    |
| x16          | 21.8 ms  | 2.3 ms      | 24.1 ms    |

**For 250Hz loop:** Use x4 or lower (8.5ms fits in 4ms loop with buffering)

### Noise Performance

| Oversampling | RMS Noise | Altitude Noise |
|--------------|-----------|----------------|
| x1           | 2.0 Pa    | ±16 cm         |
| x2           | 1.0 Pa    | ±8 cm          |
| x4           | 0.5 Pa    | ±4 cm          |
| x8           | 0.3 Pa    | ±2.4 cm        |
| x16          | 0.2 Pa    | ±1.6 cm        |

**Recommendation:** x4 is best balance for drones

---

## 🚀 Next Steps

1. **Test the library:** Run `bmp280_example.c` to verify sensor works
2. **Add filtering:** Implement dual-filter (see Joop Brokking's method)
3. **Create altitude PID:** Use filtered pressure for altitude hold
4. **Integrate with main:** Add to flight controller main loop

---

## 📖 References

- [BMP280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [Barometric Formula](https://en.wikipedia.org/wiki/Barometric_formula)
- [I2C Protocol](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

---

## ✅ Summary

**What you learned:**
- How BMP280 measures pressure
- Why we need temperature compensation
- How to convert pressure to altitude
- I2C communication basics
- Sensor configuration trade-offs

**What you can do now:**
- Read pressure and temperature
- Calculate altitude changes
- Tune sensor for your needs
- Debug sensor issues

**Next:** Implement altitude hold PID controller!
