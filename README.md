# AS5600 Driver

C++ driver for AS5600 12-bit magnetic rotary encoder

## Features

- 12-bit absolute angle measurement (0-4095)
- Multiple read modes: raw, degrees, normalized (0-1)
- Delta tracking with wrap-around handling
- Magnet detection and diagnostics
- Configurable filtering (slow filter + fast filter threshold)
- Non-volatile configuration programming (ZPOS, MPOS, MANG)
- Software zero offset
- Hardware-agnostic I2C interface

## Hardware

**AS5600** - 12-bit contactless magnetic angle sensor
- I2C address: `0x36` (7-bit)
- Supply voltage: 3.3V or 5V
- Resolution: 0.088° (12-bit)
- Interface: I2C (up to 1 MHz)

## Configuration

Default initialization (`Init()`):
- Hysteresis: 1 LSB
- Slow Filter: 8x (SF_8X)
- Fast Filter Threshold: 7 LSB (FTH_7_LSB)
- Watchdog: OFF

## Usage

### 1. Implement Platform I2C Functions

```cpp
AS5600_RET_TYPE my_i2c_read(void* handle, uint8_t addr, uint8_t reg, 
                             uint8_t* buf, uint8_t len) {
    // Your platform I2C read implementation
    return AS5600_RET_OK;
}

AS5600_RET_TYPE my_i2c_write(...) {
}

AS5600_RET_TYPE my_delay_ms(...) {
}
```

### 2. Initialize Driver

```cpp
#include "as5600.hpp"

void* i2c_handle = /* your I2C peripheral handle */;

AS5600 encoder(i2c_handle, my_i2c_read, my_i2c_write, my_delay_ms);

if (encoder.Init() == AS5600_RET_OK) {
    // Encoder ready
}
```

### 3. Read Angle

```cpp
// Raw 12-bit value (0-4095)
uint16_t raw;
encoder.ReadRawAngle(&raw);

// Degrees (0-360)
float degrees;
encoder.ReadAngleDeg(&degrees);

// Normalized (0.0-1.0)
float normalized;
encoder.ReadAngle01(&normalized);

// Delta (relative motion)
int16_t delta = encoder.GetDelta();  // -2048 to +2047
```

### 4. Magnet Detection

```cpp
if (encoder.MagnetDetected()) {
    if (encoder.MagnetTooWeak()) {
        // Move magnet closer
    } else if (encoder.MagnetTooStrong()) {
        // Move magnet away
    } else {
        // Perfect position
    }
}
```

### 5. Configure Angular Range (Optional)

```cpp
// Set start position (0-4095)
encoder.SetZeroPosition(100);

// Set stop position (0-4095)
encoder.SetMaxPosition(3000);

// Or set angular range instead
encoder.SetMaxAngle(2900);  // Range must be > 18°

// Permanently program (can only be done 3 times!)
if (encoder.BurnAngle() == AS5600_RET_OK) {
    // Verify
    encoder.VerifyBurnAngle(100, 3000);
}
```

### 6. Software Zero Offset

```cpp
// Set current position as zero
uint16_t current;
encoder.ReadRawAngle(&current);
encoder.SetSoftwareZero(current);

// Read with offset applied
uint16_t offset_angle;
encoder.ReadRawAngleWithOffset(&offset_angle);
```

## API Reference

### Initialization
- `AS5600_RET_TYPE Init()` - Initialize with default settings
- `AS5600_RET_TYPE IsPresent()` - Check if sensor responds

### Reading Angle
- `AS5600_RET_TYPE ReadRawAngle(uint16_t* angle)` - Raw 12-bit value
- `AS5600_RET_TYPE ReadRawAngleWithOffset(uint16_t* angle)` - With software offset
- `AS5600_RET_TYPE ReadAngleDeg(float* deg)` - Angle in degrees
- `AS5600_RET_TYPE ReadAngle01(float* normalized)` - Normalized 0.0-1.0
- `int16_t GetDelta()` - Relative motion since last read

### Magnet Status
- `bool MagnetDetected()` - Magnet detected
- `bool MagnetTooWeak()` - AGC maximum gain overflow
- `bool MagnetTooStrong()` - AGC minimum gain overflow

### Configuration
- `AS5600_RET_TYPE SetHysteresis(uint8_t val)` - 0-3 (2-bit)
- `AS5600_RET_TYPE SetSlowFilter(SlowFilter sf)` - SF_16X, SF_8X, SF_4X, SF_2X
- `AS5600_RET_TYPE SetFastFilter(FastFilterThreshold fth)` - FTH_SLOW_ONLY to FTH_24_LSB
- `AS5600_RET_TYPE SetWatchdog(bool enable)` - Enable/disable watchdog timer

### Angular Range Programming
- `AS5600_RET_TYPE SetZeroPosition(uint16_t raw)` - Start position
- `AS5600_RET_TYPE SetMaxPosition(uint16_t raw)` - Stop position
- `AS5600_RET_TYPE SetMaxAngle(uint16_t raw)` - Angular range
- `AS5600_RET_TYPE GetZeroPosition(uint16_t* raw)` - Read ZPOS
- `AS5600_RET_TYPE GetMaxPosition(uint16_t* raw)` - Read MPOS
- `AS5600_RET_TYPE GetMaxAngle(uint16_t* raw)` - Read MANG

### Non-Volatile Programming
- `AS5600_RET_TYPE BurnAngle()` - Permanently program ZPOS/MPOS (max 3 times)
- `AS5600_RET_TYPE VerifyBurnAngle(uint16_t zpos, uint16_t mpos)` - Verify programming

### Software Zero
- `void SetSoftwareZero(uint16_t raw)` - Set software offset

## Return Codes

```cpp
AS5600_RET_OK         // Success
AS5600_RET_ERR        // Generic error
AS5600_RET_BUSY       // Device busy
AS5600_RET_NO_MAGNET  // Magnet not detected
AS5600_RET_OTP_FULL   // OTP already burned 3 times
AS5600_RET_I2C_FAIL   // I2C communication failed
```

## File Structure

```
as5600_driver/
├── inc/
│   ├── as5600.hpp         # Main driver class
│   ├── as5600_reg.hpp     # Register definitions
│   └── as5600_port.hpp    # Return codes & typedefs
└── src/
    └── as5600.cpp         # Implementation
```

## License

MIT License - See LICENSE file for details

## Author

crisiumnih

## Datasheet

[AS5600 Datasheet](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf)
