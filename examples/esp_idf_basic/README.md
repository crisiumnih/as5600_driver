# ESP-IDF Basic Example

ESP-IDF example demonstrating AS5600 driver usage.

**Status: Tested and working on ESP32**

## Hardware Setup

### Connections
```
ESP32        AS5600
-----        ------
3.3V   --->  VCC
GND    --->  GND
GPIO21 --->  SDA
GPIO22 --->  SCL
```

### Magnet
- Place a diametrically magnetized magnet above the AS5600 sensor
- Distance: 0.5-3mm from sensor surface
- Magnet diameter: 4-6mm recommended

## Expected Output

```
I (320) AS5600_Example: === AS5600 ESP-IDF Example ===
I (330) AS5600_Example: Initializing I2C...
I (330) AS5600_Example: I2C initialized
I (340) AS5600_Example: Initializing AS5600...
I (350) AS5600_Example: AS5600 initialized successfully
I (360) AS5600_Example: Magnet position: OK
I (360) AS5600_Example: Starting angle reading...

I (470) AS5600_Example: Raw:  512 | Deg:  45.00° | Norm: 0.1250 | Delta:    +5
I (570) AS5600_Example: Raw:  517 | Deg:  45.44° | Norm: 0.1262 | Delta:    +5
I (670) AS5600_Example: Raw:  520 | Deg:  45.70° | Norm: 0.1270 | Delta:    +3
...
```
