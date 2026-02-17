# MAX30102 Heart Rate Monitor

ESP-IDF project for heart rate monitoring using the MAX30102 sensor.

## Configuration

The MAX30102 is configured for **Heart Rate mode only** (not SpO2):

- **Mode**: Heart Rate only (Register 0x09 = 0x02)
- **Sampling Rate**: 50 Hz (closest available to requested 20 Hz)
  - Note: MAX30102 supports 50/100/167/200/400/600/800/1000 Hz
  - Reading from sensor at 20 Hz in software (50ms intervals)
- **Pulse Width**: 411 μs
- **LED Configuration**:
  - Red LED (LED1): 6.4 mA amplitude - used for heart rate detection
  - IR LED (LED2): 0 mA amplitude - disabled (not needed for heart rate only)

## Hardware Connections

- SDA: GPIO 21
- SCL: GPIO 22
- I2C Address: 0x57
- I2C Frequency: 400 kHz

## Building and Flashing

```bash
idf.py build
idf.py flash monitor
```

## Register Configuration Details

| Register | Value | Description |
|----------|-------|-------------|
| 0x09 | 0x40 | Reset sensor |
| 0x09 | 0x02 | Heart Rate mode (not SpO2) |
| 0x0A | 0x0F | 50Hz sampling, 411μs pulse, 16-bit resolution |
| 0x0C | 0x1F | Red LED amplitude (6.4 mA) |
| 0x0D | 0x00 | IR LED disabled |
