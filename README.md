# MAX30102 (ESP32-S3 + ESP-IDF)

This is a minimal ESP-IDF project that reads MAX30102 FIFO samples at **100 Hz** (sensor configuration) and logs the **latest** sample every **50 ms** (**20 Hz output**) to a CSV file on **SPIFFS**.

## Build / Flash

1. Install ESP-IDF and export the environment.
2. Configure/build/flash:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Files

- `main/main.c`: MAX30102 I2C + SPIFFS CSV logger (20 Hz output)
- `partitions.csv`: custom partition table including a SPIFFS partition
- `sdkconfig.defaults`: enables SPIFFS + custom partitions on first build