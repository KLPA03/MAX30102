# MAX30102 (ESP32-S3 + ESP-IDF)

This is a minimal ESP-IDF project that reads MAX30102 FIFO samples at **100 Hz** (sensor configuration) and logs the **latest** sample every **50 ms** (**20 Hz output**) to a CSV file on a **flash-backed FAT filesystem**.

That same FAT partition is exposed as a **USB Mass Storage Class (MSC)** device (a “USB drive”) using TinyUSB.

## Build / Flash

1. Install ESP-IDF and export the environment.
2. Configure/build/flash:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Files

- `main/main.c`: MAX30102 I2C CSV logger + TinyUSB MSC (flash FAT partition)
- `partitions.csv`: custom partition table including a `data,fat` partition (`storage`)
- `sdkconfig.defaults`: enables TinyUSB MSC + FAT/wear-levelling + custom partitions

## Notes

- The logger **only writes** while the storage is mounted to the **application**.
- When a USB host (PC) mounts the drive, logging **pauses** until you **eject** the drive on the PC (so the ESP can safely remount it).