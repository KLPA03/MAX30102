# MAX30102 USB Logging (ESP-IDF + TinyUSB MSC)

This project logs raw MAX30102 FIFO samples (IR, RED) to a CSV file on a FAT partition and exposes that partition as a **USB Mass Storage (MSC)** device.

## Hardware

- **MCU**: ESP chip with USB-OTG peripheral (for MSC), e.g. **ESP32-S2 / ESP32-S3 / ESP32-P4**
- **Sensor**: MAX30102 on I2C
- **I2C pins** (default): SDA = GPIO5, SCL = GPIO4

## What it does

- When **not connected** to a USB host, the filesystem is mounted to the **application** and the firmware appends to `"/data/data.csv"`.
- When a USB host connects and configures the device, TinyUSB automatically switches the storage to **USB ownership** (the app stops writing), so your PC can mount the drive safely.
- On USB disconnect, the storage is automatically mounted back to the **application**, and logging continues.

## Build & flash

This is an ESP-IDF project using the IDF Component Manager to pull `espressif/esp_tinyusb`.

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

If you use a different chip, pick the appropriate target (e.g. `esp32s2`).

## Output

- CSV file: `BASE_PATH "/data.csv"` (default: `/data/data.csv`)
- Columns: `time_ms,IR,RED,BPM,AVG_BPM`
