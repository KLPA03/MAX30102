# ESP32-S3 MAX3010x Logger (USB Mass Storage)

ESP-IDF v5.x project for ESP32-S3 that:

- Reads **MAX3010x raw RED/IR** samples at **100 Hz** over I2C (MAX30102 PART_ID is typically `0x15`)
- Downsamples to **20 Hz** (averages 5 samples)
- Writes `log.csv` with header `time_ms,IR,RED`
- Stores CSV on a **flash-backed FAT** partition used for **USB Mass Storage (MSC)** (`/data/log.csv`)
- **Pauses file logging automatically** while the USB host (PC) owns the MSC storage, and **resumes** when the host safely ejects it

## Hardware wiring

- **Power**: ESP32-S3 is powered from PC USB.
- **Sensor**: MAX3010x (MAX30101/MAX30102) via I2C, powered from ESP32-S3 **3.3 V** with common GND.
- **I2C** (default in `main/main.c`, recommended for ESP32-S3 DevKitC-1):
  - **SDA**: GPIO8
  - **SCL**: GPIO9
  - Add proper pull-ups (most MAX30102 breakout boards already have them).

If your board uses different pins, edit these defines in `main/main.c`:

- `MAX30102_I2C_SDA_GPIO`
- `MAX30102_I2C_SCL_GPIO`

## USB MSC behavior (important)

USB MSC and the firmware **must not access the same FAT filesystem at the same time**.

This project uses the `esp_tinyusb` MSC storage “mount point” switching:

- When the **PC has the drive** (MSC mode), firmware **does not write files**.
- After you **Safely eject** the drive on the PC, the firmware mounts the FAT volume for the app and **resumes logging**.
- To read `log.csv` again on the PC, **unplug/replug USB** (most OSes require re-enumeration after an eject).

## Build & flash

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

The project uses:

- Custom partition table: `partitions.csv`
- Component Manager dependency: `main/idf_component.yml` (pulls `espressif/esp_tinyusb` automatically)

## Output (on the USB drive)

- `log.csv`
