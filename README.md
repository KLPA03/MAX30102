# ESP32-S3 MAX3010x Logger (USB Mass Storage)

ESP-IDF v5.x project for ESP32-S3 that:

- Reads **MAX3010x raw RED/IR** samples at **100 Hz** over I2C (MAX30102 PART_ID is typically `0x15`)
- Downsamples to **20 Hz** (averages 5 samples)
- Writes `log.csv` with header `time_ms,IR,RED`
- Optionally logs converted photodiode current in picoamps (pA) for larger “real” numbers
- Stores CSV on a **flash-backed FAT** partition used for **USB Mass Storage (MSC)** (`/data/log.csv`)
- **Pauses file logging automatically** while the USB host (PC) owns the MSC storage, and **resumes** when the host safely ejects it

## Hardware wiring

- **Power**: ESP32-S3 is powered from PC USB.
- **Sensor**: MAX3010x (MAX30101/MAX30102) via I2C, powered from ESP32-S3 **3.3 V** with common GND.
- **I2C** (defaults, recommended for ESP32-S3 DevKitC-1):
  - **SDA**: GPIO8
  - **SCL**: GPIO9
  - Add proper pull-ups (most MAX3010x breakout boards already have them).

If your board uses different pins or needs a slower I2C clock, configure it via:

- `idf.py menuconfig` -> **MAX3010x Logger** -> **I2C (MAX3010x)**

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

In `idf.py menuconfig` -> **MAX3010x Logger** -> **Logging**, you can choose:

- **Raw counts** (18-bit, max 262143)
- **pA** (converted photodiode current, typically 7 digits at higher signal levels)

## Using one USB cable (CDC + MSC)

This firmware enumerates on the **DevKitC-1 USB (native/OTG) port** as:

- **USB Serial (CDC)**: shows up as a COM port (use for `idf.py monitor`)
- **USB Mass Storage (MSC)**: shows up as a removable drive containing `log.csv`

Notes:

- **Flashing and monitoring are on different COM ports** on Windows:
  - Flashing uses the ROM download port (often shows as "USB JTAG/serial").
  - Monitoring uses the CDC COM port created by the application.
- Logging pauses while the PC has the drive mounted. Use Windows **Eject** to resume logging.

## Recording mode switch (RESET button)

This firmware uses the **RESET** button as a simple on/off switch by toggling a persistent flag on every boot:

- **Recording ON (LED green)**:
  - Records and appends to `/data/log.csv`
  - **Hides the MSC disk from the PC** (so `log.csv` does not appear and recording never pauses)
- **Recording OFF (LED red)**:
  - Stops recording
  - **Exposes the MSC disk to the PC** so you can copy `log.csv`

Status LED configuration is under:

- `idf.py menuconfig` -> **MAX3010x Logger** -> **Status LED**

If your LED is “embedded/on-board”, select a **Board LED preset**.
For ESP32-S3 DevKitC-1, note the board revision:

- **v1.0**: WS2812 on **GPIO48**
- **v1.1**: WS2812 on **GPIO38**

If your board isn’t listed, choose **Custom** and then select the LED type/pins.
