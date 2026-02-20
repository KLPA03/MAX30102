# MAX30102

ESP-IDF project that:

- logs `IR`, `RED`, `BPM`, `AVG_BPM` to `/spiffs/data.csv`
- **outputs at 20 Hz** (50 ms period) while draining MAX30102 FIFO so it doesn't overflow
- keeps MAX30102 configured at **100 Hz** internally and downsamples logs to **20 Hz**

## Build

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

## Notes

- The sensor is configured for **100 Hz internal sample rate**; the firmware drains FIFO and keeps the newest sample each 50 ms to produce a **20 Hz** log/processing rate.

## Extracting `data.csv` from SPIFFS (offline)

Your SPIFFS partition size in `partitions.csv` is **0x110000**. When unpacking, the size passed to `spiffsgen.py` must match the partition size.

Example (PowerShell):

```bash
python $env:IDF_PATH\components\partition_table\parttool.py --port COM6 read_partition --partition-name spiffs --output spiffs.bin
python $env:IDF_PATH\components\spiffs\spiffsgen.py --unpack 0x110000 spiffs.bin extracted
```