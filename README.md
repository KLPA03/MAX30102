# MAX30102 SPIFFS CSV Logger (ESP-IDF)

This project logs MAX30102 `IR` + `RED` samples to a CSV file stored in SPIFFS at `/spiffs/data.csv`.

## What was fixed vs the original snippet

- CSV is opened with `"a+"` and the header is written **only if the file is new/empty**
- SPIFFS mount is checked and `storage` partition is explicitly selected
- Write errors are logged; flush+fsync is done periodically (once per second) for durability
- Downsampling to 20 Hz now averages all FIFO samples available each 50 ms window (simple box filter)

## Build / flash

```bash
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py fullclean
idf.py build flash monitor
```

If you see `FAILED: partition_table/partition-table.bin`, it usually means your partition table didn’t fit your configured flash size or overlaps. This repo’s `partitions.csv` is sized to fit on small flash (including 2MB), but you must `fullclean` after changing partition settings.

Note: in `partitions.csv`, if you want ESP-IDF to auto-place a partition offset, you must leave the **Offset** column empty by using a double comma, e.g. `spiffs,,0x80000`.

## Extract `data.csv` from SPIFFS (from a flashed device)

Prereqs:

- You already ran `idf.py build` (so `build/partition_table/partition-table.bin` exists)
- `IDF_PATH` is set (from `export.sh`)

### 1) Read SPIFFS partition from the device into `spiffs.bin`

```bash
./tools/extract_spiffs.sh /dev/ttyACM0 build storage spiffs.bin
```

### 2) Unpack the SPIFFS image into a local folder

The unpack parameters must match your SPIFFS config. This repo defaults to:

- page size: 256
- block size: 4096
- object name length: 32

```bash
./tools/unpack_spiffs.sh spiffs.bin spiffs_dump 256 4096 32
ls -ლა spiffs_dump
```

Your CSV will be at `spiffs_dump/data.csv`.
