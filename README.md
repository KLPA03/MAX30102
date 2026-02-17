# MAX30102

Minimal ESP-IDF project for MAX30102 FIFO reads and SPIFFS CSV logging.

## Notes

- **I2C**: defaults to `SDA=5`, `SCL=4`, `400kHz`, device address `0x57`
- **SPIFFS**: logs to `/spiffs/data.csv`
- **Partition table**: a `spiffs` partition is provided in `partitions.csv`
