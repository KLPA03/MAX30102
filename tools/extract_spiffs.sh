#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <PORT> <BUILD_DIR> [PARTITION_NAME] [OUT_BIN]" >&2
  echo "Example: $0 /dev/ttyACM0 build storage spiffs.bin" >&2
  exit 2
fi

PORT="$1"
BUILD_DIR="$2"
PART_NAME="${3:-storage}"
OUT_BIN="${4:-spiffs.bin}"

if [[ -z "${IDF_PATH:-}" ]]; then
  echo "IDF_PATH is not set. Run: . \$HOME/esp/esp-idf/export.sh" >&2
  exit 2
fi

PART_TABLE_BIN="${BUILD_DIR%/}/partition_table/partition-table.bin"
if [[ ! -f "${PART_TABLE_BIN}" ]]; then
  echo "Partition table not found at: ${PART_TABLE_BIN}" >&2
  echo "Run: idf.py build" >&2
  exit 2
fi

python3 "${IDF_PATH}/components/partition_table/parttool.py" \
  --port "${PORT}" \
  --partition-table-file "${PART_TABLE_BIN}" \
  read_partition \
  --partition-name "${PART_NAME}" \
  --output "${OUT_BIN}"

echo "Wrote ${OUT_BIN}"

