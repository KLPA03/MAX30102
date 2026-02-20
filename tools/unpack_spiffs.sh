#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <SPIFFS_BIN> <OUT_DIR> [PAGE_SIZE] [BLOCK_SIZE] [OBJ_NAME_LEN]" >&2
  echo "Example: $0 spiffs.bin spiffs_dump 256 4096 32" >&2
  exit 2
fi

SPIFFS_BIN="$1"
OUT_DIR="$2"
PAGE_SIZE="${3:-256}"
BLOCK_SIZE="${4:-4096}"
OBJ_NAME_LEN="${5:-32}"

if [[ -z "${IDF_PATH:-}" ]]; then
  echo "IDF_PATH is not set. Run: . \$HOME/esp/esp-idf/export.sh" >&2
  exit 2
fi

python3 "${IDF_PATH}/components/spiffs/spiffsgen.py" \
  --page-size "${PAGE_SIZE}" \
  --block-size "${BLOCK_SIZE}" \
  --obj-name-len "${OBJ_NAME_LEN}" \
  --unpack "${SPIFFS_BIN}" \
  "${OUT_DIR}"

echo "Unpacked into ${OUT_DIR}"

