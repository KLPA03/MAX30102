@echo off
setlocal enabledelayedexpansion

REM Usage: tools\extract_spiffs.cmd COM5 build storage spiffs.bin

if "%~1"=="" goto :usage
if "%~2"=="" goto :usage

set PORT=%~1
set BUILD_DIR=%~2
set PART_NAME=%~3
set OUT_BIN=%~4

if "%PART_NAME%"=="" set PART_NAME=storage
if "%OUT_BIN%"=="" set OUT_BIN=spiffs.bin

if "%IDF_PATH%"=="" (
  echo IDF_PATH is not set. Run from "ESP-IDF Command Prompt" or call export.bat first.
  exit /b 2
)

set PART_TABLE_BIN=%BUILD_DIR%\partition_table\partition-table.bin
if not exist "%PART_TABLE_BIN%" (
  echo Partition table not found at: "%PART_TABLE_BIN%"
  echo Run: idf.py build
  exit /b 2
)

python "%IDF_PATH%\components\partition_table\parttool.py" ^
  --port "%PORT%" ^
  --partition-table-file "%PART_TABLE_BIN%" ^
  read_partition ^
  --partition-name "%PART_NAME%" ^
  --output "%OUT_BIN%"

echo Wrote %OUT_BIN%
exit /b 0

:usage
echo Usage: %~nx0 ^<PORT^> ^<BUILD_DIR^> [PARTITION_NAME] [OUT_BIN]
echo Example: %~nx0 COM5 build storage spiffs.bin
exit /b 2

