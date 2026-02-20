@echo off
setlocal enabledelayedexpansion

REM Usage: tools\unpack_spiffs.cmd spiffs.bin spiffs_dump 256 4096 32

if "%~1"=="" goto :usage
if "%~2"=="" goto :usage

set SPIFFS_BIN=%~1
set OUT_DIR=%~2
set PAGE_SIZE=%~3
set BLOCK_SIZE=%~4
set OBJ_NAME_LEN=%~5

if "%PAGE_SIZE%"=="" set PAGE_SIZE=256
if "%BLOCK_SIZE%"=="" set BLOCK_SIZE=4096
if "%OBJ_NAME_LEN%"=="" set OBJ_NAME_LEN=32

if "%IDF_PATH%"=="" (
  echo IDF_PATH is not set. Run from "ESP-IDF Command Prompt" or call export.bat first.
  exit /b 2
)

if not exist "%SPIFFS_BIN%" (
  echo SPIFFS image not found: "%SPIFFS_BIN%"
  exit /b 2
)

python "%IDF_PATH%\components\spiffs\spiffsgen.py" ^
  --page-size "%PAGE_SIZE%" ^
  --block-size "%BLOCK_SIZE%" ^
  --obj-name-len "%OBJ_NAME_LEN%" ^
  --unpack "%SPIFFS_BIN%" ^
  "%OUT_DIR%"

echo Unpacked into %OUT_DIR%
exit /b 0

:usage
echo Usage: %~nx0 ^<SPIFFS_BIN^> ^<OUT_DIR^> [PAGE_SIZE] [BLOCK_SIZE] [OBJ_NAME_LEN]
echo Example: %~nx0 spiffs.bin spiffs_dump 256 4096 32
exit /b 2

