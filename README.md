# MAX30102

## Modified Initialization Code for Heart Rate Mode (20 Hz Sampling)

```c
/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    max30102_write(0x09, 0x40);   // Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x09, 0x02);  // Heart Rate mode only (not SpO2)
    max30102_write(0x0A, 0x0F);  // 50 Hz sampling (closest to 20Hz), 411μs pulse width
    max30102_write(0x0C, 0x1F);  // Red LED: 6.4 mA (for heart rate)
    max30102_write(0x0D, 0x00);  // IR LED: disabled (not needed for HR mode)

    ESP_LOGI(TAG, "MAX30102 initialized in Heart Rate mode");
}
```

### Changes Made:
- **Register 0x09**: Changed from `0x03` (SpO2) to `0x02` (Heart Rate only)
- **Register 0x0A**: Changed from `0x27` to `0x0F` (50 Hz sampling - closest to 20 Hz)
- **Register 0x0D**: Changed from `0x1F` to `0x00` (IR LED disabled for HR-only mode)
- Added reset command (`0x40`) at initialization

Note: MAX30102 hardware doesn't support exactly 20 Hz. Available rates are 50/100/167/200/400/600/800/1000 Hz. Using 50 Hz as closest option.
