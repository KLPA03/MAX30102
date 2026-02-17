# MAX30102

## Modified Initialization Code for Heart Rate with Red & IR Reading (20 Hz Sampling)

```c
/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    max30102_write(0x09, 0x40);   // Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x08, 0x0F);  // FIFO Config: No averaging, FIFO rollover enabled
    max30102_write(0x09, 0x03);  // SpO2 mode (enables both Red and IR LEDs for heart rate)
    max30102_write(0x0A, 0x07);  // 50 Hz sampling, 411μs pulse width, 2048 nA range
    max30102_write(0x0C, 0x1F);  // Red LED: 6.4 mA (for heart rate)
    max30102_write(0x0D, 0x1F);  // IR LED: 6.4 mA (for heart rate)

    ESP_LOGI(TAG, "MAX30102 initialized for heart rate with Red & IR at 50 Hz (decimate to 20 Hz in software)");
}
```

### Changes Made:
- **Register 0x08**: `0x0F` - No FIFO averaging, rollover enabled
- **Register 0x09**: `0x03` - SpO2 mode (enables both Red and IR for heart rate)
- **Register 0x0A**: `0x07` - 50 Hz sampling (closest to 20 Hz), 411μs pulse width
- **Register 0x0C**: `0x1F` - Red LED at 6.4 mA (enabled)
- **Register 0x0D**: `0x1F` - IR LED at 6.4 mA (enabled)

### Achieving Exactly 20 Hz:
**Hardware Limitation**: MAX30102 base sampling rates are 50, 100, 200, 400, 800, 1000, 1600, 3200 Hz. FIFO averaging only supports powers of 2 (1, 2, 4, 8, 16, 32). No combination produces exactly 20 Hz.

**Solution**: 
- Hardware samples at **50 Hz** (closest available)
- **Software decimation**: Read every 2.5th sample OR average 5 samples over 250ms window to achieve 20 Hz effective rate
- Alternative: Use 100 Hz hardware rate and decimate by 5 in software

### Both Red and IR Enabled:
- SpO2 mode (0x03) enables both LEDs alternately
- Each LED pulse is captured for heart rate analysis
- Both channels available in FIFO for processing
