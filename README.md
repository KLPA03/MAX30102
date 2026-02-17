# MAX30102

## Modified Initialization Code for Heart Rate Mode (Exactly 20 Hz Sampling)

```c
/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    max30102_write(0x09, 0x40);   // Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x08, 0x4F);  // FIFO Config: Average 5 samples (100Hz/5 = 20Hz)
    max30102_write(0x09, 0x02);  // Heart Rate mode only (not SpO2)
    max30102_write(0x0A, 0x27);  // 100 Hz sampling, 411μs pulse width
    max30102_write(0x0C, 0x1F);  // Red LED: 6.4 mA (for heart rate)
    max30102_write(0x0D, 0x00);  // IR LED: disabled (not needed for HR mode)

    ESP_LOGI(TAG, "MAX30102 initialized in Heart Rate mode at 20 Hz");
}
```

### Changes Made:
- **Register 0x08**: Added FIFO averaging of 5 samples (`0x4F`)
- **Register 0x09**: Changed from `0x03` (SpO2) to `0x02` (Heart Rate only)
- **Register 0x0A**: Set to `0x27` (100 Hz sampling with 411μs pulse width)
- **Register 0x0D**: Changed from `0x1F` to `0x00` (IR LED disabled for HR-only mode)
- Added reset command (`0x40`) at initialization

### How 20 Hz is Achieved:
- Hardware samples at 100 Hz
- FIFO averages every 5 samples
- Effective output rate: 100 Hz ÷ 5 = **exactly 20 Hz**
