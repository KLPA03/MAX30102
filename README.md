# MAX30102

## Modified Initialization Code for Heart Rate with IR Reading (20 Hz Target)

```c
/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    max30102_write(0x09, 0x40);   // Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x08, 0x4F);  // FIFO Config: Average 4 samples, FIFO rollover enabled
    max30102_write(0x09, 0x03);  // SpO2 mode (enables both Red and IR LEDs)
    max30102_write(0x0A, 0x47);  // 100 Hz sampling, 411μs pulse width, 4096 nA range
    max30102_write(0x0C, 0x1F);  // Red LED: 6.4 mA
    max30102_write(0x0D, 0x1F);  // IR LED: 6.4 mA (enabled for reading)

    ESP_LOGI(TAG, "MAX30102 initialized with IR reading at ~20 Hz");
}
```

### Changes Made:
- **Register 0x08**: `0x4F` - FIFO averaging of 4 samples + rollover enabled
- **Register 0x09**: Kept as `0x03` (SpO2 mode - enables both Red and IR)
- **Register 0x0A**: `0x47` - 100 Hz sampling, 411μs pulse width
- **Register 0x0C**: `0x1F` - Red LED at 6.4 mA
- **Register 0x0D**: `0x1F` - IR LED at 6.4 mA (enabled for reading)
- Added reset command (`0x40`) at initialization

### Sampling Rate Configuration:
- Hardware samples at 100 Hz
- FIFO averages every 4 samples
- Effective output rate: 100 Hz ÷ 4 = **25 Hz**

**Note**: MAX30102 hardware limitations prevent exactly 20 Hz. The FIFO averaging only supports powers of 2 (1, 2, 4, 8, 16, 32 samples). The closest achievable rate to 20 Hz is **25 Hz** (100Hz ÷ 4 samples), which is 25% higher than requested. Alternative is 50 Hz (no averaging) or 12.5 Hz (100Hz ÷ 8 samples).
