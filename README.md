# MAX30102

## Heart Rate Monitoring with Red & IR Reading at Exactly 20 Hz

### Initialization Code (100 Hz Hardware Sampling)

```c
/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    max30102_write(0x09, 0x40);   // Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x08, 0x0F);  // FIFO Config: No averaging, FIFO rollover enabled
    max30102_write(0x09, 0x03);  // SpO2 mode (enables both Red and IR LEDs for heart rate)
    max30102_write(0x0A, 0x27);  // 100 Hz sampling, 411μs pulse width, 2048 nA range
    max30102_write(0x0C, 0x1F);  // Red LED: 6.4 mA (for heart rate)
    max30102_write(0x0D, 0x1F);  // IR LED: 6.4 mA (for heart rate)

    ESP_LOGI(TAG, "MAX30102 initialized for heart rate with Red & IR at 100 Hz");
}
```

### Downsampling Code (100 Hz → 20 Hz)

```c
/* ---------- READ WITH DOWNSAMPLING TO 20 Hz ---------- */
static uint8_t downsample_counter = 0;

void max30102_read_heart_rate_20hz(uint32_t *red_led, uint32_t *ir_led, bool *data_ready)
{
    uint32_t red_temp, ir_temp;
    uint8_t fifo_data[6];  // 3 bytes Red + 3 bytes IR
    
    // Read FIFO data (Red and IR)
    if (max30102_read(0x07, fifo_data, 6) == ESP_OK) {
        // Parse Red LED (first 3 bytes)
        red_temp = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
        red_temp &= 0x3FFFF;  // 18-bit data
        
        // Parse IR LED (next 3 bytes)
        ir_temp = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];
        ir_temp &= 0x3FFFF;  // 18-bit data
        
        // Downsample: output every 5th sample (100 Hz / 5 = 20 Hz)
        downsample_counter++;
        if (downsample_counter >= 5) {
            downsample_counter = 0;
            *red_led = red_temp;
            *ir_led = ir_temp;
            *data_ready = true;
        } else {
            *data_ready = false;
        }
    } else {
        *data_ready = false;
    }
}

/* ---------- USAGE IN MAIN LOOP ---------- */
void max30102_task(void *pvParameters)
{
    max30102_init();
    
    uint32_t red_led, ir_led;
    bool data_ready;
    
    while (1) {
        max30102_read_heart_rate_20hz(&red_led, &ir_led, &data_ready);
        
        if (data_ready) {
            ESP_LOGI(TAG, "HR @ 20Hz - Red: %lu, IR: %lu", red_led, ir_led);
            // Process heart rate data here
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Read at 100 Hz (every 10ms)
    }
}
```

### Configuration Summary:
- **Hardware Sampling**: 100 Hz (Register 0x0A = 0x27)
- **Both Red and IR LEDs**: Enabled at 6.4 mA each
- **Downsampling Factor**: 5 (every 5th sample)
- **Output Rate**: 100 Hz ÷ 5 = **Exactly 20 Hz**
- **Mode**: SpO2 mode (0x03) for both LED readings
- **Use Case**: Heart rate monitoring with both wavelengths
