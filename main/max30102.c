#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "max30102.h"

static const char *TAG = "MAX30102";

#define MAX30102_I2C_ADDR 0x57
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 400000

/* ---------- I2C Functions ---------- */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t max30102_write(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t max30102_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void) 
{
    // Reset the sensor
    max30102_write(0x09, 0x40);   // Mode Configuration: Reset
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // Configure for Heart Rate mode only (not SpO2)
    max30102_write(0x09, 0x02);  // Mode Configuration: Heart Rate mode (0x02)
    
    // SpO2 Configuration: 20 Hz sampling rate, 411 μs pulse width, no ADC range
    // Bits [7:5] = 000 (50 Hz - closest to 20 Hz available)
    // Bits [4:2] = 011 (411 μs pulse width)
    // Bits [1:0] = 11 (ADC Range 16384 nA, 16-bit resolution)
    // Note: MAX30102 doesn't support exactly 20 Hz, using 50 Hz as closest option
    max30102_write(0x0A, 0x0F);  // 50 Hz, 411μs pulse width, 16384 nA range
    
    // Set LED pulse amplitudes for heart rate detection
    max30102_write(0x0C, 0x1F);   // LED1 (Red LED) Pulse Amplitude = 6.4 mA
    max30102_write(0x0D, 0x00);   // LED2 (IR LED) Pulse Amplitude = 0 (not used in HR mode)

    ESP_LOGI(TAG, "MAX30102 initialized in Heart Rate mode at 50 Hz sampling rate");
}

/* ---------- MAX30102 Read Data ---------- */
esp_err_t max30102_read_fifo(uint32_t *red_led)
{
    uint8_t data[3];
    esp_err_t ret = max30102_read(0x07, data, 3); // Read FIFO Data Register
    
    if (ret == ESP_OK) {
        *red_led = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        *red_led &= 0x3FFFF; // 18-bit data
    }
    
    return ret;
}

void max30102_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    
    ESP_LOGI(TAG, "Initializing MAX30102...");
    max30102_init();
    
    uint32_t red_led;
    
    while (1) {
        if (max30102_read_fifo(&red_led) == ESP_OK) {
            ESP_LOGI(TAG, "Heart Rate Red LED: %lu", red_led);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Read at 20 Hz (50ms interval)
    }
}
