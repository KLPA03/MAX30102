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

static uint8_t downsample_counter = 0;

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
void max30102_init(void) 
{
    max30102_write(0x09, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100)); 

    max30102_write(0x08, 0x0F);
    max30102_write(0x09, 0x03);
    max30102_write(0x0A, 0x27);
    max30102_write(0x0C, 0x1F);
    max30102_write(0x0D, 0x1F);

    ESP_LOGI(TAG, "MAX30102 initialized for heart rate with Red & IR at 100 Hz");
}

/* ---------- READ WITH DOWNSAMPLING TO 20 Hz ---------- */
void max30102_read_heart_rate_20hz(uint32_t *red_led, uint32_t *ir_led, bool *data_ready)
{
    uint32_t red_temp, ir_temp;
    uint8_t fifo_data[6];
    
    if (max30102_read(0x07, fifo_data, 6) == ESP_OK) {
        red_temp = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
        red_temp &= 0x3FFFF;
        
        ir_temp = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];
        ir_temp &= 0x3FFFF;
        
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

/* ---------- TASK ---------- */
void max30102_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    
    ESP_LOGI(TAG, "Initializing MAX30102...");
    max30102_init();
    
    uint32_t red_led, ir_led;
    bool data_ready;
    
    while (1) {
        max30102_read_heart_rate_20hz(&red_led, &ir_led, &data_ready);
        
        if (data_ready) {
            ESP_LOGI(TAG, "HR @ 20Hz - Red: %lu, IR: %lu", red_led, ir_led);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
