#include "max30101.h"

#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "max30101";

#define MAX30101_I2C_TIMEOUT_MS  50

// Register map (MAX30101)
#define MAX30101_REG_INT_STATUS_1     0x00
#define MAX30101_REG_INT_STATUS_2     0x01
#define MAX30101_REG_INT_ENABLE_1     0x02
#define MAX30101_REG_INT_ENABLE_2     0x03
#define MAX30101_REG_FIFO_WR_PTR      0x04
#define MAX30101_REG_OVF_COUNTER      0x05
#define MAX30101_REG_FIFO_RD_PTR      0x06
#define MAX30101_REG_FIFO_DATA        0x07
#define MAX30101_REG_FIFO_CONFIG      0x08
#define MAX30101_REG_MODE_CONFIG      0x09
#define MAX30101_REG_SPO2_CONFIG      0x0A
#define MAX30101_REG_LED1_PA          0x0C
#define MAX30101_REG_LED2_PA          0x0D
#define MAX30101_REG_REV_ID           0xFE
#define MAX30101_REG_PART_ID          0xFF

static esp_err_t max30101_read(max30101_t *s, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(s->i2c_port, s->i2c_addr, &reg, 1, data, len, pdMS_TO_TICKS(MAX30101_I2C_TIMEOUT_MS));
}

static esp_err_t max30101_write(max30101_t *s, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + 16];
    if (len > 16) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(s->i2c_port, s->i2c_addr, buf, 1 + len, pdMS_TO_TICKS(MAX30101_I2C_TIMEOUT_MS));
}

esp_err_t max30101_read_reg(max30101_t *s, uint8_t reg, uint8_t *val)
{
    return max30101_read(s, reg, val, 1);
}

esp_err_t max30101_write_reg(max30101_t *s, uint8_t reg, uint8_t val)
{
    return max30101_write(s, reg, &val, 1);
}

esp_err_t max30101_init(max30101_t *out, const max30101_i2c_config_t *cfg)
{
    if (!out || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    out->i2c_port = (i2c_port_t)cfg->i2c_port;
    out->i2c_addr = cfg->i2c_addr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda_io,
        .scl_io_num = cfg->scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq_hz,
    };

    esp_err_t err = i2c_param_config(out->i2c_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(out->i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    uint8_t part = 0, rev = 0;
    if (max30101_read_reg(out, MAX30101_REG_PART_ID, &part) == ESP_OK &&
        max30101_read_reg(out, MAX30101_REG_REV_ID, &rev) == ESP_OK) {
        ESP_LOGI(TAG, "PART_ID=0x%02X REV_ID=0x%02X", part, rev);
    } else {
        ESP_LOGW(TAG, "Unable to read PART_ID/REV_ID (check wiring/I2C addr)");
    }

    // Clear interrupt status registers (read-to-clear).
    (void)max30101_read_reg(out, MAX30101_REG_INT_STATUS_1, &part);
    (void)max30101_read_reg(out, MAX30101_REG_INT_STATUS_2, &rev);

    return ESP_OK;
}

esp_err_t max30101_configure_spo2(max30101_t *s)
{
    if (!s) {
        return ESP_ERR_INVALID_ARG;
    }

    // Soft reset (bit 6).
    esp_err_t err = max30101_write_reg(s, MAX30101_REG_MODE_CONFIG, 0x40);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "reset write failed (%s)", esp_err_to_name(err));
        return err;
    }
    for (int i = 0; i < 50; i++) {
        uint8_t mode = 0;
        err = max30101_read_reg(s, MAX30101_REG_MODE_CONFIG, &mode);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "reset read failed (%s)", esp_err_to_name(err));
            return err;
        }
        if ((mode & 0x40) == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Disable interrupts (we poll FIFO).
    err = max30101_write_reg(s, MAX30101_REG_INT_ENABLE_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "int en1 failed (%s)", esp_err_to_name(err));
        return err;
    }
    err = max30101_write_reg(s, MAX30101_REG_INT_ENABLE_2, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "int en2 failed (%s)", esp_err_to_name(err));
        return err;
    }

    // Clear FIFO pointers.
    err = max30101_write_reg(s, MAX30101_REG_FIFO_WR_PTR, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fifo wr ptr failed (%s)", esp_err_to_name(err));
        return err;
    }
    err = max30101_write_reg(s, MAX30101_REG_OVF_COUNTER, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fifo ovf failed (%s)", esp_err_to_name(err));
        return err;
    }
    err = max30101_write_reg(s, MAX30101_REG_FIFO_RD_PTR, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fifo rd ptr failed (%s)", esp_err_to_name(err));
        return err;
    }

    // FIFO config: average=1, rollover enabled, almost-full threshold=0x0F.
    err = max30101_write_reg(s, MAX30101_REG_FIFO_CONFIG, 0x1F);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fifo cfg failed (%s)", esp_err_to_name(err));
        return err;
    }

    // SpO2 config:
    // - ADC range = 4096 nA (01)
    // - Sample rate = 100 sps (011)
    // - Pulse width = 411 us (18-bit) (11)
    err = max30101_write_reg(s, MAX30101_REG_SPO2_CONFIG, 0x2F);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spo2 cfg failed (%s)", esp_err_to_name(err));
        return err;
    }

    // LED pulse amplitudes (tune for your optics).
    err = max30101_write_reg(s, MAX30101_REG_LED1_PA, 0x24);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led1 failed (%s)", esp_err_to_name(err));
        return err;
    }
    err = max30101_write_reg(s, MAX30101_REG_LED2_PA, 0x24);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led2 failed (%s)", esp_err_to_name(err));
        return err;
    }

    // SpO2 mode (Red + IR): MODE[2:0] = 0b011.
    err = max30101_write_reg(s, MAX30101_REG_MODE_CONFIG, 0x03);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mode set failed (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t max30101_read_fifo_red_ir(max30101_t *s, uint32_t *red, uint32_t *ir)
{
    if (!s || !red || !ir) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[6] = {0};
    esp_err_t err = max30101_read(s, MAX30101_REG_FIFO_DATA, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    uint32_t r = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    uint32_t i = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    *red = r & 0x3FFFF;
    *ir = i & 0x3FFFF;
    return ESP_OK;
}

esp_err_t max30101_deinit(max30101_t *s)
{
    if (!s) {
        return ESP_ERR_INVALID_ARG;
    }
    (void)i2c_driver_delete(s->i2c_port);
    return ESP_OK;
}

