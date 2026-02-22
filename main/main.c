#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"

#include "wear_levelling.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

/* ================== ESP32-S3 CONNECTION TO MAX30102 ================== */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA         5
#define I2C_SCL         4
#define I2C_FREQ_HZ     400000
#define MAX30102_ADDR   0x57

/* MAX30102 registers */
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D

static const char *TAG = "MAX30102";

/* ---------- PREPARING ESP32-S3 TO USE I2C COMMUNICATION ---------- */
static void init_i2c(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

/* ---------- USB MSC (flash-backed FAT) ---------- */
#define BASE_PATH "/data"

static tinyusb_msc_storage_handle_t s_storage = NULL;

static void storage_mount_changed_cb(tinyusb_msc_storage_handle_t handle, tinyusb_msc_event_t *event, void *arg)
{
    switch (event->id) {
    case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
        ESP_LOGI(TAG, "Storage mount complete");
        break;
    case TINYUSB_MSC_EVENT_MOUNT_FAILED:
    case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
        ESP_LOGE(TAG, "Storage mount failed or format required");
        break;
    default:
        break;
    }
}

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    const esp_partition_t *data_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG, "Failed to find FATFS partition. Check partitions.csv.");
        return ESP_ERR_NOT_FOUND;
    }
    return wl_mount(data_partition, wl_handle);
}

static void init_usb_msc(void)
{
    static wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP,  // App owns storage unless USB host claims it
        .medium.wl_handle = wl_handle,
        .fat_fs = {
            .base_path = BASE_PATH,
        },
    };

    ESP_ERROR_CHECK(tinyusb_msc_new_storage_spiflash(&storage_cfg, &s_storage));
    ESP_ERROR_CHECK(tinyusb_msc_set_storage_callback(storage_mount_changed_cb, NULL));

    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
}

static bool storage_is_mounted_to_app(void)
{
    if (s_storage == NULL) return false;
    tinyusb_msc_mount_point_t mp = TINYUSB_MSC_STORAGE_MOUNT_USB;
    if (tinyusb_msc_get_storage_mount_point(s_storage, &mp) != ESP_OK) return false;
    return mp == TINYUSB_MSC_STORAGE_MOUNT_APP;
}

/* ---------- MAX30102 LOW LEVEL ---------- */
static esp_err_t max30102_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_PORT, MAX30102_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t max30102_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MAX30102_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t max30102_read_u8(uint8_t reg, uint8_t *val)
{
    return max30102_read(reg, val, 1);
}

/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void)
{
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x40));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(max30102_write(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_RD_PTR, 0x00));

    ESP_ERROR_CHECK(max30102_write(REG_FIFO_CONFIG, 0x4F));

    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x03));

    ESP_ERROR_CHECK(max30102_write(REG_SPO2_CONFIG, 0x27));

    ESP_ERROR_CHECK(max30102_write(REG_LED1_PA, 0x1F));
    ESP_ERROR_CHECK(max30102_write(REG_LED2_PA, 0x1F));

    (void)max30102_write(REG_INTR_ENABLE_1, 0x00);
    (void)max30102_write(REG_INTR_ENABLE_2, 0x00);

    ESP_LOGI(TAG, "MAX30102 initialized");
}

static uint8_t max30102_fifo_samples_available(void)
{
    uint8_t wr = 0, rd = 0;
    if (max30102_read_u8(REG_FIFO_WR_PTR, &wr) != ESP_OK) return 0;
    if (max30102_read_u8(REG_FIFO_RD_PTR, &rd) != ESP_OK) return 0;
    return (uint8_t)((wr - rd) & 0x1F);
}

static esp_err_t max30102_read_fifo_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6];
    esp_err_t err = max30102_read(REG_FIFO_DATA, data, sizeof(data));
    if (err != ESP_OK) return err;

    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *red &= 0x03FFFF;
    *ir  &= 0x03FFFF;
    return ESP_OK;
}

/* ================== APP MAIN ================== */
void app_main(void)
{
    ESP_LOGI("CHECK", "I AM RUNNING");

    init_usb_msc();
    init_i2c();
    max30102_init();

    const TickType_t period_ticks = pdMS_TO_TICKS(50);
    const TickType_t usb_poll_ticks = pdMS_TO_TICKS(200);

    FILE *f = NULL;
    bool header_done = false;

    ESP_LOGI(TAG, "Recording started (sensor 100 Hz, output 20 Hz).");
    ESP_LOGI(TAG, "When USB host mounts MSC, logging pauses until host ejects the drive.");

    while (1) {
        if (!storage_is_mounted_to_app()) {
            if (f) {
                fflush(f);
                fclose(f);
                f = NULL;
            }
            header_done = false;
            vTaskDelay(usb_poll_ticks);
            continue;
        }

        if (f == NULL) {
            f = fopen(BASE_PATH "/data.csv", "a+");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open %s/data.csv (is the drive formatted?)", BASE_PATH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            fseek(f, 0, SEEK_END);
            long sz = ftell(f);
            if (!header_done && sz == 0) {
                fprintf(f, "time_ms,IR,RED,BPM,AVG_BPM\n");
                fflush(f);
            }
            header_done = true;
        }

        uint8_t n = max30102_fifo_samples_available();
        uint32_t ir = 0, red = 0;

        if (n == 0) {
            vTaskDelay(period_ticks);
            continue;
        }

        while (n > 1) {
            (void)max30102_read_fifo_sample(&red, &ir);
            n--;
        }
        ESP_ERROR_CHECK(max30102_read_fifo_sample(&red, &ir));

        int64_t time_ms = esp_timer_get_time() / 1000;
        int bpm = 0;
        int avg_bpm = 0;

        fprintf(f, "%lld,%lu,%lu,%d,%d\n",
                (long long)time_ms, (unsigned long)ir, (unsigned long)red, bpm, avg_bpm);
        fflush(f);

        ESP_LOGI(TAG, "REC | IR:%lu RED:%lu", (unsigned long)ir, (unsigned long)red);

        vTaskDelay(period_ticks);
    }
}

