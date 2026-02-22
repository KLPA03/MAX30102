#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "wear_levelling.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

/* ---------- MAX30102 I2C Settings ---------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA         5
#define I2C_SCL         4
#define I2C_FREQ_HZ     400000
#define MAX30102_ADDR   0x57

/* MAX30102 registers */
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D

#define BASE_PATH "/data"

static const char *TAG = "MAX30102";

/* ---------- I2C Init ---------- */
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

/* ---------- MAX30102 Low Level ---------- */
static esp_err_t max30102_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
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

/* ---------- MAX30102 Init ---------- */
static void max30102_init(void)
{
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x40)); // reset
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(max30102_write(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_RD_PTR, 0x00));

    // FIFO_A_FULL = 0x0F (almost full), sample avg = 16, FIFO rollover enabled
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_CONFIG, 0x4F));
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x03)); // SpO2 mode
    // SPO2_ADC_RGE = 4096nA, SPO2_SR = 100Hz, LED_PW = 411us (18-bit)
    ESP_ERROR_CHECK(max30102_write(REG_SPO2_CONFIG, 0x27));
    ESP_ERROR_CHECK(max30102_write(REG_LED1_PA, 0x1F));
    ESP_ERROR_CHECK(max30102_write(REG_LED2_PA, 0x1F));

    // Disable interrupts
    ESP_ERROR_CHECK(max30102_write(0x02, 0x00));
    ESP_ERROR_CHECK(max30102_write(0x03, 0x00));

    ESP_LOGI(TAG, "MAX30102 initialized");
}

static uint8_t max30102_fifo_samples_available(void)
{
    uint8_t wr = 0, rd = 0;
    if (max30102_read_u8(REG_FIFO_WR_PTR, &wr) != ESP_OK) {
        return 0;
    }
    if (max30102_read_u8(REG_FIFO_RD_PTR, &rd) != ESP_OK) {
        return 0;
    }
    return (wr - rd) & 0x1F;
}

static esp_err_t max30102_read_fifo_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6];
    esp_err_t err = max30102_read(REG_FIFO_DATA, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }
    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *red &= 0x03FFFF;
    *ir  &= 0x03FFFF;
    return ESP_OK;
}

/* ---------- USB MSC Init ---------- */
static tinyusb_msc_storage_handle_t s_storage = NULL;
static SemaphoreHandle_t s_file_lock;
static FILE *s_file;
static bool s_header_done;

static void close_csv_locked(void)
{
    if (!s_file) {
        return;
    }
    fflush(s_file);
    fclose(s_file);
    s_file = NULL;
    s_header_done = false;
}

static void storage_event_cb(tinyusb_msc_storage_handle_t handle, tinyusb_msc_event_t *event, void *arg)
{
    (void)handle;
    (void)arg;

    switch (event->id) {
    case TINYUSB_MSC_EVENT_MOUNT_START:
        // Ensure the application closes all files before the FS is unmounted
        if (s_file_lock && xSemaphoreTake(s_file_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
            close_csv_locked();
            xSemaphoreGive(s_file_lock);
        }
        break;
    case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
        ESP_LOGI(TAG, "Storage mount complete. App mounted: %s",
                 (event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP) ? "Yes" : "No");
        break;
    case TINYUSB_MSC_EVENT_MOUNT_FAILED:
        ESP_LOGE(TAG, "Storage mount failed");
        break;
    case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
        ESP_LOGW(TAG, "Storage needs formatting (do_not_format=true)");
        break;
    case TINYUSB_MSC_EVENT_FORMAT_FAILED:
        ESP_LOGE(TAG, "Storage format failed");
        break;
    default:
        break;
    }
}

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    const esp_partition_t *data_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (!data_partition) {
        ESP_LOGE(TAG, "Failed to find FATFS partition. Check `partitions.csv`.");
        return ESP_ERR_NOT_FOUND;
    }
    return wl_mount(data_partition, wl_handle);
}

static void init_usb_msc(void)
{
    static wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_driver_config_t msc_drv_cfg = {
        .user_flags = {
            .auto_mount_off = 0, // keep default auto mount/unmount on USB connect/disconnect
        },
        .callback = storage_event_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_msc_install_driver(&msc_drv_cfg));

    static char base_path[] = BASE_PATH;
    const tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP, // app logs by default; USB connection switches to USB automatically
        .medium.wl_handle = wl_handle,
        .fat_fs = {
            .base_path = base_path,
            .config = {
                .max_files = 5,
                .format_if_mount_failed = false, // esp_tinyusb formats internally unless do_not_format=true
                .allocation_unit_size = 0,
            },
            .do_not_format = false, // allow first-boot format of a blank FAT partition
            .format_flags = 0,      // use default (FM_ANY)
        },
    };

    ESP_ERROR_CHECK(tinyusb_msc_new_storage_spiflash(&storage_cfg, &s_storage));

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB MSC initialized");
}

static bool storage_is_mounted_app(void)
{
    if (!s_storage) {
        return false;
    }
    tinyusb_msc_mount_point_t mp;
    if (tinyusb_msc_get_storage_mount_point(s_storage, &mp) != ESP_OK) {
        return false;
    }
    return (mp == TINYUSB_MSC_STORAGE_MOUNT_APP);
}

/* ---------- APP MAIN ---------- */
void app_main(void)
{
    ESP_LOGI(TAG, "System started");

    s_file_lock = xSemaphoreCreateMutex();
    if (!s_file_lock) {
        ESP_LOGE(TAG, "Failed to create file mutex");
        return;
    }

    init_i2c();
    max30102_init();
    init_usb_msc();

    const TickType_t period = pdMS_TO_TICKS(50);

    while (1) {
        if (!storage_is_mounted_app()) {
            if (xSemaphoreTake(s_file_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
                close_csv_locked();
                xSemaphoreGive(s_file_lock);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (xSemaphoreTake(s_file_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (!s_file) {
                s_file = fopen(BASE_PATH "/data.csv", "a+");
                if (!s_file) {
                    xSemaphoreGive(s_file_lock);
                    ESP_LOGE(TAG, "Failed to open %s", BASE_PATH "/data.csv");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                fseek(s_file, 0, SEEK_END);
                long sz = ftell(s_file);
                if (!s_header_done && sz == 0) {
                    fprintf(s_file, "time_ms,IR,RED,BPM,AVG_BPM\n");
                    fflush(s_file);
                }
                s_header_done = true;
            }
            xSemaphoreGive(s_file_lock);
        }

        uint8_t n = max30102_fifo_samples_available();
        uint32_t ir = 0, red = 0;

        if (n > 0) {
            // Read only the latest sample in the FIFO
            while (n > 1) {
                (void)max30102_read_fifo_sample(&red, &ir);
                n--;
            }
            ESP_ERROR_CHECK(max30102_read_fifo_sample(&red, &ir));

            const int64_t t_ms = esp_timer_get_time() / 1000;
            const int bpm = 0;
            const int avg_bpm = 0;

            if (xSemaphoreTake(s_file_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
                if (s_file) {
                    fprintf(s_file, "%lld,%lu,%lu,%d,%d\n",
                            (long long)t_ms,
                            (unsigned long)ir,
                            (unsigned long)red,
                            bpm,
                            avg_bpm);
                    fflush(s_file);
                }
                xSemaphoreGive(s_file_lock);
            }

            ESP_LOGI(TAG, "REC | IR:%lu RED:%lu", (unsigned long)ir, (unsigned long)red);
        }

        vTaskDelay(period);
    }
}

