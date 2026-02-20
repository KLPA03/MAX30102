#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ================== ESP32-S3 CONNECTION TO MAX30102 ================== */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA         5
#define I2C_SCL         4
#define I2C_FREQ_HZ     400000
#define MAX30102_ADDR   0x57

/* MAX30102 registers */
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

/* ---------- I2C ---------- */
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

/* ---------- SPIFFS ---------- */
static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total=%u, used=%u", (unsigned)total, (unsigned)used);
    } else {
        ESP_LOGW(TAG, "SPIFFS info failed: %s", esp_err_to_name(ret));
    }
}

/* ---------- MAX30102 low level ---------- */
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

/* ---------- MAX30102 init ---------- */
static void max30102_init(void)
{
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x40)); // reset
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(max30102_write(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_RD_PTR, 0x00));

    ESP_ERROR_CHECK(max30102_write(REG_FIFO_CONFIG, 0x4F)); // sample avg=4, rollover enabled
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x03)); // SpO2 mode (RED+IR)
    ESP_ERROR_CHECK(max30102_write(REG_SPO2_CONFIG, 0x27)); // 100 Hz

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
    return (uint8_t)((wr - rd) & 0x1F); // 32-deep FIFO
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

/* ---------- CSV helper ---------- */
static FILE *open_csv_or_die(const char *path)
{
    FILE *f = fopen(path, "a+");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s: errno=%d (%s)", path, errno, strerror(errno));
        return NULL;
    }

    (void)setvbuf(f, NULL, _IOLBF, 0); // line buffered if supported by VFS

    struct stat st;
    bool needs_header = (stat(path, &st) != 0) || (st.st_size == 0);
    if (needs_header) {
        if (fprintf(f, "time_ms,IR,RED,BPM,AVG_BPM\n") < 0) {
            ESP_LOGE(TAG, "Failed to write header: errno=%d (%s)", errno, strerror(errno));
            fclose(f);
            return NULL;
        }
        fflush(f);
        fsync(fileno(f));
    }

    return f;
}

static void csv_try_reopen(FILE **pf, const char *path)
{
    if (pf == NULL) return;
    if (*pf != NULL) {
        fclose(*pf);
        *pf = NULL;
    }
    *pf = open_csv_or_die(path);
}

/* ================== APP MAIN ================== */
void app_main(void)
{
    ESP_LOGI(TAG, "Boot");

    init_spiffs();
    init_i2c();
    max30102_init();

    const char *csv_path = "/spiffs/data.csv";
    FILE *f = open_csv_or_die(csv_path);
    if (f == NULL) {
        ESP_LOGE(TAG, "CSV open failed; aborting");
        return;
    }

    ESP_LOGI(TAG, "Recording started (sensor 100 Hz, output 20 Hz)");

    const TickType_t period_ticks = pdMS_TO_TICKS(50); // 20 Hz output
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_sync_ms = esp_timer_get_time() / 1000;

    while (1) {
        uint8_t n = max30102_fifo_samples_available();
        if (n == 0) {
            vTaskDelayUntil(&last_wake, period_ticks);
            continue;
        }

        uint64_t sum_ir = 0, sum_red = 0;
        uint32_t ir = 0, red = 0;
        uint8_t count = 0;
        while (n-- > 0) {
            if (max30102_read_fifo_sample(&red, &ir) == ESP_OK) {
                sum_ir += ir;
                sum_red += red;
                count++;
            }
        }
        if (count == 0) {
            vTaskDelayUntil(&last_wake, period_ticks);
            continue;
        }

        uint32_t ir_out = (uint32_t)(sum_ir / count);
        uint32_t red_out = (uint32_t)(sum_red / count);

        int64_t time_ms = esp_timer_get_time() / 1000;
        int bpm = 0;
        int avg_bpm = 0;

        if (f == NULL) {
            csv_try_reopen(&f, csv_path);
            if (f == NULL) {
                vTaskDelayUntil(&last_wake, period_ticks);
                continue;
            }
        }

        bool ok = true;
        if (fprintf(f, "%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%d,%d\n", time_ms, ir_out, red_out, bpm, avg_bpm) < 0) {
            ESP_LOGE(TAG, "CSV write failed: errno=%d (%s)", errno, strerror(errno));
            ok = false;
        } else if (fflush(f) != 0) {
            ESP_LOGW(TAG, "fflush failed: errno=%d (%s)", errno, strerror(errno));
            ok = false;
        }

        if (!ok) {
            ESP_LOGW(TAG, "Re-opening CSV file");
            csv_try_reopen(&f, csv_path);
        }

        if ((time_ms - last_sync_ms) >= 1000) { // durability sync once per second
            if (f != NULL) {
                if (fsync(fileno(f)) != 0) {
                    ESP_LOGW(TAG, "fsync failed: errno=%d (%s)", errno, strerror(errno));
                }
            }
            last_sync_ms = time_ms;
        }

        ESP_LOGI(TAG, "REC | IR:%" PRIu32 " RED:%" PRIu32 " (avg of %u)", ir_out, red_out, (unsigned)count);
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

