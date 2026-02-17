#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"

/* ================== ESP32-S3 CONNECTION TO MAX30102 ================== */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA         5
#define I2C_SCL         4
#define I2C_FREQ_HZ     400000

/* MAX30102 7-bit I2C address */
#define MAX30102_ADDR   0x57

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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .clk_flags = 0,
#endif
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

static esp_err_t i2c_probe(uint8_t addr_7bit)
{
    /* i2c_master_probe exists in newer IDF versions, but this works everywhere */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

/* ---------- SPIFFS MOUNTING IN FLASH MEMORY ---------- */
static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    size_t total = 0, used = 0;
    esp_err_t err = esp_spiffs_info(NULL, &total, &used);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted (total=%u, used=%u)", (unsigned)total, (unsigned)used);
    } else {
        ESP_LOGW(TAG, "SPIFFS mounted, but info failed: %s", esp_err_to_name(err));
    }
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
static esp_err_t max30102_init(void)
{
    /* Verify the device ACKs on the bus */
    esp_err_t err = i2c_probe(MAX30102_ADDR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 not found on I2C (addr=0x%02X): %s", MAX30102_ADDR, esp_err_to_name(err));
        return err;
    }

    uint8_t part_id = 0, rev_id = 0;
    (void)max30102_read_u8(0xFF, &part_id); /* PART_ID */
    (void)max30102_read_u8(0xFE, &rev_id);  /* REV_ID */
    ESP_LOGI(TAG, "MAX30102 probe OK (PART_ID=0x%02X REV_ID=0x%02X)", part_id, rev_id);

    /* Reset: MODE_CONFIG (0x09), bit6 = 1 */
    err = max30102_write(0x09, 0x40);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Clear FIFO pointers */
    ESP_ERROR_CHECK(max30102_write(0x04, 0x00)); /* FIFO_WR_PTR */
    ESP_ERROR_CHECK(max30102_write(0x05, 0x00)); /* OVF_COUNTER */
    ESP_ERROR_CHECK(max30102_write(0x06, 0x00)); /* FIFO_RD_PTR */

    /* FIFO_CONFIG (0x08):
     * - sample average = 4 (0b010 << 5)
     * - FIFO rollover disabled (bit4=0)
     * - FIFO almost full = 0x0F
     */
    ESP_ERROR_CHECK(max30102_write(0x08, (0x02 << 5) | 0x0F));

    /* SpO2 mode: MODE_CONFIG (0x09) = 0x03 */
    ESP_ERROR_CHECK(max30102_write(0x09, 0x03));

    /* SPO2_CONFIG (0x0A): ADC range=4096nA (0b01), sample rate=100Hz (0b001), pulse width=411us/18-bit (0b11) */
    ESP_ERROR_CHECK(max30102_write(0x0A, 0x27));

    /* LED pulse amplitudes */
    ESP_ERROR_CHECK(max30102_write(0x0C, 0x1F)); /* LED1_PA (RED) */
    ESP_ERROR_CHECK(max30102_write(0x0D, 0x1F)); /* LED2_PA (IR) */

    /* Clear any pending interrupts by reading INT_STATUS_1/2 */
    uint8_t is1 = 0, is2 = 0;
    (void)max30102_read_u8(0x00, &is1);
    (void)max30102_read_u8(0x01, &is2);

    ESP_LOGI(TAG, "MAX30102 initialized (INT_STATUS_1=0x%02X INT_STATUS_2=0x%02X)", is1, is2);
    return ESP_OK;
}

/* ---------- READ FIFO ---------- */
static esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6] = {0};
    esp_err_t err = max30102_read(0x07, data, sizeof(data)); /* FIFO_DATA */
    if (err != ESP_OK) return err;

    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *red &= 0x03FFFF;
    *ir  &= 0x03FFFF;
    return ESP_OK;
}

static bool file_is_empty(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        return true; /* doesn't exist yet */
    }
    return st.st_size == 0;
}

void app_main(void)
{
    ESP_LOGI("CHECK", "I AM RUNNING");

    init_spiffs();
    init_i2c();

    ESP_ERROR_CHECK(max30102_init());

    const char *csv_path = "/spiffs/data.csv";
    bool write_header = file_is_empty(csv_path);

    FILE *f = fopen(csv_path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open CSV file: %s", csv_path);
        return;
    }

    if (write_header) {
        fprintf(f, "time_ms,IR,RED,BPM,AVG_BPM\n");
        fflush(f);
    }

    ESP_LOGI(TAG, "Recording started -> %s", csv_path);

    while (1) {
        uint32_t ir = 0, red = 0;
        int bpm = 0;       /* placeholder */
        int avg_bpm = 0;   /* placeholder */

        esp_err_t err = max30102_read_fifo(&red, &ir);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FIFO read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        int64_t time_ms = esp_timer_get_time() / 1000;

        fprintf(f, "%lld,%lu,%lu,%d,%d\n",
                (long long)time_ms,
                (unsigned long)ir,
                (unsigned long)red,
                bpm,
                avg_bpm);
        fflush(f);

        ESP_LOGI(TAG, "REC | IR:%lu RED:%lu", (unsigned long)ir, (unsigned long)red);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

