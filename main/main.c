#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"

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
#define REG_LED1_PA         0x0C /* RED */
#define REG_LED2_PA         0x0D /* IR */

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
    ESP_LOGI(TAG, "SPIFFS mounted");
}

/* ---------- MAX30102 LOW LEVEL ---------- */
static esp_err_t max30102_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_PORT, MAX30102_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t max30102_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_PORT, MAX30102_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t max30102_read_u8(uint8_t reg, uint8_t *val)
{
    return max30102_read(reg, val, 1);
}

/* ---------- MAX30102 INIT ---------- */
static void max30102_init(void)
{
    /* Reset */
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x40));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Clear FIFO pointers */
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_RD_PTR, 0x00));

    /* FIFO: sample average = 4, rollover = enabled, almost full = 0 */
    ESP_ERROR_CHECK(max30102_write(REG_FIFO_CONFIG, 0x4F));

    /* SpO2 mode (RED+IR) */
    ESP_ERROR_CHECK(max30102_write(REG_MODE_CONFIG, 0x03));

    /*
     * SpO2 config:
     * - ADC range = 4096 nA (0b01 << 5)
     * - sample rate = 100 Hz (0b011 << 2)
     * - pulse width = 411 us (0b11)
     *
     * Note: MAX30102 doesn't support 20 Hz directly; we downsample in firmware to 20 Hz.
     */
    ESP_ERROR_CHECK(max30102_write(REG_SPO2_CONFIG, 0x27));

    /* LED currents */
    ESP_ERROR_CHECK(max30102_write(REG_LED1_PA, 0x1F)); /* RED */
    ESP_ERROR_CHECK(max30102_write(REG_LED2_PA, 0x1F)); /* IR */

    /* Disable interrupts (poll FIFO) */
    (void)max30102_write(REG_INTR_ENABLE_1, 0x00);
    (void)max30102_write(REG_INTR_ENABLE_2, 0x00);

    ESP_LOGI(TAG, "MAX30102 initialized");
}

static uint8_t max30102_fifo_samples_available(void)
{
    uint8_t wr = 0, rd = 0;
    if (max30102_read_u8(REG_FIFO_WR_PTR, &wr) != ESP_OK) return 0;
    if (max30102_read_u8(REG_FIFO_RD_PTR, &rd) != ESP_OK) return 0;
    return (uint8_t)((wr - rd) & 0x1F); /* 32-depth FIFO */
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

/* ---------- HEART RATE ESTIMATION (simple peak detection on IR AC component) ---------- */
typedef struct {
    float dc_ema;
    float abs_ema;
    float thresh;

    bool in_peak;
    float peak_max;
    int64_t peak_time_ms;
    int64_t last_beat_ms;

    int bpm;
    int bpm_hist[8];
    int bpm_hist_len;
    int bpm_hist_idx;
    int avg_bpm;
} hr_state_t;

static void hr_init(hr_state_t *s, uint32_t ir0)
{
    memset(s, 0, sizeof(*s));
    s->dc_ema = (float)ir0;
    s->abs_ema = 0.0f;
    s->thresh = 200.0f; /* small starting threshold; will adapt */
    s->last_beat_ms = -1;
}

static void hr_push_bpm(hr_state_t *s, int bpm)
{
    if (bpm <= 0) return;
    s->bpm_hist[s->bpm_hist_idx] = bpm;
    s->bpm_hist_idx = (s->bpm_hist_idx + 1) % (int)(sizeof(s->bpm_hist) / sizeof(s->bpm_hist[0]));
    if (s->bpm_hist_len < (int)(sizeof(s->bpm_hist) / sizeof(s->bpm_hist[0]))) {
        s->bpm_hist_len++;
    }

    int sum = 0;
    for (int i = 0; i < s->bpm_hist_len; i++) sum += s->bpm_hist[i];
    s->avg_bpm = (s->bpm_hist_len > 0) ? (sum / s->bpm_hist_len) : 0;
}

static void hr_update(hr_state_t *s, uint32_t ir, int64_t t_ms)
{
    /* For ~20 Hz updates: fairly quick DC tracking, moderate envelope tracking */
    const float alpha_dc = 0.05f;
    const float alpha_abs = 0.10f;
    const int64_t refractory_ms = 300; /* prevent double-beat detection */

    s->dc_ema += alpha_dc * ((float)ir - s->dc_ema);
    float ac = (float)ir - s->dc_ema;

    float abs_ac = fabsf(ac);
    s->abs_ema += alpha_abs * (abs_ac - s->abs_ema);

    /* Dynamic threshold with floor */
    float dyn = s->abs_ema * 1.5f;
    if (dyn < 200.0f) dyn = 200.0f;
    s->thresh = dyn;

    bool refractory_ok = (s->last_beat_ms < 0) || ((t_ms - s->last_beat_ms) > refractory_ms);

    if (!s->in_peak) {
        if (refractory_ok && ac > s->thresh) {
            s->in_peak = true;
            s->peak_max = ac;
            s->peak_time_ms = t_ms;
        }
    } else {
        if (ac > s->peak_max) {
            s->peak_max = ac;
            s->peak_time_ms = t_ms;
        }

        /* end peak when we drop well below threshold or cross baseline */
        if (ac < (s->thresh * 0.5f) || ac < 0.0f) {
            s->in_peak = false;

            if (s->last_beat_ms >= 0) {
                int64_t ibi_ms = s->peak_time_ms - s->last_beat_ms;
                if (ibi_ms >= 300 && ibi_ms <= 2000) {
                    s->bpm = (int)(60000 / ibi_ms);
                    hr_push_bpm(s, s->bpm);
                }
            }
            s->last_beat_ms = s->peak_time_ms;
        }
    }
}

/* ================== APP MAIN ================== */
void app_main(void)
{
    ESP_LOGI("CHECK", "I AM RUNNING");

    init_spiffs();
    init_i2c();
    max30102_init();

    FILE *f = fopen("/spiffs/data.csv", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open CSV file");
        return;
    }

    /* Always write header (simple); you can remove if you want header only once. */
    fprintf(f, "time_ms,IR,RED,BPM,AVG_BPM\n");
    fflush(f);

    ESP_LOGI(TAG, "Recording started (output 20 Hz)");

    /* Prime HR state */
    uint32_t ir0 = 0, red0 = 0;
    (void)max30102_read_fifo_sample(&red0, &ir0);
    hr_state_t hr;
    hr_init(&hr, ir0);

    const TickType_t period_ticks = pdMS_TO_TICKS(50); /* 20 Hz output */

    while (1) {
        /* Drain FIFO to keep newest sample (avoid overflow when sensor runs faster than 20 Hz) */
        uint8_t n = max30102_fifo_samples_available();
        uint32_t ir = 0, red = 0;

        if (n == 0) {
            /* no new samples yet */
            vTaskDelay(period_ticks);
            continue;
        }

        /* Discard all but the last sample */
        while (n > 1) {
            (void)max30102_read_fifo_sample(&red, &ir);
            n--;
        }
        ESP_ERROR_CHECK(max30102_read_fifo_sample(&red, &ir));

        int64_t time_ms = esp_timer_get_time() / 1000;
        hr_update(&hr, ir, time_ms);

        fprintf(f, "%lld,%lu,%lu,%d,%d\n",
                (long long)time_ms, (unsigned long)ir, (unsigned long)red, hr.bpm, hr.avg_bpm);
        fflush(f);

        ESP_LOGI(TAG, "REC | IR:%lu RED:%lu | BPM:%d AVG:%d", (unsigned long)ir, (unsigned long)red, hr.bpm, hr.avg_bpm);

        vTaskDelay(period_ticks);
    }
}

