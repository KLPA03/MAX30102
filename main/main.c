#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wear_levelling.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

#include "tusb.h"

#include "led_strip.h"

// ----------------------------- User config ---------------------------------

#define MAX30102_I2C_PORT      I2C_NUM_0
#define MAX30102_I2C_ADDR      0x57
#define I2C_FREQ_HZ            CONFIG_APP_I2C_FREQ_HZ
#define MAX30102_I2C_SDA_GPIO  ((gpio_num_t)CONFIG_APP_I2C_SDA_GPIO)
#define MAX30102_I2C_SCL_GPIO  ((gpio_num_t)CONFIG_APP_I2C_SCL_GPIO)

#define MSC_FAT_BASE_PATH      "/data"

#define LOG_FILENAME           "log.csv"
#define MSC_LOG_PATH           MSC_FAT_BASE_PATH "/" LOG_FILENAME

// MAX30102 sampling: configure sensor at 100 Hz, then downsample to 20 Hz by averaging 5 samples.
#define RAW_SAMPLE_RATE_HZ     100
#define DOWNSAMPLE_FACTOR      5

// MAX3010x ADC is 18-bit; values are 0..262143. With SPO2_CONFIG ADC range=4096nA (0x2F),
// we can convert raw counts to photodiode current:
// current_pA = raw * (4096 nA * 1000 pA/nA) / 262143
#define MAX3010X_ADC_COUNTS_MAX   262143U

#if CONFIG_APP_MAX3010X_ADC_RANGE_2048NA
#define MAX3010X_ADC_RANGE_NA     2048U
#define MAX3010X_SPO2_ADC_RANGE_BITS  0x00
#elif CONFIG_APP_MAX3010X_ADC_RANGE_4096NA
#define MAX3010X_ADC_RANGE_NA     4096U
#define MAX3010X_SPO2_ADC_RANGE_BITS  0x20
#elif CONFIG_APP_MAX3010X_ADC_RANGE_8192NA
#define MAX3010X_ADC_RANGE_NA     8192U
#define MAX3010X_SPO2_ADC_RANGE_BITS  0x40
#elif CONFIG_APP_MAX3010X_ADC_RANGE_16384NA
#define MAX3010X_ADC_RANGE_NA     16384U
#define MAX3010X_SPO2_ADC_RANGE_BITS  0x60
#else
#define MAX3010X_ADC_RANGE_NA     4096U
#define MAX3010X_SPO2_ADC_RANGE_BITS  0x20
#endif

// ----------------------------- MAX30102 registers ---------------------------

#define MAX30102_REG_INTR_STATUS_1   0x00
#define MAX30102_REG_INTR_STATUS_2   0x01
#define MAX30102_REG_INTR_ENABLE_1   0x02
#define MAX30102_REG_INTR_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR     0x04
#define MAX30102_REG_OVF_COUNTER     0x05
#define MAX30102_REG_FIFO_RD_PTR     0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_REG_FIFO_CONFIG     0x08
#define MAX30102_REG_MODE_CONFIG     0x09
#define MAX30102_REG_SPO2_CONFIG     0x0A
#define MAX30102_REG_LED1_PA         0x0C // RED
#define MAX30102_REG_LED2_PA         0x0D // IR
#define MAX30102_REG_PART_ID         0xFF

// MODE_CONFIG bits
#define MAX30102_MODE_RESET_BIT      (1U << 6)

// ----------------------------- Globals -------------------------------------

static const char *TAG = "max3010x_msc";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_max30102;

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t s_msc_storage = NULL;
static bool s_app_storage_ready = false;

static volatile tinyusb_msc_mount_point_t s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
static volatile bool s_msc_transition = false;

static SemaphoreHandle_t s_log_mutex;
static FILE *s_log_msc = NULL;
static int s_log_lines_since_reopen = 0;

#define RECORDING_STATE_SCHEMA_VERSION  2

// Closing the file periodically makes FAT metadata robust against sudden RESET.
// Trade-off: more directory updates, but much less chance of garbage tail bytes.
#define LOG_REOPEN_EVERY_N_LINES  20

typedef struct {
    uint64_t t_ms;
    uint64_t idx_20hz;
    uint32_t ir;
    uint32_t red;
} log_sample_t;

#define SAMPLE_QUEUE_LEN  256
static QueueHandle_t s_sample_queue = NULL;

// Recording mode toggled by RESET button:
// - recording ON  -> GREEN LED, MSC drive hidden (no log.csv on PC), data appended to /data/log.csv
// - recording OFF -> RED LED, MSC drive exposed to PC, no recording
static bool s_recording_enabled = true;

static TaskHandle_t s_sensor_task = NULL;

typedef enum {
    STATUS_LED_KIND_DISABLED = 0,
    STATUS_LED_KIND_WS2812,
    STATUS_LED_KIND_GPIO_DUAL,
} status_led_kind_t;

static status_led_kind_t s_status_led_kind = STATUS_LED_KIND_DISABLED;
static gpio_num_t s_status_led_ws2812_gpio = GPIO_NUM_NC;
static gpio_num_t s_status_led_red_gpio = GPIO_NUM_NC;
static gpio_num_t s_status_led_green_gpio = GPIO_NUM_NC;
static bool s_status_led_gpio_active_low = true;

static led_strip_handle_t s_ws2812_leds[2] = { NULL, NULL };
static int s_ws2812_led_count = 0;

// ----------------------------- I2C helpers ---------------------------------

#define I2C_XFER_TIMEOUT_MS  100
#define I2C_RETRY_COUNT      3

// Forward declarations (needed for recovery path)
static bool i2c_probe_max3010x(void);
static esp_err_t max30102_init_100hz(void);
static esp_err_t max30102_get_unread_samples(uint8_t *unread);

static void i2c_bus_unlock_gpio(gpio_num_t sda, gpio_num_t scl)
{
#if CONFIG_APP_I2C_RECOVERY_ENABLED
    // Best-effort "bus unstick" sequence:
    // - release SDA
    // - toggle SCL up to 9 pulses to advance any stuck slave
    // - generate a STOP
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << (int)sda) | (1ULL << (int)scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&cfg);

    (void)gpio_set_level(sda, 1);
    (void)gpio_set_level(scl, 1);
    esp_rom_delay_us(5);

    for (int i = 0; i < 9; i++) {
        (void)gpio_set_level(scl, 0);
        esp_rom_delay_us(5);
        (void)gpio_set_level(scl, 1);
        esp_rom_delay_us(5);
        if (gpio_get_level(sda) == 1) {
            break;
        }
    }

    // STOP: SDA low -> SCL high -> SDA high
    (void)gpio_set_level(sda, 0);
    esp_rom_delay_us(5);
    (void)gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    (void)gpio_set_level(sda, 1);
    esp_rom_delay_us(5);
#else
    (void)sda;
    (void)scl;
#endif
}

static esp_err_t max3010x_i2c_reinit(void)
{
#if CONFIG_APP_I2C_RECOVERY_ENABLED
    ESP_LOGW(TAG, "Re-initializing I2C bus + MAX3010x (recovery)");

    if (s_max30102) {
        (void)i2c_master_bus_rm_device(s_max30102);
        s_max30102 = NULL;
    }
    if (s_i2c_bus) {
        (void)i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }

    i2c_bus_unlock_gpio(MAX30102_I2C_SDA_GPIO, MAX30102_I2C_SCL_GPIO);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = MAX30102_I2C_PORT,
        .sda_io_num = MAX30102_I2C_SDA_GPIO,
        .scl_io_num = MAX30102_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = CONFIG_APP_I2C_ENABLE_INTERNAL_PULLUPS,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "i2c_new_master_bus failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 50000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_max30102), TAG, "add device failed");

    if (!i2c_probe_max3010x()) {
        ESP_LOGW(TAG, "I2C probe still failing after recovery");
        return ESP_FAIL;
    }

    esp_err_t err = max30102_init_100hz();
    // Give sensor/bus a moment to settle after reconfig; helps avoid immediate follow-up failures.
    vTaskDelay(pdMS_TO_TICKS(50));
    // Best-effort clear of FIFO pointers right after init.
    uint8_t tmp = 0;
    (void)max30102_get_unread_samples(&tmp);
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static bool i2c_probe_max3010x(void)
{
    // Probe with retries to avoid aborting on wiring/power-up timing issues
    for (int i = 0; i < 10; i++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, MAX30102_I2C_ADDR, I2C_XFER_TIMEOUT_MS);
        if (err == ESP_OK) {
            return true;
        }
        ESP_LOGW(TAG, "I2C probe 0x%02X failed (%s), retrying...", MAX30102_I2C_ADDR, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static esp_err_t i2c_reg_write_u8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        err = i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(I2C_XFER_TIMEOUT_MS));
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (s_i2c_bus) {
            (void)i2c_master_bus_reset(s_i2c_bus);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return err;
}

static esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        err = i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(I2C_XFER_TIMEOUT_MS));
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (s_i2c_bus) {
            (void)i2c_master_bus_reset(s_i2c_bus);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return err;
}

static esp_err_t max30102_read_u8(uint8_t reg, uint8_t *val)
{
    return i2c_reg_read(s_max30102, reg, val, 1);
}

// ----------------------------- MAX30102 driver ------------------------------

static esp_err_t max30102_soft_reset(void)
{
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET_BIT),
                        TAG, "reset write failed");

    // Wait until RESET bit clears
    for (int i = 0; i < 50; i++) {
        uint8_t mode = 0;
        ESP_RETURN_ON_ERROR(max30102_read_u8(MAX30102_REG_MODE_CONFIG, &mode), TAG, "reset read failed");
        if ((mode & MAX30102_MODE_RESET_BIT) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t max30102_init_100hz(void)
{
    uint8_t part_id = 0;
    esp_err_t err = max30102_read_u8(MAX30102_REG_PART_ID, &part_id);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MAX3010x PART_ID=0x%02X (0x11=MAX30101, 0x15=MAX30102)", part_id);
    } else {
        ESP_LOGW(TAG, "Unable to read MAX30102 PART_ID (%s)", esp_err_to_name(err));
    }

    ESP_RETURN_ON_ERROR(max30102_soft_reset(), TAG, "soft reset failed");

    // Disable interrupts (polling FIFO)
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_INTR_ENABLE_1, 0x00), TAG, "int disable1");
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_INTR_ENABLE_2, 0x00), TAG, "int disable2");

    // Clear FIFO pointers
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_FIFO_WR_PTR, 0x00), TAG, "fifo wr ptr");
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_OVF_COUNTER, 0x00), TAG, "fifo ovf");
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_FIFO_RD_PTR, 0x00), TAG, "fifo rd ptr");

    // FIFO config: sample avg = 1, rollover = enabled, almost full = 0x0F
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_FIFO_CONFIG, 0x1F), TAG, "fifo config");

    // SPO2 config:
    // - ADC range: configurable (menuconfig)
    // - Sample rate: 100Hz (011 << 2)
    // - Pulse width: 411us / 18-bit (11)
    const uint8_t spo2_cfg = (uint8_t)(MAX3010X_SPO2_ADC_RANGE_BITS | 0x0C | 0x03);
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_SPO2_CONFIG, spo2_cfg), TAG, "spo2 config");

    // LED pulse amplitudes (tune for your sensor / signal level)
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_LED1_PA, (uint8_t)CONFIG_APP_MAX3010X_LED_RED_PA), TAG, "led1");
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_LED2_PA, (uint8_t)CONFIG_APP_MAX3010X_LED_IR_PA), TAG, "led2");

    // Mode: SpO2 (RED + IR)
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_MODE_CONFIG, 0x03), TAG, "mode spo2");

    // Clear any pending interrupt status
    uint8_t tmp[2];
    (void)i2c_reg_read(s_max30102, MAX30102_REG_INTR_STATUS_1, tmp, 2);

    ESP_LOGI(TAG, "MAX30102 configured for %dHz raw sampling", RAW_SAMPLE_RATE_HZ);
    return ESP_OK;
}

static esp_err_t max30102_get_unread_samples(uint8_t *unread)
{
    // Read FIFO_WR_PTR, OVF_COUNTER, FIFO_RD_PTR in one transaction (0x04..0x06)
    uint8_t ptrs[3] = {0};
    ESP_RETURN_ON_ERROR(i2c_reg_read(s_max30102, MAX30102_REG_FIFO_WR_PTR, ptrs, sizeof(ptrs)), TAG, "fifo ptrs read failed");
    uint8_t wr = ptrs[0] & 0x1F;
    uint8_t rd = ptrs[2] & 0x1F;
    *unread = (uint8_t)((wr - rd) & 0x1F);
    return ESP_OK;
}

// ----------------------------- Storage / logging ---------------------------

static esp_err_t ensure_csv_header(FILE **fp, const char *path)
{
    if (*fp != NULL) {
        return ESP_OK;
    }

    // Check if file exists and non-empty
    FILE *fr = fopen(path, "r");
    if (fr) {
        int c = fgetc(fr);
        fclose(fr);
        if (c != EOF) {
            *fp = fopen(path, "a");
            return (*fp) ? ESP_OK : ESP_FAIL;
        }
    }

    // Create new file with header
    *fp = fopen(path, "w");
    if (!*fp) {
        return ESP_FAIL;
    }
    (void)setvbuf(*fp, NULL, _IONBF, 0);
    const char *hdr =
#if CONFIG_APP_LOG_UNITS_PICOAMPS
    #if CONFIG_APP_CSV_COMPACT
        "time_hms,IR_pA,RED_pA\n";
    #else
        "t_ms,time_hms,idx_20hz,IR_pA,RED_pA\n";
    #endif
#else
    #if CONFIG_APP_CSV_COMPACT
        "time_hms,IR,RED\n";
    #else
        "t_ms,time_hms,idx_20hz,IR,RED\n";
    #endif
#endif
    if (fwrite(hdr, 1, strlen(hdr), *fp) != strlen(hdr)) {
        fclose(*fp);
        *fp = NULL;
        return ESP_FAIL;
    }
    fflush(*fp);
    (void)fsync(fileno(*fp));
    return ESP_OK;
}

static void log_reopen_if_needed(void)
{
    if (!s_log_msc) {
        return;
    }
    if (s_log_lines_since_reopen < LOG_REOPEN_EVERY_N_LINES) {
        return;
    }
    // Close to ensure directory entry (file size) is committed.
    fflush(s_log_msc);
    (void)fsync(fileno(s_log_msc));
    fclose(s_log_msc);
    s_log_msc = NULL;
    s_log_lines_since_reopen = 0;
}

static void log_close_all(void)
{
    if (s_log_msc) {
        fflush(s_log_msc);
        (void)fsync(fileno(s_log_msc));
        fclose(s_log_msc);
        s_log_msc = NULL;
    }
    s_log_lines_since_reopen = 0;
}

static bool logging_allowed(void);
static void apply_recording_mode(void);

static void logger_task(void *arg)
{
    (void)arg;
    log_sample_t s;
    int lines_since_sync = 0;

    while (true) {
        if (!logging_allowed()) {
            // Drop queued samples while not allowed to log.
            if (s_sample_queue) {
                (void)xQueueReceive(s_sample_queue, &s, pdMS_TO_TICKS(50));
            } else {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            continue;
        }

        if (!s_sample_queue) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (xQueueReceive(s_sample_queue, &s, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        if (!s_log_mutex || xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }

        // MSC/FAT log (only when storage is mounted to APP)
        if (ensure_csv_header(&s_log_msc, MSC_LOG_PATH) != ESP_OK) {
            ESP_LOGW(TAG, "MSC log open failed (%s)", MSC_LOG_PATH);
        }

        if (s_log_msc) {
            uint32_t total_s = (uint32_t)(s.t_ms / 1000ULL);
            uint32_t ms_part = (uint32_t)(s.t_ms % 1000ULL);
            uint32_t s_part = total_s % 60U;
            uint32_t m_part = (total_s / 60U) % 60U;
            uint32_t h_part = (total_s / 3600U);

            // Hours can grow beyond 2 digits; keep buffer generous to avoid -Wformat-truncation.
            char t_hms[32];
            // Include milliseconds so 20Hz samples (50ms steps) never repeat timestamps.
            (void)snprintf(t_hms, sizeof(t_hms), "%02"PRIu32":%02"PRIu32":%02"PRIu32".%03"PRIu32,
                           h_part, m_part, s_part, ms_part);

            char line[128];
            int len;
#if CONFIG_APP_CSV_COMPACT
            len = snprintf(line, sizeof(line), "%s,%"PRIu32",%"PRIu32"\n", t_hms, s.ir, s.red);
#else
            len = snprintf(line, sizeof(line), "%"PRIu64",%s,%"PRIu64",%"PRIu32",%"PRIu32"\n",
                           s.t_ms, t_hms, s.idx_20hz, s.ir, s.red);
#endif
            if (len > 0 && len < (int)sizeof(line)) {
                size_t w = fwrite(line, 1, (size_t)len, s_log_msc);
                if (w != (size_t)len) {
                    ESP_LOGW(TAG, "MSC log write failed (errno=%d)", errno);
                } else {
                    s_log_lines_since_reopen++;
                    lines_since_sync++;
                    log_reopen_if_needed();
                }
            } else {
                ESP_LOGW(TAG, "MSC log line format overflow");
            }

            // Periodic sync (fsync is slow; doing it every line reduces effective sample rate).
            if (lines_since_sync >= 20) {
                fflush(s_log_msc);
                (void)fsync(fileno(s_log_msc));
                lines_since_sync = 0;
            }
        }

        xSemaphoreGive(s_log_mutex);
    }
}

static bool logging_allowed(void)
{
    if (!s_recording_enabled) {
        return false;
    }

    if (s_app_storage_ready) {
        return true;
    }

    return (!s_msc_transition) &&
           (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP);
}

static void recording_persist_state(bool enabled)
{
    nvs_handle_t h;
    if (nvs_open("app", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_set_u8(h, "rec", enabled ? 1 : 0);
    (void)nvs_commit(h);
    nvs_close(h);
}

static void recording_set_and_restart(bool enabled, const char *reason)
{
    ESP_LOGI(TAG, "%s: switching recording %s and restarting",
             reason, enabled ? "ON" : "OFF");

    s_recording_enabled = enabled;
    recording_persist_state(enabled);

    if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        log_close_all();
        xSemaphoreGive(s_log_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

#if CONFIG_APP_RECORDING_TOGGLE_WITH_BOOT_BUTTON && (CONFIG_APP_BOOT_BUTTON_GPIO != 0)
static void boot_button_task(void *arg)
{
    (void)arg;
    const gpio_num_t btn = (gpio_num_t)CONFIG_APP_BOOT_BUTTON_GPIO;
    const int min_press_ms = CONFIG_APP_BOOT_BUTTON_HOLD_MS;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << (int)btn),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&cfg);
    ESP_LOGI(TAG, "BOOT button runtime toggle enabled on GPIO%d (press/release >= %dms)",
             (int)btn, min_press_ms);

    bool was_pressed = false;
    int64_t pressed_us = 0;

    while (true) {
        bool pressed = (gpio_get_level(btn) == 0);
        int64_t now_us = esp_timer_get_time();

        if (pressed && !was_pressed) {
            pressed_us = now_us;
        }

        if (!pressed && was_pressed) {
            int64_t dur_ms = (now_us - pressed_us) / 1000;
            if (dur_ms >= min_press_ms) {
                ESP_LOGI(TAG, "BOOT button press detected (%lld ms)", dur_ms);
                recording_set_and_restart(!s_recording_enabled, "BOOT button");
            }
        }

        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

static void cdc_handle_command(const char *cmd)
{
    if (strcmp(cmd, "status") == 0) {
        ESP_LOGI(TAG, "Status: recording=%s storage=%s transition=%s",
                 s_recording_enabled ? "ON" : "OFF",
                 (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP) ? "APP(hidden)" : "USB(exposed)",
                 s_msc_transition ? "yes" : "no");
        return;
    }
    if (strcmp(cmd, "msd") == 0 || strcmp(cmd, "off") == 0) {
        recording_set_and_restart(false, "USB CDC command");
        return;
    }
    if (strcmp(cmd, "rec") == 0 || strcmp(cmd, "on") == 0) {
        recording_set_and_restart(true, "USB CDC command");
        return;
    }
    if (strcmp(cmd, "toggle") == 0) {
        recording_set_and_restart(!s_recording_enabled, "USB CDC command");
        return;
    }

    ESP_LOGI(TAG, "Unknown USB CDC command '%s' (use: status, msd, rec, toggle)", cmd);
}

static void cdc_command_task(void *arg)
{
    (void)arg;
#if CONFIG_APP_USB_CDC_CONSOLE
    char cmd[32];
    size_t len = 0;

    ESP_LOGI(TAG, "USB CDC commands: status | msd | rec | toggle");

    while (true) {
        if (!tud_ready()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        while (tud_cdc_available()) {
            int ch = tud_cdc_read_char();
            if (ch < 0) {
                break;
            }

            if (ch == '\r' || ch == '\n') {
                if (len > 0) {
                    cmd[len] = '\0';
                    cdc_handle_command(cmd);
                    len = 0;
                }
                continue;
            }

            if (len < (sizeof(cmd) - 1)) {
                cmd[len++] = (char)tolower((unsigned char)ch);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
#else
    vTaskDelete(NULL);
#endif
}

static void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
#if CONFIG_APP_STATUS_LED_PRESET_DEVKITC1 || CONFIG_APP_STATUS_LED_PRESET_CUSTOM
    if (s_status_led_kind != STATUS_LED_KIND_WS2812) {
        return;
    }
    if (s_ws2812_led_count <= 0) {
        return;
    }
    // led_strip uses RGB order in API regardless of pixel format.
    for (int i = 0; i < s_ws2812_led_count; i++) {
        if (s_ws2812_leds[i]) {
            (void)led_strip_set_pixel(s_ws2812_leds[i], 0, r, g, b);
            (void)led_strip_refresh(s_ws2812_leds[i]);
        }
    }
#else
    (void)r;
    (void)g;
    (void)b;
#endif
}

static void status_led_set_recording(bool recording_enabled)
{
    if (s_status_led_kind == STATUS_LED_KIND_WS2812) {
        if (recording_enabled) {
            // ON = green
            status_led_set_rgb(0, 64, 0);
        } else {
            // OFF = red
            status_led_set_rgb(64, 0, 0);
        }
        return;
    }

    if (s_status_led_kind == STATUS_LED_KIND_GPIO_DUAL) {
        bool red_on = !recording_enabled;
        bool green_on = recording_enabled;

        if (s_status_led_gpio_active_low) {
            red_on = !red_on;
            green_on = !green_on;
        }
        (void)gpio_set_level(s_status_led_red_gpio, red_on ? 1 : 0);
        (void)gpio_set_level(s_status_led_green_gpio, green_on ? 1 : 0);
        return;
    }

    (void)recording_enabled;
}

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER";
    }
}

static void recording_toggle_on_boot(void)
{
    // Persist state in NVS and (optionally) only toggle on external reset button.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open("app", NVS_READWRITE, &h));

    uint8_t v = 0;
    uint8_t schema_ver = 0;
    err = nvs_get_u8(h, "rec_ver", &schema_ver);
    if (err == ESP_ERR_NVS_NOT_FOUND || schema_ver != RECORDING_STATE_SCHEMA_VERSION) {
        // Always come up recording ON after flashing/migration so the USB drive is not exposed
        // immediately by a stale sdkconfig or old saved state.
        v = 1;
        ESP_ERROR_CHECK(nvs_set_u8(h, "rec", v));
        ESP_ERROR_CHECK(nvs_set_u8(h, "rec_ver", RECORDING_STATE_SCHEMA_VERSION));
        ESP_ERROR_CHECK(nvs_commit(h));
        ESP_LOGI(TAG, "Recording state initialized to %s", v ? "ON" : "OFF");
    } else {
        ESP_ERROR_CHECK(err);
        err = nvs_get_u8(h, "rec", &v);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
#if CONFIG_APP_RECORDING_DEFAULT_ON
            v = 1;
#else
            v = 0;
#endif
            ESP_ERROR_CHECK(nvs_set_u8(h, "rec", v));
            ESP_ERROR_CHECK(nvs_commit(h));
        } else {
            ESP_ERROR_CHECK(err);
        }
    }

    esp_reset_reason_t reason = esp_reset_reason();
    bool do_toggle = false;
#if CONFIG_APP_RECORDING_TOGGLE_ON_RESET
    do_toggle = true;
    #if CONFIG_APP_RECORDING_TOGGLE_ONLY_ON_EXT_RESET
    do_toggle = (reason == ESP_RST_EXT);
    #endif
#endif
    if (do_toggle) {
        v = (uint8_t)(!v);
        ESP_ERROR_CHECK(nvs_set_u8(h, "rec", v));
        ESP_ERROR_CHECK(nvs_commit(h));
    }
    nvs_close(h);

    s_recording_enabled = (v != 0);
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason_str(reason));
    ESP_LOGI(TAG, "Recording mode: %s%s",
             s_recording_enabled ? "ON (record-only, drive hidden)" : "OFF (drive exposed)",
#if CONFIG_APP_RECORDING_TOGGLE_ON_RESET && CONFIG_APP_RECORDING_TOGGLE_ONLY_ON_EXT_RESET
             " (toggles only on RESET button)"
#elif CONFIG_APP_RECORDING_TOGGLE_ON_RESET
             " (toggles on every boot)"
#else
             " (preserved across reset; use CDC commands to change)"
#endif
    );
}

static void storage_mount_changed_cb(tinyusb_msc_storage_handle_t handle, tinyusb_msc_event_t *event, void *arg)
{
    (void)handle;
    (void)arg;

    switch (event->id) {
    case TINYUSB_MSC_EVENT_MOUNT_START:
        s_msc_transition = true;
        if (s_log_mutex) {
            if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                log_close_all();
                xSemaphoreGive(s_log_mutex);
            }
        }
        break;
    case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
        s_msc_mount_point = event->mount_point;
        s_msc_transition = false;
        ESP_LOGI(TAG, "MSC disk state: %s",
                 (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP) ? "HIDDEN (APP owns disk)" : "EXPOSED (USB host owns disk)");
        break;
    case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
        ESP_LOGW(TAG, "MSC storage needs formatting (will be auto-formatted on mount if allowed)");
        break;
    case TINYUSB_MSC_EVENT_MOUNT_FAILED:
        ESP_LOGE(TAG, "MSC storage mount/unmount failed");
        s_msc_transition = false;
        break;
    default:
        break;
    }
}

static esp_err_t msc_storage_init_spiflash(void)
{
    const esp_partition_t *fat_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    if (!fat_part) {
        // Fallback: first FAT partition
        fat_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    }
    ESP_RETURN_ON_FALSE(fat_part != NULL, ESP_ERR_NOT_FOUND, TAG, "FAT partition not found (label 'storage')");

    ESP_LOGI(TAG, "Mounting wear levelling on FAT partition '%s' at offset 0x%"PRIx32" size 0x%"PRIx32,
             fat_part->label, fat_part->address, fat_part->size);
    ESP_RETURN_ON_ERROR(wl_mount(fat_part, &s_wl_handle), TAG, "wl_mount failed");

    // Disable TinyUSB auto-mount switching. We fully control whether the disk is exposed to the PC.
    // This is required so "recording ON" never pauses due to Windows auto-mounting the drive.
    const tinyusb_msc_driver_config_t drv_cfg = {
        .user_flags = {
            .auto_mount_off = 1,
        },
        .callback = storage_mount_changed_cb,
        .callback_arg = NULL,
    };
    esp_err_t drv_err = tinyusb_msc_install_driver(&drv_cfg);
    if (drv_err != ESP_OK && drv_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(drv_err, TAG, "tinyusb_msc_install_driver failed");
    }

    // Mount point is controlled by our "recording mode" logic after init.
    tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP,
        .fat_fs = {
            .base_path = MSC_FAT_BASE_PATH,
            .config = {
                .max_files = 5,
                .format_if_mount_failed = true,
                .allocation_unit_size = 0,
            },
            .do_not_format = false,
            .format_flags = 0,
        },
    };
    storage_cfg.medium.wl_handle = s_wl_handle;

    ESP_RETURN_ON_ERROR(tinyusb_msc_new_storage_spiflash(&storage_cfg, &s_msc_storage), TAG, "new_storage_spiflash failed");

    return ESP_OK;
}

static esp_err_t app_storage_init_spiflash(void)
{
    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 0,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    ESP_LOGI(TAG, "Mounting app FAT filesystem at %s", MSC_FAT_BASE_PATH);
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MSC_FAT_BASE_PATH, "storage", &mount_cfg, &s_wl_handle);
    if (err == ESP_OK) {
        s_app_storage_ready = true;
        s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;
    }
    return err;
}

static void apply_recording_mode(void)
{
    // Ensure files are closed before switching ownership.
    if (s_log_mutex) {
        if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            log_close_all();
            xSemaphoreGive(s_log_mutex);
        }
    }

    if (s_recording_enabled && s_app_storage_ready) {
        s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;
        if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            (void)ensure_csv_header(&s_log_msc, MSC_LOG_PATH);
            log_close_all();
            xSemaphoreGive(s_log_mutex);
        }
        status_led_set_recording(true);
        return;
    }

    if (!s_msc_storage) {
        status_led_set_recording(s_recording_enabled);
        return;
    }

    if (s_recording_enabled) {
        // Hide disk from the host and mount to APP for logging
        ESP_ERROR_CHECK(tinyusb_msc_set_storage_mount_point(s_msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP));
        s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;

        // Ensure the CSV exists early so it will be present when later exposed to the PC.
        if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            (void)ensure_csv_header(&s_log_msc, MSC_LOG_PATH);
            log_close_all();
            xSemaphoreGive(s_log_mutex);
        }
    } else {
        // Before exposing, best-effort ensure file exists and is closed cleanly.
        // (Even if our cached mount point is stale, ensure_csv_header() will just fail gracefully.)
        if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            (void)ensure_csv_header(&s_log_msc, MSC_LOG_PATH);
            log_close_all();
            xSemaphoreGive(s_log_mutex);
        }
        // Expose disk to host and stop logging
        ESP_ERROR_CHECK(tinyusb_msc_set_storage_mount_point(s_msc_storage, TINYUSB_MSC_STORAGE_MOUNT_USB));
        s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
    }

    status_led_set_recording(s_recording_enabled);
}

// ----------------------------- Sensor / logger task -------------------------

static void sensor_task(void *arg)
{
    (void)arg;
    uint64_t acc_ir = 0;
    uint64_t acc_red = 0;
    int acc_n = 0;
    int consecutive_i2c_errors = 0;
    uint32_t last_recover_ms = 0;
    uint32_t invalid_state_backoff_until_ms = 0;
    uint32_t invalid_state_count_window_start_ms = 0;
    int invalid_state_count_in_window = 0;
    bool timebase_set = false;
    uint64_t ds_idx = 0;
    const uint32_t ds_period_ms = (uint32_t)((1000 / RAW_SAMPLE_RATE_HZ) * DOWNSAMPLE_FACTOR); // 50ms at 100->20Hz

    const int max_samples_per_bulk = 10;
    uint8_t fifo_bulk[6 * 10];

    while (true) {
        if (!s_recording_enabled) {
            // Stop sampling activity when recording is OFF (drive exposed).
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (invalid_state_backoff_until_ms && (now_ms < invalid_state_backoff_until_ms)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint8_t unread = 0;
        esp_err_t err = max30102_get_unread_samples(&unread);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FIFO ptr read failed (%s)", esp_err_to_name(err));
            consecutive_i2c_errors++;
            if (err == ESP_ERR_INVALID_STATE) {
                // Driver reports bus not idle / invalid state (often SDA/SCL stuck low).
                // Do not hammer recovery continuously; use a short window + backoff.
                if ((now_ms - invalid_state_count_window_start_ms) > 3000) {
                    invalid_state_count_window_start_ms = now_ms;
                    invalid_state_count_in_window = 0;
                }
                invalid_state_count_in_window++;

                if ((now_ms - last_recover_ms) > 500) {
                    (void)max3010x_i2c_reinit();
                    last_recover_ms = now_ms;
                }

                // If it keeps happening, pause polling for a bit to let the bus recover electrically.
                if (invalid_state_count_in_window >= 5) {
                    invalid_state_backoff_until_ms = now_ms + 2000;
                    invalid_state_count_in_window = 0;
                }

                consecutive_i2c_errors = 0;
                continue;
            }
            if (consecutive_i2c_errors >= 5) {
                if ((now_ms - last_recover_ms) > 1000) {
                    (void)max3010x_i2c_reinit();
                    last_recover_ms = now_ms;
                }
                consecutive_i2c_errors = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        consecutive_i2c_errors = 0;

        if (unread == 0) {
            // Let FIFO accumulate to reduce I2C traffic and improve robustness.
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Read FIFO in small bulks to reduce I2C start/stop overhead.
        uint8_t remaining = unread;
        while (remaining > 0) {
            uint8_t n = remaining;
            if (n > (uint8_t)max_samples_per_bulk) {
                n = (uint8_t)max_samples_per_bulk;
            }
            const size_t bytes = (size_t)n * 6;
            err = i2c_reg_read(s_max30102, MAX30102_REG_FIFO_DATA, fifo_bulk, bytes);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "FIFO bulk read failed (%s)", esp_err_to_name(err));
                consecutive_i2c_errors++;
                if (err == ESP_ERR_INVALID_STATE) {
                    // Bus stuck / not idle. Recover immediately instead of spamming retries.
                    if ((now_ms - invalid_state_count_window_start_ms) > 3000) {
                        invalid_state_count_window_start_ms = now_ms;
                        invalid_state_count_in_window = 0;
                    }
                    invalid_state_count_in_window++;

                    if ((now_ms - last_recover_ms) > 500) {
                        (void)max3010x_i2c_reinit();
                        last_recover_ms = now_ms;
                    }
                    if (invalid_state_count_in_window >= 5) {
                        invalid_state_backoff_until_ms = now_ms + 2000;
                        invalid_state_count_in_window = 0;
                    }

                    consecutive_i2c_errors = 0;
                }
                if (consecutive_i2c_errors >= 5) {
                    if ((now_ms - last_recover_ms) > 1000) {
                        (void)max3010x_i2c_reinit();
                        last_recover_ms = now_ms;
                    }
                    consecutive_i2c_errors = 0;
                }
                break;
            }
            consecutive_i2c_errors = 0;

            for (uint8_t i = 0; i < n; i++) {
                const uint8_t *d = &fifo_bulk[i * 6];
                uint32_t raw_red = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
                uint32_t raw_ir  = ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];
                raw_red &= 0x3FFFF;
                raw_ir &= 0x3FFFF;

                if (!timebase_set) {
                    // Anchor sample clock to first sample read; then advance by the configured sample rate.
                    ds_idx = 0;
                    timebase_set = true;
                }

                acc_ir += raw_ir;
                acc_red += raw_red;
                acc_n++;

                if (acc_n >= DOWNSAMPLE_FACTOR) {
                    uint32_t ir_ds = (uint32_t)(acc_ir / DOWNSAMPLE_FACTOR);
                    uint32_t red_ds = (uint32_t)(acc_red / DOWNSAMPLE_FACTOR);
                    // Use sample-clock derived timestamp so downsampled rows are exactly 20Hz (50ms steps).
                    uint64_t sample_t_ms = (ds_idx * (uint64_t)ds_period_ms);

                    uint32_t ir_out = ir_ds;
                    uint32_t red_out = red_ds;
#if CONFIG_APP_LOG_UNITS_PICOAMPS
                    ir_out = (uint32_t)(((uint64_t)ir_ds * (uint64_t)MAX3010X_ADC_RANGE_NA * 1000ULL) / (uint64_t)MAX3010X_ADC_COUNTS_MAX);
                    red_out = (uint32_t)(((uint64_t)red_ds * (uint64_t)MAX3010X_ADC_RANGE_NA * 1000ULL) / (uint64_t)MAX3010X_ADC_COUNTS_MAX);
                    ESP_LOGI(TAG, "t_ms=%"PRIu64" IR_pA=%"PRIu32" RED_pA=%"PRIu32"%s",
                             sample_t_ms, ir_out, red_out, logging_allowed() ? "" : " (paused)");
#else
                    ESP_LOGI(TAG, "t_ms=%"PRIu64" IR=%"PRIu32" RED=%"PRIu32"%s",
                             sample_t_ms, ir_out, red_out, logging_allowed() ? "" : " (paused)");
#endif

                    if (s_sample_queue && logging_allowed()) {
                        log_sample_t out = {
                            .t_ms = sample_t_ms,
                            .idx_20hz = ds_idx,
                            .ir = ir_out,
                            .red = red_out,
                        };
                        if (xQueueSend(s_sample_queue, &out, 0) != pdTRUE) {
                            // Queue full: drop oldest and retry once so logging doesn't "stop".
                            log_sample_t dropped;
                            (void)xQueueReceive(s_sample_queue, &dropped, 0);
                            (void)xQueueSend(s_sample_queue, &out, 0);
                        }
                    }

                    ds_idx++;
                    acc_ir = 0;
                    acc_red = 0;
                    acc_n = 0;
                }
            }

            remaining = (uint8_t)(remaining - n);
        }

        // Let FIFO accumulate a bit; avoids hammering I2C.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ----------------------------- TinyUSB descriptors --------------------------

static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,   // Espressif VID (change for production)
    .idProduct = 0x4002,  // PID used in ESP-IDF examples
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

static char const *s_string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: English
    "Espressif",                    // 1: Manufacturer
    "ESP32-S3 MAX3010x Logger",      // 2: Product
    "0001",                         // 3: Serial
};

// USB configuration descriptors
#define EPNUM_MSC_OUT       0x01
#define EPNUM_MSC_IN        0x81
#define EPNUM_CDC_NOTIF     0x82
#define EPNUM_CDC_OUT       0x03
#define EPNUM_CDC_IN        0x83

#define TUSB_DESC_CDC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define TUSB_DESC_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_CDC = 0,     // CDC control
    ITF_NUM_CDC_DATA,    // CDC data
    ITF_NUM_MSC,         // MSC
    ITF_NUM_TOTAL
};

static uint8_t const s_composite_fs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CDC: interface number, string index, EP notification address, notification EP size, EP out, EP in, EP size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // MSC: interface number, string index, EP out, EP in, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

static uint8_t const s_cdc_only_fs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_CDC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CDC: interface number, string index, EP notification address, notification EP size, EP out, EP in, EP size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// ----------------------------- app_main -------------------------------------

static int usb_cdc_vprintf(const char *fmt, va_list ap)
{
    // Format first (doesn't consume 'ap' since we use a copy)
    char buf[256];
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap_copy);
    va_end(ap_copy);

    if (!CONFIG_APP_USB_CDC_CONSOLE) {
        return vprintf(fmt, ap);
    }

    // Always mirror logs to ROM printf (USB-Serial/JTAG/UART) so you can still monitor even
    // when the CDC COM port disappears/re-enumerates on Windows.
    if (len > 0) {
        size_t n_rom = (size_t)len;
        if (n_rom >= sizeof(buf)) {
            n_rom = sizeof(buf) - 1;
        }
        buf[n_rom] = '\0';
        esp_rom_printf("%s", buf);
    }

    // If USB isn't ready yet, fall back to default stdout (usually UART).
    // Note: Some hosts/tools may not assert "connected" line state immediately; writing is still safe.
    if (!tud_ready()) {
        return len;
    }

    if (len <= 0) {
        return len;
    }

    size_t n = (size_t)len;
    if (n > sizeof(buf)) {
        n = sizeof(buf);
    }

    // Non-blocking write: TinyUSB buffers internally.
    (void)tud_cdc_write(buf, n);
    (void)tud_cdc_write_flush();
    return len;
}

static esp_err_t ws2812_try_init_on_gpio(gpio_num_t gpio)
{
    if (s_ws2812_led_count >= (int)(sizeof(s_ws2812_leds) / sizeof(s_ws2812_leds[0]))) {
        return ESP_ERR_NO_MEM;
    }

    led_strip_handle_t h = NULL;
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WS2812 init failed (%s) on GPIO%d", esp_err_to_name(err), (int)gpio);
        return err;
    }

    s_ws2812_leds[s_ws2812_led_count++] = h;
    ESP_LOGI(TAG, "WS2812 status LED ready on GPIO%d", (int)gpio);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Booting");

    s_log_mutex = xSemaphoreCreateMutex();

    // Init status LED (best-effort)
    recording_toggle_on_boot();

    // Decide LED wiring from preset or custom settings.
#if CONFIG_APP_STATUS_LED_PRESET_DEVKITC1
    s_status_led_kind = STATUS_LED_KIND_WS2812;
    s_status_led_ws2812_gpio = GPIO_NUM_48;
    ESP_LOGI(TAG, "Status LED preset: DevKitC-1 v1.0 (WS2812 on GPIO48)");
#elif CONFIG_APP_STATUS_LED_PRESET_DEVKITC1_V1_1
    s_status_led_kind = STATUS_LED_KIND_WS2812;
    s_status_led_ws2812_gpio = GPIO_NUM_38;
    ESP_LOGI(TAG, "Status LED preset: DevKitC-1 v1.1 (WS2812 on GPIO38)");
#elif CONFIG_APP_STATUS_LED_PRESET_WS2812_GPIO33
    s_status_led_kind = STATUS_LED_KIND_WS2812;
    s_status_led_ws2812_gpio = GPIO_NUM_33;
    ESP_LOGI(TAG, "Status LED preset: WS2812 on GPIO33");
#elif CONFIG_APP_STATUS_LED_PRESET_WS2812_GPIO21
    s_status_led_kind = STATUS_LED_KIND_WS2812;
    s_status_led_ws2812_gpio = GPIO_NUM_21;
    ESP_LOGI(TAG, "Status LED preset: WS2812 on GPIO21");
#elif CONFIG_APP_STATUS_LED_PRESET_CUSTOM
    // Custom configuration below
#if CONFIG_APP_STATUS_LED_WS2812
    s_status_led_kind = STATUS_LED_KIND_WS2812;
    s_status_led_ws2812_gpio = (gpio_num_t)CONFIG_APP_STATUS_LED_WS2812_GPIO;
    ESP_LOGI(TAG, "Status LED: WS2812 on GPIO%d (custom)", (int)s_status_led_ws2812_gpio);
#elif CONFIG_APP_STATUS_LED_GPIO_DUAL
    s_status_led_kind = STATUS_LED_KIND_GPIO_DUAL;
    s_status_led_red_gpio = (gpio_num_t)CONFIG_APP_STATUS_LED_GPIO_RED;
    s_status_led_green_gpio = (gpio_num_t)CONFIG_APP_STATUS_LED_GPIO_GREEN;
    s_status_led_gpio_active_low = CONFIG_APP_STATUS_LED_GPIO_ACTIVE_LOW;
    ESP_LOGI(TAG, "Status LED: GPIO dual (RED=GPIO%d, GREEN=GPIO%d, active_%s) (custom)",
             (int)s_status_led_red_gpio, (int)s_status_led_green_gpio,
             s_status_led_gpio_active_low ? "low" : "high");
#else
    s_status_led_kind = STATUS_LED_KIND_DISABLED;
    ESP_LOGI(TAG, "Status LED: disabled (custom)");
#endif
#else
    s_status_led_kind = STATUS_LED_KIND_DISABLED;
    ESP_LOGI(TAG, "Status LED: disabled");
#endif

    if (s_status_led_kind == STATUS_LED_KIND_WS2812) {
        // DevKitC-1 has two revisions with different RGB LED pins. To avoid confusion, try both.
#if CONFIG_APP_STATUS_LED_PRESET_DEVKITC1 || CONFIG_APP_STATUS_LED_PRESET_DEVKITC1_V1_1
        (void)ws2812_try_init_on_gpio(GPIO_NUM_48);
        (void)ws2812_try_init_on_gpio(GPIO_NUM_38);
#else
        (void)ws2812_try_init_on_gpio(s_status_led_ws2812_gpio);
#endif
        if (s_ws2812_led_count <= 0) {
            ESP_LOGW(TAG, "No WS2812 LED driver could be initialized; disabling status LED");
            s_status_led_kind = STATUS_LED_KIND_DISABLED;
        }
    } else if (s_status_led_kind == STATUS_LED_KIND_GPIO_DUAL) {
        gpio_config_t out_cfg = {
            .pin_bit_mask = (1ULL << (int)s_status_led_red_gpio) | (1ULL << (int)s_status_led_green_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&out_cfg));
    }

    // Show ON/OFF state immediately (not only after USB/MSC init).
    status_led_set_recording(s_recording_enabled);
    // (Self-test blink removed to avoid confusion with ON/OFF toggle)

    ESP_LOGI(TAG, "I2C: SDA=GPIO%d SCL=GPIO%d freq=%dHz internal_pullups=%s",
             (int)MAX30102_I2C_SDA_GPIO, (int)MAX30102_I2C_SCL_GPIO, (int)I2C_FREQ_HZ,
             CONFIG_APP_I2C_ENABLE_INTERNAL_PULLUPS ? "on" : "off");

    if (CONFIG_APP_SUPPRESS_IDF_I2C_MASTER_ERRORS) {
        // IDF prints an ESP_LOGE for each failed transaction; mute to keep the console usable.
        esp_log_level_set("i2c.master", ESP_LOG_NONE);
    }

    // I2C init (new driver, ESP-IDF v5.x)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = MAX30102_I2C_PORT,
        .sda_io_num = MAX30102_I2C_SDA_GPIO,
        .scl_io_num = MAX30102_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        // Use synchronous transactions (more reliable for sensor bring-up and gives bus errors)
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = CONFIG_APP_I2C_ENABLE_INTERNAL_PULLUPS,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    if (!i2c_probe_max3010x()) {
        ESP_LOGE(TAG, "MAX3010x not detected at 0x%02X. Check wiring/power (3.3V, GND, SDA=%d, SCL=%d).",
                 MAX30102_I2C_ADDR, (int)MAX30102_I2C_SDA_GPIO, (int)MAX30102_I2C_SCL_GPIO);
        // Keep USB MSC running even if sensor is missing
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 50000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_max30102));

    esp_err_t sensor_err = max30102_init_100hz();
    if (sensor_err != ESP_OK) {
        ESP_LOGE(TAG, "MAX3010x init failed (%s). Sensor logging will be disabled.", esp_err_to_name(sensor_err));
    }

    // Storage setup:
    // - recording ON  -> app mounts FAT directly and USB stays CDC-only
    // - recording OFF -> TinyUSB exposes FAT as MSC to the host
    if (s_recording_enabled) {
        ESP_ERROR_CHECK(app_storage_init_spiflash());
    } else {
        ESP_ERROR_CHECK(msc_storage_init_spiflash());
    }

    ESP_LOGI(TAG, "Installing TinyUSB driver (%s)",
             s_recording_enabled ? "CDC only" : "CDC + MSC composite");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &s_device_desc;
    tusb_cfg.descriptor.full_speed_config = s_recording_enabled ?
        s_cdc_only_fs_configuration_desc : s_composite_fs_configuration_desc;
    tusb_cfg.descriptor.string = s_string_desc_arr;
    tusb_cfg.descriptor.string_count = sizeof(s_string_desc_arr) / sizeof(s_string_desc_arr[0]);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB driver installed.");

    // Route logs to USB CDC without relying on esp_tinyusb helper APIs (they vary across versions).
    if (CONFIG_APP_USB_CDC_CONSOLE) {
        esp_log_set_vprintf(usb_cdc_vprintf);
        // Print a confirmation line that should appear on the CDC COM port.
        ESP_LOGI(TAG, "USB CDC logging enabled (this should show on the CDC COM port)");
    }

    // Apply mode after USB + storage are initialized.
    apply_recording_mode();

    // Start tasks regardless of initial mode. Logging is gated by `logging_allowed()`.
    // This ensures that switching mode later immediately works.
    s_sample_queue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(log_sample_t));
    if (!s_sample_queue) {
        ESP_LOGE(TAG, "Failed to create sample queue");
    } else {
        xTaskCreate(logger_task, "logger", 4096, NULL, 5, NULL);
        xTaskCreate(sensor_task, "max3010x", 4096, NULL, 10, &s_sensor_task);
    }

    xTaskCreate(cdc_command_task, "cdc_cmd", 3072, NULL, 4, NULL);

#if CONFIG_APP_RECORDING_TOGGLE_WITH_BOOT_BUTTON && (CONFIG_APP_BOOT_BUTTON_GPIO != 0)
    // Optional runtime toggle using a non-strapping GPIO button.
    xTaskCreate(boot_button_task, "boot_btn", 2048, NULL, 4, NULL);
#elif CONFIG_APP_RECORDING_TOGGLE_WITH_BOOT_BUTTON
    ESP_LOGW(TAG, "BOOT runtime toggle disabled on GPIO%d strapping pin; use CDC commands instead",
             (int)CONFIG_APP_BOOT_BUTTON_GPIO);
#endif
}

