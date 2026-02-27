#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
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

static volatile tinyusb_msc_mount_point_t s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
static volatile bool s_msc_transition = false;

static SemaphoreHandle_t s_log_mutex;
static FILE *s_log_msc = NULL;

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

static led_strip_handle_t s_led = NULL;

// ----------------------------- I2C helpers ---------------------------------

#define I2C_XFER_TIMEOUT_MS  100
#define I2C_RETRY_COUNT      3

// Forward declarations (needed for recovery path)
static bool i2c_probe_max3010x(void);
static esp_err_t max30102_init_100hz(void);

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

    return max30102_init_100hz();
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
    // - ADC range: 4096nA (01 << 5)
    // - Sample rate: 100Hz (011 << 2)
    // - Pulse width: 411us / 18-bit (11)
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_SPO2_CONFIG, 0x2F), TAG, "spo2 config");

    // LED pulse amplitudes (tune for your sensor / signal level)
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_LED1_PA, 0x24), TAG, "led1");
    ESP_RETURN_ON_ERROR(i2c_reg_write_u8(s_max30102, MAX30102_REG_LED2_PA, 0x24), TAG, "led2");

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
    fprintf(*fp, "time_ms,IR,RED\n");
    fflush(*fp);
    return ESP_OK;
}

static void log_close_all(void)
{
    if (s_log_msc) {
        fflush(s_log_msc);
        fclose(s_log_msc);
        s_log_msc = NULL;
    }
}

static bool logging_allowed(void)
{
    return s_recording_enabled &&
           (!s_msc_transition) &&
           (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP);
}

static void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
#if CONFIG_APP_STATUS_LED_PRESET_DEVKITC1 || CONFIG_APP_STATUS_LED_PRESET_CUSTOM
    if (s_status_led_kind != STATUS_LED_KIND_WS2812) {
        return;
    }
    if (!s_led) {
        return;
    }
    // led_strip uses RGB order in API regardless of pixel format.
    (void)led_strip_set_pixel(s_led, 0, r, g, b);
    (void)led_strip_refresh(s_led);
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
            status_led_set_rgb(0, 16, 0);
        } else {
            // OFF = red
            status_led_set_rgb(16, 0, 0);
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

static void recording_toggle_on_boot(void)
{
    // Toggle persistent state on every boot (RESET acts like an on/off switch).
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
    err = nvs_get_u8(h, "rec", &v);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        v = 0; // so first boot toggles to ON
    } else {
        ESP_ERROR_CHECK(err);
    }

    v = (uint8_t)(!v);
    ESP_ERROR_CHECK(nvs_set_u8(h, "rec", v));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);

    s_recording_enabled = (v != 0);
    ESP_LOGI(TAG, "Recording mode: %s (toggled by RESET)", s_recording_enabled ? "ON (record-only, drive hidden)" : "OFF (drive exposed)");
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

static void apply_recording_mode(void)
{
    // Ensure files are closed before switching ownership.
    if (s_log_mutex) {
        if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            log_close_all();
            xSemaphoreGive(s_log_mutex);
        }
    }

    if (!s_msc_storage) {
        return;
    }

    if (s_recording_enabled) {
        // Hide disk from the host and mount to APP for logging
        ESP_ERROR_CHECK(tinyusb_msc_set_storage_mount_point(s_msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP));
        s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;
    } else {
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

    const int max_samples_per_bulk = 10;
    uint8_t fifo_bulk[6 * 10];

    while (true) {
        uint8_t unread = 0;
        esp_err_t err = max30102_get_unread_samples(&unread);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FIFO ptr read failed (%s)", esp_err_to_name(err));
            consecutive_i2c_errors++;
            if (consecutive_i2c_errors >= 5) {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
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
                if (consecutive_i2c_errors >= 5) {
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
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

                acc_ir += raw_ir;
                acc_red += raw_red;
                acc_n++;

                if (acc_n >= DOWNSAMPLE_FACTOR) {
                    uint32_t ir_ds = (uint32_t)(acc_ir / DOWNSAMPLE_FACTOR);
                    uint32_t red_ds = (uint32_t)(acc_red / DOWNSAMPLE_FACTOR);
                    uint64_t t_ms = (uint64_t)(esp_timer_get_time() / 1000);

                    ESP_LOGI(TAG, "time_ms=%"PRIu64" IR=%"PRIu32" RED=%"PRIu32"%s",
                             t_ms, ir_ds, red_ds, logging_allowed() ? "" : " (paused)");

                    if (logging_allowed() && s_log_mutex) {
                        if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                            // MSC/FAT log (only when storage is mounted to APP)
                            if (ensure_csv_header(&s_log_msc, MSC_LOG_PATH) != ESP_OK) {
                                ESP_LOGW(TAG, "MSC log open failed (%s)", MSC_LOG_PATH);
                            }

                            if (s_log_msc) {
                                fprintf(s_log_msc, "%"PRIu64",%"PRIu32",%"PRIu32"\n", t_ms, ir_ds, red_ds);
                                fflush(s_log_msc);
                            }
                            xSemaphoreGive(s_log_mutex);
                        }
                    }

                    acc_ir = 0;
                    acc_red = 0;
                    acc_n = 0;
                }
            }

            remaining = (uint8_t)(remaining - n);
        }

        // Let FIFO accumulate a bit; avoids hammering I2C.
        vTaskDelay(pdMS_TO_TICKS(10));
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

// USB composite configuration descriptor: CDC (serial) + MSC (drive)
#define EPNUM_MSC_OUT       0x01
#define EPNUM_MSC_IN        0x81
#define EPNUM_CDC_NOTIF     0x82
#define EPNUM_CDC_OUT       0x03
#define EPNUM_CDC_IN        0x83

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

    // If USB isn't ready yet, fall back to default stdout (usually UART).
    if (!tud_ready() || !tud_cdc_connected()) {
        return vprintf(fmt, ap);
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
    ESP_LOGI(TAG, "Status LED preset: DevKitC-1 (WS2812 on GPIO48)");
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
        led_strip_config_t strip_config = {
            .strip_gpio_num = s_status_led_ws2812_gpio,
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
        esp_err_t led_err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
        if (led_err != ESP_OK) {
            ESP_LOGW(TAG, "WS2812 status LED init failed (%s) on GPIO%d",
                     esp_err_to_name(led_err), (int)s_status_led_ws2812_gpio);
            s_led = NULL;
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

    // USB MSC storage on internal flash (FATFS + wear levelling)
    ESP_ERROR_CHECK(msc_storage_init_spiflash());

    ESP_LOGI(TAG, "Installing TinyUSB driver (CDC + MSC composite)");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &s_device_desc;
    tusb_cfg.descriptor.full_speed_config = s_composite_fs_configuration_desc;
    tusb_cfg.descriptor.string = s_string_desc_arr;
    tusb_cfg.descriptor.string_count = sizeof(s_string_desc_arr) / sizeof(s_string_desc_arr[0]);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB driver installed. When the PC mounts the drive, logging is paused. After safe-eject, logging resumes.");

    // Route logs to USB CDC without relying on esp_tinyusb helper APIs (they vary across versions).
    if (CONFIG_APP_USB_CDC_CONSOLE) {
        esp_log_set_vprintf(usb_cdc_vprintf);
    }

    // Apply mode after USB + storage are initialized.
    apply_recording_mode();

    if (s_recording_enabled && sensor_err == ESP_OK) {
        xTaskCreate(sensor_task, "max3010x", 4096, NULL, 10, &s_sensor_task);
    }
}

