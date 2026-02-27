#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wear_levelling.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "tinyusb_msc.h"

#include "led_strip.h"

// ----------------------------- User config ---------------------------------

// I2C pins (adjust for your ESP32-S3 board)
#ifndef MAX30102_I2C_SDA_GPIO
#define MAX30102_I2C_SDA_GPIO  GPIO_NUM_8
#endif

#ifndef MAX30102_I2C_SCL_GPIO
#define MAX30102_I2C_SCL_GPIO  GPIO_NUM_9
#endif

#ifndef MAX30102_I2C_PORT
#define MAX30102_I2C_PORT      I2C_NUM_0
#endif

#define MAX30102_I2C_ADDR      0x57
// 100kHz is much more tolerant of breadboards/long wires and weak pull-ups.
#define I2C_FREQ_HZ            100000

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

// ESP32-S3 DevKitC-1 commonly has a single WS2812 RGB LED on GPIO48.
#ifndef STATUS_LED_GPIO
#define STATUS_LED_GPIO GPIO_NUM_48
#endif

static led_strip_handle_t s_led = NULL;

// ----------------------------- I2C helpers ---------------------------------

#define I2C_XFER_TIMEOUT_MS  100
#define I2C_RETRY_COUNT      3

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
        vTaskDelay(pdMS_TO_TICKS(2));
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
        vTaskDelay(pdMS_TO_TICKS(2));
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
        ESP_LOGI(TAG, "MAX3010x PART_ID=0x%02X (0x15 indicates MAX30102)", part_id);
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

static esp_err_t max30102_read_fifo_sample(uint32_t *ir, uint32_t *red)
{
    uint8_t data[6] = {0};
    ESP_RETURN_ON_ERROR(i2c_reg_read(s_max30102, MAX30102_REG_FIFO_DATA, data, sizeof(data)),
                        TAG, "fifo read failed");

    // In SpO2 mode: FIFO contains RED then IR, 3 bytes each (18 bits used).
    uint32_t raw_red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t raw_ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    raw_red &= 0x3FFFF;
    raw_ir  &= 0x3FFFF;

    *red = raw_red;
    *ir = raw_ir;
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
    if (!s_led) {
        return;
    }
    // led_strip uses RGB order in API regardless of pixel format.
    (void)led_strip_set_pixel(s_led, 0, r, g, b);
    (void)led_strip_refresh(s_led);
}

static void status_led_set_recording(bool recording_enabled)
{
    if (recording_enabled) {
        // ON = green
        status_led_set_rgb(0, 16, 0);
    } else {
        // OFF = red
        status_led_set_rgb(16, 0, 0);
    }
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
        ESP_LOGI(TAG, "MSC storage owner now: %s",
                 (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP) ? "APP (logging allowed)" : "USB HOST (logging paused)");
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
    ESP_RETURN_ON_ERROR(tinyusb_msc_set_storage_callback(storage_mount_changed_cb, NULL), TAG, "set_storage_callback failed");

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

    while (true) {
        uint8_t unread = 0;
        esp_err_t err = max30102_get_unread_samples(&unread);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FIFO ptr read failed (%s)", esp_err_to_name(err));
            consecutive_i2c_errors++;
            if (consecutive_i2c_errors >= 10) {
                ESP_LOGW(TAG, "Too many I2C errors, re-initializing MAX3010x...");
                (void)max30102_init_100hz();
                consecutive_i2c_errors = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        consecutive_i2c_errors = 0;

        if (unread == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        for (uint8_t i = 0; i < unread; i++) {
            uint32_t ir = 0, red = 0;
            err = max30102_read_fifo_sample(&ir, &red);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "FIFO sample read failed (%s)", esp_err_to_name(err));
                consecutive_i2c_errors++;
                break;
            }
            consecutive_i2c_errors = 0;

            acc_ir += ir;
            acc_red += red;
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

        // Yield; FIFO can contain multiple samples if task was delayed.
        vTaskDelay(pdMS_TO_TICKS(2));
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

void app_main(void)
{
    ESP_LOGI(TAG, "Booting");

    s_log_mutex = xSemaphoreCreateMutex();

    // Init status LED (best-effort)
    led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
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
        ESP_LOGW(TAG, "Status LED init failed (%s) on GPIO%d", esp_err_to_name(led_err), (int)STATUS_LED_GPIO);
        s_led = NULL;
    }

    recording_toggle_on_boot();

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
        .flags.enable_internal_pullup = true,
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

    // Init USB CDC ACM (creates a COM port on the PC). Route logs/stdout to it.
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_console_init(TINYUSB_CDC_ACM_0));

    // Apply mode after USB + storage are initialized.
    apply_recording_mode();

    if (s_recording_enabled && sensor_err == ESP_OK) {
        xTaskCreate(sensor_task, "max3010x", 4096, NULL, 10, &s_sensor_task);
    }
}

