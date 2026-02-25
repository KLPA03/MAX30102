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
#include "wear_levelling.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

// ----------------------------- User config ---------------------------------

// I2C pins (adjust for your ESP32-S3 board)
#ifndef MAX30102_I2C_SDA_GPIO
#define MAX30102_I2C_SDA_GPIO  GPIO_NUM_5
#endif

#ifndef MAX30102_I2C_SCL_GPIO
#define MAX30102_I2C_SCL_GPIO  GPIO_NUM_4
#endif

#ifndef MAX30102_I2C_PORT
#define MAX30102_I2C_PORT      I2C_NUM_0
#endif

#define MAX30102_I2C_ADDR      0x57
#define I2C_FREQ_HZ            400000

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

static const char *TAG = "max30101_msc";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_max30102;

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t s_msc_storage = NULL;

static volatile tinyusb_msc_mount_point_t s_msc_mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
static volatile bool s_msc_transition = false;

static SemaphoreHandle_t s_log_mutex;
static FILE *s_log_msc = NULL;

// ----------------------------- I2C helpers ---------------------------------

static bool i2c_probe_max3010x(void)
{
    // Probe with retries to avoid aborting on wiring/power-up timing issues
    for (int i = 0; i < 10; i++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, MAX30102_I2C_ADDR, 50);
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
    return i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(100));
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
        ESP_LOGI(TAG, "MAX3010x PART_ID=0x%02X (MAX30102 is 0x15; MAX30101 differs by revision)", part_id);
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
    uint8_t wr = 0, rd = 0;
    ESP_RETURN_ON_ERROR(max30102_read_u8(MAX30102_REG_FIFO_WR_PTR, &wr), TAG, "wr ptr read failed");
    ESP_RETURN_ON_ERROR(max30102_read_u8(MAX30102_REG_FIFO_RD_PTR, &rd), TAG, "rd ptr read failed");
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
    return (!s_msc_transition) && (s_msc_mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP);
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

    tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB, // Start exposed to PC; logging resumes after PC ejects
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

// ----------------------------- Sensor / logger task -------------------------

static void sensor_task(void *arg)
{
    (void)arg;
    uint64_t acc_ir = 0;
    uint64_t acc_red = 0;
    int acc_n = 0;

    while (true) {
        uint8_t unread = 0;
        esp_err_t err = max30102_get_unread_samples(&unread);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FIFO ptr read failed (%s)", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (unread == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        for (uint8_t i = 0; i < unread; i++) {
            uint32_t ir = 0, red = 0;
            err = max30102_read_fifo_sample(&ir, &red);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "FIFO sample read failed (%s)", esp_err_to_name(err));
                break;
            }

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
    "ESP32-S3 MAX30101 Logger",      // 2: Product
    "0001",                         // 3: Serial
};

// ----------------------------- app_main -------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "Booting");

    s_log_mutex = xSemaphoreCreateMutex();

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
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_max30102));

    esp_err_t sensor_err = max30102_init_100hz();
    if (sensor_err != ESP_OK) {
        ESP_LOGE(TAG, "MAX3010x init failed (%s). Sensor logging will be disabled.", esp_err_to_name(sensor_err));
    }

    // USB MSC storage on internal flash (FATFS + wear levelling)
    ESP_ERROR_CHECK(msc_storage_init_spiflash());

    ESP_LOGI(TAG, "Installing TinyUSB driver (MSC device)");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &s_device_desc;
    tusb_cfg.descriptor.string = s_string_desc_arr;
    tusb_cfg.descriptor.string_count = sizeof(s_string_desc_arr) / sizeof(s_string_desc_arr[0]);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB driver installed. When the PC mounts the drive, logging is paused. After safe-eject, logging resumes.");

    if (sensor_err == ESP_OK) {
        xTaskCreate(sensor_task, "max30101", 4096, NULL, 10, NULL);
    }
}

