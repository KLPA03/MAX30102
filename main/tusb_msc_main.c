/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <errno.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "wear_levelling.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tusb_msc_storage.h"

#include "esp_console_dev_uart.h"
#include "esp_console_repl.h"

#include "max30101.h"

static const char *TAG = "example_main";

/* TinyUSB descriptors
   ********************************************************************* */
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,

    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

static uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
};

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // Espressif VID (change for your product)
    .idProduct = 0x4002,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: English (0x0409)
    "TinyUSB",                      // 1: Manufacturer
    "ESP32-S3 MAX30101 Logger",      // 2: Product
    "00000001",                     // 3: Serial
    "MSC",                          // 4: MSC
};
/*********************************************************************** TinyUSB descriptors*/

#define BASE_PATH          "/data"
#define LOG_FILE_PATH      BASE_PATH "/max30101.csv"
#define PROMPT_STR         CONFIG_IDF_TARGET

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool s_storage_mounted = false;

static max30101_t s_max30101;
static bool s_log_enabled = CONFIG_EXAMPLE_LOG_ENABLED_DEFAULT;
static uint32_t s_log_interval_ms = CONFIG_EXAMPLE_LOG_INTERVAL_MS;

static void storage_ls(void);
static esp_err_t storage_mount(void);
static esp_err_t storage_unmount(void);
static void max30101_log_task(void *arg);

static int console_mount(int argc, char **argv);
static int console_unmount(int argc, char **argv);
static int console_read(int argc, char **argv);
static int console_write(int argc, char **argv);
static int console_size(int argc, char **argv);
static int console_status(int argc, char **argv);
static int console_log_start(int argc, char **argv);
static int console_log_stop(int argc, char **argv);
static int console_log_once(int argc, char **argv);
static int console_max_reg_read(int argc, char **argv);
static int console_max_reg_write(int argc, char **argv);
static int console_exit(int argc, char **argv);

static const esp_console_cmd_t cmds[] = {
    {
        .command = "read",
        .help = "read " BASE_PATH "/README.MD and print its contents",
        .hint = NULL,
        .func = &console_read,
    },
    {
        .command = "write",
        .help = "create file " BASE_PATH "/README.MD if it does not exist",
        .hint = NULL,
        .func = &console_write,
    },
    {
        .command = "size",
        .help = "show storage size and sector size",
        .hint = NULL,
        .func = &console_size,
    },
    {
        .command = "mount",
        .help = "mount storage to " BASE_PATH " (application access)",
        .hint = NULL,
        .func = &console_mount,
    },
    {
        .command = "expose",
        .help = "unmount storage (host PC access over USB MSC)",
        .hint = NULL,
        .func = &console_unmount,
    },
    {
        .command = "status",
        .help = "show storage/logging status",
        .hint = NULL,
        .func = &console_status,
    },
    {
        .command = "log_start",
        .help = "start CSV logging to " LOG_FILE_PATH,
        .hint = NULL,
        .func = &console_log_start,
    },
    {
        .command = "log_stop",
        .help = "stop CSV logging",
        .hint = NULL,
        .func = &console_log_stop,
    },
    {
        .command = "log_once",
        .help = "read one sample and append one CSV row (if mounted)",
        .hint = NULL,
        .func = &console_log_once,
    },
    {
        .command = "maxr",
        .help = "read MAX30101 register: maxr <reg_hex>",
        .hint = NULL,
        .func = &console_max_reg_read,
    },
    {
        .command = "maxw",
        .help = "write MAX30101 register: maxw <reg_hex> <val_hex>",
        .hint = NULL,
        .func = &console_max_reg_write,
    },
    {
        .command = "exit",
        .help = "exit from application",
        .hint = NULL,
        .func = &console_exit,
    }
};

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    ESP_LOGI(TAG, "Initializing wear levelling");

    const esp_partition_t *data_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG, "Failed to find FATFS partition. Check partition table.");
        return ESP_ERR_NOT_FOUND;
    }
    return wl_mount(data_partition, wl_handle);
}

static esp_err_t storage_mount(void)
{
    if (s_storage_mounted) {
        return ESP_OK;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGW(TAG, "Storage is in use by USB host; can't mount now");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Mounting storage at %s ...", BASE_PATH);
    esp_err_t err = tinyusb_msc_storage_mount(BASE_PATH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(err));
        return err;
    }
    s_storage_mounted = true;
    storage_ls();
    return ESP_OK;
}

static esp_err_t storage_unmount(void)
{
    if (!s_storage_mounted) {
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Unmounting storage ...");
    esp_err_t err = tinyusb_msc_storage_unmount();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unmount failed: %s", esp_err_to_name(err));
        return err;
    }
    s_storage_mounted = false;
    return ESP_OK;
}

static void storage_ls(void)
{
    ESP_LOGI(TAG, "\nls %s:", BASE_PATH);
    DIR *dh = opendir(BASE_PATH);
    if (!dh) {
        if (errno == ENOENT) {
            ESP_LOGE(TAG, "Directory doesn't exist: %s", BASE_PATH);
        } else {
            ESP_LOGE(TAG, "Unable to read directory: %s", BASE_PATH);
        }
        return;
    }
    struct dirent *d;
    while ((d = readdir(dh)) != NULL) {
        printf("%s\n", d->d_name);
    }
    closedir(dh);
}

static esp_err_t ensure_csv_header(void)
{
    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == 0 && st.st_size > 0) {
        return ESP_OK;
    }

    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (!f) {
        return ESP_FAIL;
    }
    fprintf(f, "timestamp_ms,red,ir\n");
    fclose(f);
    return ESP_OK;
}

static esp_err_t append_csv_row(uint32_t red, uint32_t ir)
{
    if (!s_storage_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ensure_csv_header();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ensure header failed: %s", esp_err_to_name(err));
        return err;
    }

    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (!f) {
        return ESP_FAIL;
    }
    int64_t ms = esp_timer_get_time() / 1000;
    fprintf(f, "%" PRIi64 ",%" PRIu32 ",%" PRIu32 "\n", ms, red, ir);
    fclose(f);
    return ESP_OK;
}

static void max30101_log_task(void *arg)
{
    (void)arg;
    while (true) {
        if (!s_log_enabled) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!s_storage_mounted || tinyusb_msc_storage_in_use_by_usb_host()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint32_t red = 0, ir = 0;
        esp_err_t err = max30101_read_fifo_red_ir(&s_max30101, &red, &ir);
        if (err == ESP_OK) {
            (void)append_csv_row(red, ir);
        } else {
            ESP_LOGW(TAG, "MAX30101 read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(s_log_interval_ms));
    }
}

static int console_mount(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return storage_mount() == ESP_OK ? 0 : -1;
}

static int console_unmount(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGW(TAG, "Host already using MSC");
    }
    return storage_unmount() == ESP_OK ? 0 : -1;
}

static int console_read(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_storage_mounted) {
        ESP_LOGE(TAG, "storage not mounted");
        return -1;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGE(TAG, "storage exposed over USB; app can't read");
        return -1;
    }
    const char *filename = BASE_PATH "/README.MD";
    FILE *ptr = fopen(filename, "r");
    if (ptr == NULL) {
        ESP_LOGE(TAG, "Filename not present: %s", filename);
        return -1;
    }
    char buf[256];
    while (fgets(buf, sizeof(buf), ptr) != NULL) {
        printf("%s", buf);
    }
    fclose(ptr);
    return 0;
}

static int console_write(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_storage_mounted) {
        ESP_LOGE(TAG, "storage not mounted");
        return -1;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGE(TAG, "storage exposed over USB; app can't write");
        return -1;
    }
    const char *filename = BASE_PATH "/README.MD";
    FILE *fd = fopen(filename, "r");
    if (!fd) {
        ESP_LOGW(TAG, "README.MD doesn't exist yet, creating");
        fd = fopen(filename, "w");
        if (!fd) {
            ESP_LOGE(TAG, "Failed to create %s", filename);
            return -1;
        }
        fprintf(fd, "MAX30101 CSV logger over USB Mass Storage (SPI flash)\n");
        fprintf(fd, "Use 'log_start' to append samples to %s\n", LOG_FILE_PATH);
        fprintf(fd, "Use 'expose' to unmount and let the PC read the drive.\n");
        fclose(fd);
    } else {
        fclose(fd);
    }
    return 0;
}

static int console_size(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_storage_mounted) {
        ESP_LOGE(TAG, "storage not mounted");
        return -1;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGE(TAG, "storage exposed over USB; app can't access");
        return -1;
    }
    uint32_t sec_count = tinyusb_msc_storage_get_sector_count();
    uint32_t sec_size = tinyusb_msc_storage_get_sector_size();
    printf("Storage Capacity %" PRIu64 "MB\n", ((uint64_t)sec_count) * sec_size / (1024 * 1024));
    printf("Sector Size %u bytes\n", sec_size);
    return 0;
}

static int console_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("storage mounted: %s\n", s_storage_mounted ? "Yes" : "No");
    printf("storage exposed over USB: %s\n", tinyusb_msc_storage_in_use_by_usb_host() ? "Yes" : "No");
    printf("logging enabled: %s\n", s_log_enabled ? "Yes" : "No");
    printf("log interval: %u ms\n", (unsigned)s_log_interval_ms);
    return 0;
}

static int console_log_start(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_log_enabled = true;
    printf("logging started\n");
    return 0;
}

static int console_log_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_log_enabled = false;
    printf("logging stopped\n");
    return 0;
}

static int console_log_once(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_storage_mounted || tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGE(TAG, "storage not available for app writes (mount it and ensure host isn't using it)");
        return -1;
    }
    uint32_t red = 0, ir = 0;
    esp_err_t err = max30101_read_fifo_red_ir(&s_max30101, &red, &ir);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30101 read failed: %s", esp_err_to_name(err));
        return -1;
    }
    err = append_csv_row(red, ir);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CSV append failed: %s", esp_err_to_name(err));
        return -1;
    }
    printf("appended: red=%" PRIu32 " ir=%" PRIu32 "\n", red, ir);
    return 0;
}

static int console_max_reg_read(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: maxr <reg_hex>\n");
        return -1;
    }
    uint32_t reg = strtoul(argv[1], NULL, 0);
    uint8_t val = 0;
    esp_err_t err = max30101_read_reg(&s_max30101, (uint8_t)reg, &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read reg 0x%02" PRIX32 " failed: %s", reg, esp_err_to_name(err));
        return -1;
    }
    printf("reg 0x%02" PRIX32 " = 0x%02X\n", reg, val);
    return 0;
}

static int console_max_reg_write(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: maxw <reg_hex> <val_hex>\n");
        return -1;
    }
    uint32_t reg = strtoul(argv[1], NULL, 0);
    uint32_t val = strtoul(argv[2], NULL, 0);
    esp_err_t err = max30101_write_reg(&s_max30101, (uint8_t)reg, (uint8_t)val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write reg 0x%02" PRIX32 " failed: %s", reg, esp_err_to_name(err));
        return -1;
    }
    printf("wrote reg 0x%02" PRIX32 " = 0x%02" PRIX32 "\n", reg, val);
    return 0;
}

static int console_exit(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    tinyusb_msc_storage_deinit();
    (void)max30101_deinit(&s_max30101);
    printf("Application Exiting\n");
    exit(0);
    return 0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing storage...");
    ESP_ERROR_CHECK(storage_init_spiflash(&s_wl_handle));

    const tinyusb_msc_spiflash_config_t config_spi = {
        .wl_handle = s_wl_handle
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));

    // Mount for application access by default (logger writes files here).
    ESP_ERROR_CHECK(storage_mount());

    ESP_LOGI(TAG, "Initializing MAX30101 over I2C...");
    max30101_i2c_config_t i2c_cfg = {
        .i2c_port = CONFIG_EXAMPLE_I2C_PORT_NUM,
        .sda_io = CONFIG_EXAMPLE_I2C_SDA,
        .scl_io = CONFIG_EXAMPLE_I2C_SCL,
        .i2c_freq_hz = CONFIG_EXAMPLE_I2C_FREQ_HZ,
        .i2c_addr = CONFIG_EXAMPLE_MAX30101_I2C_ADDR,
    };
    ESP_ERROR_CHECK(max30101_init(&s_max30101, &i2c_cfg));
    ESP_ERROR_CHECK(max30101_configure_spo2(&s_max30101));

    xTaskCreate(max30101_log_task, "max30101_log", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "USB MSC initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB MSC initialization DONE");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = 64;

    esp_console_register_help_command();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    for (int i = 0; i < (int)(sizeof(cmds) / sizeof(cmds[0])); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

