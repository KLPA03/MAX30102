#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
} max30101_t;

typedef struct {
    int i2c_port;
    int sda_io;
    int scl_io;
    uint32_t i2c_freq_hz;
    uint8_t i2c_addr;
} max30101_i2c_config_t;

/**
 * @brief Create I2C bus + device handle for MAX30101.
 *
 * @note Caller must eventually call max30101_deinit().
 */
esp_err_t max30101_init(max30101_t *out, const max30101_i2c_config_t *cfg);

/**
 * @brief Reset + configure MAX30101 for SpO2 mode (Red+IR).
 */
esp_err_t max30101_configure_spo2(max30101_t *s);

/**
 * @brief Read one Red+IR sample from FIFO (18-bit values).
 *
 * @param red  Returned Red sample (0..0x3FFFF)
 * @param ir   Returned IR sample (0..0x3FFFF)
 */
esp_err_t max30101_read_fifo_red_ir(max30101_t *s, uint32_t *red, uint32_t *ir);

esp_err_t max30101_read_reg(max30101_t *s, uint8_t reg, uint8_t *val);
esp_err_t max30101_write_reg(max30101_t *s, uint8_t reg, uint8_t val);

esp_err_t max30101_deinit(max30101_t *s);

#ifdef __cplusplus
}
#endif

