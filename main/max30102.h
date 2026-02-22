#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize MAX30102 sensor
 */
void max30102_init(void);

/**
 * @brief Read heart rate data with 20 Hz downsampling
 * 
 * @param red_led Pointer to store Red LED value
 * @param ir_led Pointer to store IR LED value
 * @param data_ready Pointer to store data ready flag
 */
void max30102_read_heart_rate_20hz(uint32_t *red_led, uint32_t *ir_led, bool *data_ready);

/**
 * @brief MAX30102 task for continuous monitoring
 * 
 * @param pvParameters Task parameters
 */
void max30102_task(void *pvParameters);

#endif // MAX30102_H
