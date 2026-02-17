#ifndef MAX30102_H
#define MAX30102_H

#include "esp_err.h"

/**
 * @brief Read FIFO data from MAX30102
 * 
 * @param red_led Pointer to store Red LED value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t max30102_read_fifo(uint32_t *red_led);

/**
 * @brief MAX30102 task for continuous heart rate monitoring
 * 
 * @param pvParameters Task parameters
 */
void max30102_task(void *pvParameters);

#endif // MAX30102_H
