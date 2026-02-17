#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "max30102.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAX30102 Heart Rate Monitor");
    
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, NULL);
}
