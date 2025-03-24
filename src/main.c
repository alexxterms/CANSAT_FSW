#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme280_driver.h"

#define TAG "MAIN"

void app_main() {
    ESP_LOGI(TAG, "Initializing BME280...");
    bme280_init();  // Initialize the BME280 sensor

    while (1) {
        ESP_LOGI(TAG, "Reading BME280 data...");
        bme280_check_device();
        bme280_read_sensor_data();  // Read and print sensor data
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds
    }
}
