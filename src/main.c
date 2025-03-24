#include <stdio.h>
#include "bmp280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "BMP280";

void app_main() {
    ESP_LOGI(TAG, "Initializing BMP280...");

    bmp280_t bmp280_dev;  // ✅ Correct type
    bmp280_params_t params;
    
    bmp280_init_default_params(&params);  // Initialize default parameters

    // ✅ Initialize descriptor correctly
    ESP_ERROR_CHECK(bmp280_init_desc(&bmp280_dev, BMP280_I2C_ADDRESS_0, I2C_MASTER_NUM, 21, 22));

    // ✅ Pass `bmp280_params_t` to init function
    ESP_ERROR_CHECK(bmp280_init(&bmp280_dev, &params));

    while (1) {
        float temperature, pressure;
        if (bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL) == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %.2f°C, Pressure: %.2f hPa", temperature, pressure / 100);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data!");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
