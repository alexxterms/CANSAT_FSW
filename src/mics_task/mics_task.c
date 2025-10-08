#include "mics_task.h"
#include "mics5524.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "communications.h" // For accessing shared SensorData

extern SensorData sensorData; // Shared sensor data structure

static const char *TAGG = "GAS_TASK";

void gas_sensor_task(void *pvParameters) {
    // Initialize the MICS5524 gas sensor
    mics5524_init();

    while (1) {
        // Read gas sensor value
        float gas_value = mics5524_read();

        // Store gas value in shared SensorData struct
        sensorData.gas_level = gas_value;

        // Log the gas value
        ESP_LOGI(TAGG, "Gas Level: %.2f", gas_value);

        // Delay before the next reading (1Hz sampling rate)
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}