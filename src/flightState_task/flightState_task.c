#include "flightState_task.h"
#include "communications.h" // For accessing shared SensorData
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_FLIGHT_STATE = "FLIGHT_STATE_TASK";

// Global variables for flight state management
extern FlightState currentState;
extern SensorData sensorData;
static float previousAltitude = 0;

// Function to update the flight state based on sensor data
void update_flight_state(float acceleration, float altitude, float velocity) {
    switch (currentState) {
        case PRE_LAUNCH:
            if (acceleration > 15) { // When above 15G is detected (this is a guesstimation)
                currentState = ASCENT;
                ESP_LOGI(TAG_FLIGHT_STATE, "State changed to ASCENT");
            }
            break;

        case ASCENT:
            if (velocity <= 0) { // Gotta trial and error this one
                currentState = APOGEE;
                ESP_LOGI(TAG_FLIGHT_STATE, "State changed to APOGEE");
            }
            break;

        case APOGEE:
            if (altitude - previousAltitude > 10) { // Altitude drop (guesstimation)
                currentState = DESCENT;
                ESP_LOGI(TAG_FLIGHT_STATE, "State changed to DESCENT");
            }
            break;

        case DESCENT:
            if (velocity < 1 && altitude < 5) { // Near ground (guesstimation)
                currentState = LANDING;
                ESP_LOGI(TAG_FLIGHT_STATE, "State changed to LANDING");
            }
            break;

        case LANDING:
            ESP_LOGI(TAG_FLIGHT_STATE, "State is LANDING");
            break;

        default:
            ESP_LOGW(TAG_FLIGHT_STATE, "Unknown flight state");
            break;
    }

    // Update the previous altitude for the next iteration
    previousAltitude = altitude;
}
/*
// Task to manage flight states
void flight_state_task(void *pvParameters) {
    while (1) {
        // Retrieve sensor data
        float accel = sensorData.ac;
        float altitude = sensorData.altitude; // Replace with actual altitude retrieval function
        float velocity = sensorData.velocity; // Replace with actual velocity retrieval function

        // Update the flight state
        update_flight_state(accel, altitude, velocity);

        // Delay before the next state update
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz update rate
    }
} */