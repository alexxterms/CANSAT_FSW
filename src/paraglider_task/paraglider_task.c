#include "paraglider_task.h"
#include "autonomous_paraglider.h"
#include "communications.h" // For accessing shared SensorData
#include "gps.h"            // For GPS data
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern SensorData sensorData; // Shared sensor data structure
extern FlightState currentState; // Current flight state from main flight software

static const char *TAG_PARAGLIDER = "PARAGLIDER_NAV";

void paraglider_navigation_task(void *pvParameters) {
    // Initialize the paraglider system
    esp_err_t ret = paraglider_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARAGLIDER, "Failed to initialize paraglider: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    // Example: Set a target location (you can change these coordinates)
    double target_lat = 37.7749;  // San Francisco example
    double target_lon = -122.4194;
    paraglider_set_target(target_lat, target_lon);

    // Example: Add some waypoints (optional)
    waypoint_t waypoint1 = {
        .position = {.latitude = 37.7700, .longitude = -122.4100, .altitude = 0},
        .arrival_radius_m = 30.0f,
        .name = "Waypoint 1"
    };
    paraglider_add_waypoint(&waypoint1);

    // Configure loitering parameters (optional - defaults will be used if not set)
    paraglider_set_loiter_params(target_lat, target_lon, 150.0f, LOITER_DIRECTION_CW);

    // Wait a bit for GPS to get a fix
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Start navigation
    paraglider_start_navigation();
    ESP_LOGI(TAG_PARAGLIDER, "Paraglider navigation started");

    while (1) {
        // Update paraglider with current flight state from main flight software
        flight_state_t flight_state;
        switch (currentState) {
            case PRE_LAUNCH:    flight_state = FLIGHT_PRE_LAUNCH; break;
            case ASCENT:        flight_state = FLIGHT_ASCENT; break;
            case APOGEE:        flight_state = FLIGHT_APOGEE; break;
            case DESCENT:       flight_state = FLIGHT_DESCENT; break;
            case LANDING:       flight_state = FLIGHT_LANDING; break;
            default:            flight_state = FLIGHT_PRE_LAUNCH; break;
        }
        paraglider_update_flight_state(flight_state);

        // Update current position from GPS data
        gps_fix_t gps_fix;
        if (gps_get_fix(&gps_fix) && gps_fix.valid) {
            double current_lat = gps_fix.lat_e7 / 1e7;
            double current_lon = gps_fix.lon_e7 / 1e7;
            float current_heading = sensorData.yaw; // Use IMU heading
            float current_altitude = gps_fix.alt_cm / 100.0f; // Convert cm to meters
            
            // Update paraglider with current position, heading, and altitude
            paraglider_update_position(current_lat, current_lon, current_heading, current_altitude);
            
            // Perform navigation update (this does the actual steering)
            ret = paraglider_navigation_update();
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG_PARAGLIDER, "Navigation update failed: %s", esp_err_to_name(ret));
            }
            
            // Log current navigation mode and status
            nav_mode_t nav_mode = paraglider_get_nav_mode();
            static nav_mode_t prev_nav_mode = NAV_MODE_WAYPOINT;
            if (nav_mode != prev_nav_mode) {
                const char* mode_names[] = {"WAYPOINT", "LOITER", "LANDING", "LANDED"};
                ESP_LOGI(TAG_PARAGLIDER, "Navigation mode changed to: %s", mode_names[nav_mode]);
                prev_nav_mode = nav_mode;
            }
            
            // Check if final landing is complete
            if (paraglider_is_target_reached() && nav_mode == NAV_MODE_LANDED) {
                ESP_LOGI(TAG_PARAGLIDER, "Mission complete! Paraglider has landed at target.");
                // Mission complete - you can set new target or perform post-landing tasks
                vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds before continuing
            }
        } else {
            ESP_LOGW(TAG_PARAGLIDER, "No valid GPS fix, navigation paused");
            paraglider_set_neutral(); // Keep servos neutral if no GPS
        }
        
        // Update at 5Hz (200ms) - this is the main navigation control loop
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}