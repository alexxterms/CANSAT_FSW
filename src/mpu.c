/*Things to do : Complete GPS TASK --> Test
                 Try reading different gases from MICS --> Test
                 Complete SD Logging --> Test
                 Test SENSORS altogether 
                 Start LoRa Packeting 
                 --> Optimize Optimize Optimize
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"
#include "driver/uart.h" // Add this include for UART_NUM_0

#include "ahrs.h"
#include "bmx160.h"  // Changed from mpu9250.h
// #include "calibrate.h"  // BMX160 has its own calibration functions
// #include "common.h"     // Not needed for BMX160
#include <math.h>  // Needed for M_PI
#include "mics5524.h"
#include "bmx280.h"
#include "communications.h" // Import FlightState and SensorData from here
#include "gps.h" // Include the header for gps_task
#include "autonomous_paraglider.h"
#include "esp_random.h" // Include for esp_random()

#define DEG2RAD(x) ((x) * M_PI / 180.0)

// Using FlightState from communications.h
FlightState currentState = PRE_LAUNCH;

// Using SensorData from communications.h
SensorData sensorData;

static float previousAltitude = 0;


// --------------------------------------------> IMU PROCESS STARTS HERE <--------------------------------------------------------

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define SAMPLE_FREQ_Hz 100.0
static const char *TAGM = "IMU_TASK";

// pre-calibrated values that will be fed to BMX160
bmx160_calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};

/**
 * Transformation functions for BMX160 orientation
 * (you may need to adjust these based on your PCB layout)
 */
static void transform_accel_gyro(bmx160_vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -x;
    v->y = -z;
    v->z = -y;
}

static void transform_mag(bmx160_vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -y;
    v->y = z;
    v->z = -x;
}

//main BMX160 process func
void run_imu(void)
{
    ESP_LOGI(TAGM, "Initializing BMX160...");
    ESP_ERROR_CHECK(i2c_bmx160_init(&cal));
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);

    uint64_t i = 0;

    while (true)
    {
        bmx160_vector_t va, vg, vm;

        // Get the Accelerometer, Gyroscope and Magnetometer values from BMX160
        // Updated to use bmx160_get_accel_gyro_mag instead of get_accel_gyro_mag
        if (bmx160_get_accel_gyro_mag(&va, &vg, &vm) != ESP_OK)
        {
            ESP_LOGE(TAGM, "Failed to read BMX160 data!");
            continue;
        }

        // Transform these values to the orientation of your device
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm
        ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                    va.x, va.y, va.z,
                    vm.x, vm.y, vm.z);

        // Store data in struct
        sensorData.ax = va.x;
        sensorData.ay = va.y;
        sensorData.az = va.z;
        sensorData.gx = vg.x;
        sensorData.gy = vg.y;
        sensorData.gz = vg.z;
        sensorData.mx = vm.x;
        sensorData.my = vm.y;
        sensorData.mz = vm.z;

        // Get Euler angles from AHRS
        ahrs_get_euler_in_degrees(&sensorData.yaw, &sensorData.pitch, &sensorData.roll);

        // Get temperature from BMX160
        // Updated to use bmx160_get_temperature_celsius instead of get_temperature_celsius
        if (bmx160_get_temperature_celsius(&sensorData.temperature) != ESP_OK)
        {
            sensorData.temperature = -999;  // Error flag
        }

        // Print the data out every 10 iterations
        if (i++ % 10 == 0)
        {
            ESP_LOGI(TAGM, "Heading: %2.3f°, Pitch: %2.3f°, Roll: %2.3f°, Temp: %2.3f°C",
                     sensorData.yaw, sensorData.pitch, sensorData.roll, sensorData.temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling 
    }
}

// all the BMX160 (IMU) process goes here
static void imu_task(void *arg) {
#ifdef CONFIG_CALIBRATION_MODE
    ESP_LOGI(TAGM, "Entering BMX160 calibration mode...");
    bmx160_calibrate_gyro();
    bmx160_calibrate_accel();
    bmx160_calibrate_mag();
#else
    run_imu();
#endif

    // Cleanup resources before deleting the task
    ESP_LOGI(TAGM, "BMX160 Task exited.");
    vTaskDelete(NULL);
}

// ----------------------------------------> BME PROCESS STARTS HERE <-------------------------------------



#define BMX280_SDA_NUM GPIO_NUM_8
#define BMX280_SCL_NUM GPIO_NUM_9
#define BMX280_I2C_PORT I2C_NUM_0

static const char* TAGB = "BME280";
// all the baro (bme280) process goes here
void barometer_task(void *arg) {
    // Configure I2C
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BMX280_SDA_NUM,
        .scl_io_num = BMX280_SCL_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000  // 400kHz I2C clock speed
    };

    // Initialize I2C driver
    ESP_ERROR_CHECK(i2c_param_config(BMX280_I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(BMX280_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Initialize BME280
    bmx280_t* bmx280 = bmx280_create(BMX280_I2C_PORT);
    if (!bmx280) {
        ESP_LOGE(TAGB, "Could not create bmx280 driver.");
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));

    // Configure the BME280
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));

    while (1) {
        // Wait until the sensor is done sampling
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (bmx280_isSampling(bmx280));

        // Read sensor data
        float temp = 0, pres = 0, hum = 0;
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

        // Store in struct
        sensorData.temperature = temp;
        sensorData.pressure = pres;
        sensorData.humidity = hum;

        ESP_LOGI(TAGB, "Temperature: %.2f°C, Pressure: %.2f hPa, Humidity: %.2f%%", temp, pres, hum);

        // Delay before next reading
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}
static const char *TAG_GPS = "GPS";
static const char *TAG_VOLTAGE = "VOLT";
static const char *TAG_RW = "RW";
// all the gps process goes here


void voltage_task(void *pvParameters) {
    while (1) {
        float voltage = 7.4 + ((esp_random() % 100) / 100.0);  // 7.4V to 8.4V

        ESP_LOGI(TAG_VOLTAGE, "Voltage: %.2f V", voltage);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void reaction_wheel_task(void *pvParameters) {
    while (1) {
        int rpm1 = 160;
        int rpm2 = 148;

        ESP_LOGI(TAG_RW, "RW1: %d RPM | RW2: %d RPM", rpm1, rpm2);
        vTaskDelay(500 / portTICK_PERIOD_MS);

    }
}


// ----------------------------------------> MICS PROCESS STARTS HERE <-------------------------------------

static const char *TAGG = "GAS_TASK";

void gas_sensor_task(void *pvParameters) {
    mics5524_init();

    while (1) {
        float gas_value = mics5524_read();

        // Store gas value in shared sensorData struct
        sensorData.gas_level = gas_value;
        ESP_LOGI(TAGG, "Value: %.2f", gas_value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1Hz
    }
}

// This is logic for flight state change
void update_flight_state(float acceleration, float altitude, float velocity) {
    switch (currentState) {
        case PRE_LAUNCH:
            if (acceleration > 15) { // When above 15G is detected (this is a guesstimation)
                currentState = ASCENT;
            }
            break;

        case ASCENT:
            if (velocity <= 0) { // Gotta trial and error this one
                currentState = APOGEE;
            }
            break;

        case APOGEE:
            if (altitude - previousAltitude > 10) { // altitude drop (guesstimation)
                currentState = DESCENT;
            }
            break;

        case DESCENT:
            if (velocity < 1 && altitude < 5) { // Near ground (guesstimation)
                currentState = LANDING;
            }
            break;

        case LANDING:
        
            break;
    }
}

 // this will manage flight states
/*void state_management_task(void *pvParameters) {
    while (1) {
        float accel = sensorData.ac;
        float altitude = get_altitude();
        float velocity = get_velocity();
        
        update_flight_state(accel, altitude, velocity);

        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}
*/
/////FAKE

// ----------------------------------------> ENHANCED DIFFERENTIAL COMMS TASK <-------------------------------------

static const char *TAG_COMMS = "COMMS_TASK";

void comms_task(void *pvParameters) {
    // Initialize both packet systems
    packet_system_init();
    differential_packet_init();
    
    uint32_t heartbeat_counter = 0;
    uint32_t stats_counter = 0;
    packet_context_t packet_stats;
    uint32_t diff_total, diff_partial, diff_full, diff_bytes_saved;

    ESP_LOGI(TAG_COMMS, "Enhanced differential communication system started");

    while (1) {
        // Send differential telemetry burst with real sensor data
        esp_err_t ret = send_differential_telemetry_burst(&sensorData);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_COMMS, "Failed to send differential telemetry: %s", esp_err_to_name(ret));
        }

        // Send heartbeat every 10 seconds (50 cycles at 200ms each)
        if (++heartbeat_counter >= 50) {
            ret = send_heartbeat_packet();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_COMMS, "Failed to send heartbeat: %s", esp_err_to_name(ret));
            }
            heartbeat_counter = 0;
        }

        // Log statistics every 30 seconds (150 cycles at 200ms each)
        if (++stats_counter >= 150) {
            // Log basic packet statistics
            if (get_packet_stats(&packet_stats) == ESP_OK) {
                ESP_LOGI(TAG_COMMS, "Basic Stats - Sent: %lu, Received: %lu, CRC Errors: %lu, Frame Errors: %lu",
                         packet_stats.packets_sent, packet_stats.packets_received, 
                         packet_stats.crc_errors, packet_stats.frame_errors);
            }
            
            // Log differential packetization statistics
            if (get_differential_stats(&diff_total, &diff_partial, &diff_full, &diff_bytes_saved) == ESP_OK) {
                ESP_LOGI(TAG_COMMS, "Differential Stats - Total: %lu, Partial: %lu, Full: %lu, Bytes Saved: ~%lu",
                         diff_total, diff_partial, diff_full, diff_bytes_saved);
                
                if (diff_total > 0) {
                    float efficiency = ((float)diff_partial / diff_total) * 100.0f;
                    ESP_LOGI(TAG_COMMS, "Efficiency: %.1f%% packets were differential (smaller)", efficiency);
                }
            }
            
            stats_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz telemetry rate
    }
}

// ----------------------------------------> PARAGLIDER NAVIGATION TASK <-------------------------------------
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
        .arrival_radius_m = 30.0f, //what is 30.0f here
        .name = "Waypoint 1"
    };
    paraglider_add_waypoint(&waypoint1);

    // Configure loitering parameters (optional - defaults will be used if not set)
    paraglider_set_loiter_params(target_lat, target_lon, 150.0f, LOITER_DIRECTION_CW); //what is 150.0f here

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
/*
void app_main() {
    gps_init(); // Initialize the GPS module once during setup
    xTaskCreatePinnedToCore(barometer_task, "Barometer Task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "IMU Task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(gas_sensor_task, "Gas Sensor Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(voltage_task, "Voltage Task", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(reaction_wheel_task, "RW Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(comms_task, "Comms Task", 4096, NULL, 4, NULL, 1);
    
    // Add the paraglider navigation task
    xTaskCreatePinnedToCore(paraglider_navigation_task, "Paraglider Nav", 4096, NULL, 4, NULL, 0);
}
    */

// 100hz --> + 25% targeted freq
//yield strength, tensile strength, safety factor.