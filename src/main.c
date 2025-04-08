
/*Things to do : Complete GPS TASK --> Test
                 Complete MICS Task --> Test
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

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
#include <math.h>  // Needed for M_PI

#include "bmx280.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)



// This enum stores flight states
typedef enum {
    PRE_LAUNCH,
    ASCENT,
    APOGEE,
    DESCENT,
    LANDING
} FlightState;

FlightState currentState = PRE_LAUNCH;

// sensor data struct for other tasks to munch on
typedef struct {
    float roll, pitch, yaw;  
    float ax, ay, az;        
    float gx, gy, gz;       
    float mx, my, mz;       
    float altitude, pressure;
    float voltage;
    float latitude, longitude;
    float gas_level;
    float temperature;
    float humidity;
} SensorData;

SensorData sensorData;


// ########################################### MPU PROCESS STARTS HERE ###################################################

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define SAMPLE_FREQ_Hz 100.0
static const char *TAGM = "IMU_TASK";

// pre-calibrated values that will be fed in mpu
calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 */
static void transform_accel_gyro(vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -x;
    v->y = -z;
    v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 */
static void transform_mag(vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -y;
    v->y = z;
    v->z = -x;
}

//main mpu process func
void run_imu(void)
{
    ESP_LOGI(TAGM, "Initializing IMU...");
    ESP_ERROR_CHECK(i2c_mpu9250_init(&cal));
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);

    uint64_t i = 0;

    while (true)
    {
        vector_t va, vg, vm;

        // Get the Accelerometer, Gyroscope and Magnetometer values.
        if (get_accel_gyro_mag(&va, &vg, &vm) != ESP_OK)
        {
            ESP_LOGE(TAGM, "Failed to read IMU data!");
            continue;
        }

        // Transform these values to the orientation of our device.
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm, can we use kalman?????? cpu cost where???
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

        // this gets temp, its not that good but lets see, probably will use bme temp
        if (get_temperature_celsius(&sensorData.temperature) != ESP_OK)
        {
            sensorData.temperature = -999;  // Error flag
        }

        // Print the data out every 10 iterations
        if (i++ % 10 == 0)
        {
            ESP_LOGI(TAGM, "Heading: %2.3f°, Pitch: %2.3f°, Roll: %2.3f°, Temp: %2.3f°C",
                     sensorData.yaw, sensorData.pitch, sensorData.roll, sensorData.temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz samplingg 
    }
}

// all the imu (mpu9250) process goes here
static void imu_task(void *arg) {
#ifdef CONFIG_CALIBRATION_MODE
    ESP_LOGI(TAGB, "Entering calibration mode...");
    calibrate_gyro();
    calibrate_accel();
    calibrate_mag();
#else
    run_imu();
#endif

    // Cleanup resources before deleting the task
    i2c_driver_delete(I2C_MASTER_NUM);
    ESP_LOGI(TAGB, "IMU Task exited.");
    vTaskDelete(NULL);
}

// ################################################ BME PROCESS STARTS HERE ######################################################



#define BMX280_SDA_NUM GPIO_NUM_13
#define BMX280_SCL_NUM GPIO_NUM_14
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

// all the gps process goes here
void gps_task(void *pvParameters) {
    while (1) {
        sensorData.latitude = get_latitude();
        sensorData.longitude = get_longitude();

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1Hz
    }
}

// all the mics5524 process goes here
void gas_sensor_task(void *pvParameters) {
    while (1) {
        sensorData.gas_level = get_gas_level();

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1Hz
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
void state_management_task(void *pvParameters) {
    while (1) {
        float accel = get_acceleration();
        float altitude = get_altitude();
        float velocity = get_velocity();
        
        update_flight_state(accel, altitude, velocity);

        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}


void app_main() {
    
    imu_init();
    barometer_init();
    gps_init();
    sd_card_init();
    lora_init();

    // tasks are created here 
    xTaskCreatePinnedToCore(imu_task, "IMU Task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(barometer_task, "Barometer Task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&gps_task, "GPS Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(&battery_task, "Battery Task", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(&gas_sensor_task, "Gas Sensor Task", 2048, NULL, 2, NULL, 1);

    xTaskCreatePinnedToCore(&state_management_task, "State Management", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&sd_logging_task, "SD Card Logging", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(&lora_communication_task, "LoRa Task", 4096, NULL, 3, NULL, 0);
}    
