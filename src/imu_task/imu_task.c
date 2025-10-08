#include "imu_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmx160.h"
#include "ahrs.h"
#include "communications.h" // For accessing shared SensorData

extern SensorData sensorData; // Shared sensor data structure

#define SAMPLE_FREQ_Hz 100.0
static const char *TAGM = "IMU_TASK";

// Pre-calibrated values for BMX160
static bmx160_calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};

// Transformation functions for BMX160 orientation
void transform_accel_gyro(bmx160_vector_t *v) {
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -x;
    v->y = -z;
    v->z = -y;
}

void transform_mag(bmx160_vector_t *v) {
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -y;
    v->y = z;
    v->z = -x;
}

// Main BMX160 process function
void run_imu(void) {
    ESP_LOGI(TAGM, "Initializing BMX160...");
    ESP_ERROR_CHECK(i2c_bmx160_init(&cal));
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);

    uint64_t i = 0;

    while (true) {
        bmx160_vector_t va, vg, vm;

        // Get the Accelerometer, Gyroscope, and Magnetometer values from BMX160
        if (bmx160_get_accel_gyro_mag(&va, &vg, &vm) != ESP_OK) {
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

        // Store data in the shared SensorData struct
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
        if (bmx160_get_temperature_celsius(&sensorData.temperature) != ESP_OK) {
            sensorData.temperature = -999;  // Error flag
        }

        // Print the data out every 10 iterations
        if (i++ % 10 == 0) {
            ESP_LOGI(TAGM, "Heading: %2.3f°, Pitch: %2.3f°, Roll: %2.3f°, Temp: %2.3f°C",
                     sensorData.yaw, sensorData.pitch, sensorData.roll, sensorData.temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling
    }
}

// IMU task entry point
void imu_task(void *arg) {
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