#include "bmx160.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BMX160";

// Global variables
static i2c_port_t g_i2c_port = I2C_NUM_0;
static const bmx160_calibration_t *g_calibration = NULL;
static bool g_initialized = false;

// Accelerometer and gyroscope scaling factors
static float g_accel_scale = 1.0f;
static float g_gyro_scale = 1.0f;

// I2C helper functions
static esp_err_t bmx160_write_reg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(g_i2c_port, BMX160_I2C_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

static esp_err_t bmx160_read_reg(uint8_t reg, uint8_t *data) {
    return i2c_master_write_read_device(g_i2c_port, BMX160_I2C_ADDR, &reg, 1, data, 1, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t bmx160_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(g_i2c_port, BMX160_I2C_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Calculate scaling factors based on range settings
static void bmx160_update_scales(uint8_t acc_range, uint8_t gyro_range) {
    // Accelerometer scaling (LSB/g)
    switch (acc_range) {
        case BMX160_ACC_RANGE_2G:   g_accel_scale = 16384.0f; break;
        case BMX160_ACC_RANGE_4G:   g_accel_scale = 8192.0f; break;
        case BMX160_ACC_RANGE_8G:   g_accel_scale = 4096.0f; break;
        case BMX160_ACC_RANGE_16G:  g_accel_scale = 2048.0f; break;
        default:                    g_accel_scale = 16384.0f; break;
    }
    
    // Gyroscope scaling (LSB/dps)
    switch (gyro_range) {
        case BMX160_GYR_RANGE_125DPS:  g_gyro_scale = 262.4f; break;
        case BMX160_GYR_RANGE_250DPS:  g_gyro_scale = 131.2f; break;
        case BMX160_GYR_RANGE_500DPS:  g_gyro_scale = 65.6f; break;
        case BMX160_GYR_RANGE_1000DPS: g_gyro_scale = 32.8f; break;
        case BMX160_GYR_RANGE_2000DPS: g_gyro_scale = 16.4f; break;
        default:                       g_gyro_scale = 131.2f; break;
    }
}

esp_err_t bmx160_soft_reset(void) {
    esp_err_t ret = bmx160_write_reg(BMX160_REG_CMD, BMX160_CMD_SOFTRESET);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete
    }
    return ret;
}

esp_err_t bmx160_read_chip_id(uint8_t *chip_id) {
    return bmx160_read_reg(BMX160_REG_CHIP_ID, chip_id);
}

esp_err_t bmx160_set_power_mode(uint8_t acc_mode, uint8_t gyro_mode, uint8_t mag_mode) {
    esp_err_t ret;
    
    // Set accelerometer power mode
    ret = bmx160_write_reg(BMX160_REG_CMD, BMX160_CMD_ACC_SET_PMU_MODE | acc_mode);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set gyroscope power mode
    ret = bmx160_write_reg(BMX160_REG_CMD, BMX160_CMD_GYR_SET_PMU_MODE | gyro_mode);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50)); // Gyro needs more time
    
    // Set magnetometer power mode
    ret = bmx160_write_reg(BMX160_REG_CMD, BMX160_CMD_MAG_SET_PMU_MODE | mag_mode);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    return ESP_OK;
}

esp_err_t bmx160_configure(const bmx160_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret;
    
    // Configure accelerometer
    uint8_t acc_conf = (config->acc_odr & 0x0F);
    ret = bmx160_write_reg(BMX160_REG_ACC_CONF, acc_conf);
    if (ret != ESP_OK) return ret;
    
    ret = bmx160_write_reg(BMX160_REG_ACC_RANGE, config->acc_range);
    if (ret != ESP_OK) return ret;
    
    // Configure gyroscope
    uint8_t gyr_conf = (config->gyro_odr & 0x0F);
    ret = bmx160_write_reg(BMX160_REG_GYR_CONF, gyr_conf);
    if (ret != ESP_OK) return ret;
    
    ret = bmx160_write_reg(BMX160_REG_GYR_RANGE, config->gyro_range);
    if (ret != ESP_OK) return ret;
    
    // Configure magnetometer
    uint8_t mag_conf = (config->mag_odr & 0x0F);
    ret = bmx160_write_reg(BMX160_REG_MAG_CONF, mag_conf);
    if (ret != ESP_OK) return ret;
    
    // Update scaling factors
    bmx160_update_scales(config->acc_range, config->gyro_range);
    
    return ESP_OK;
}

esp_err_t bmx160_init(i2c_port_t i2c_port, const bmx160_calibration_t *cal) {
    g_i2c_port = i2c_port;
    g_calibration = cal;
    
    esp_err_t ret;
    uint8_t chip_id;
    
    // Soft reset the sensor
    ret = bmx160_soft_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset BMX160");
        return ret;
    }
    
    // Read and verify chip ID
    ret = bmx160_read_chip_id(&chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMX160 chip ID");
        return ret;
    }
    
    if (chip_id != BMX160_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X, expected 0x%02X", chip_id, BMX160_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "BMX160 chip ID verified: 0x%02X", chip_id);
    
    // Configure default settings
    bmx160_config_t default_config = {
        .acc_range = BMX160_ACC_RANGE_4G,
        .acc_odr = BMX160_ACC_ODR_100HZ,
        .gyro_range = BMX160_GYR_RANGE_250DPS,
        .gyro_odr = BMX160_GYR_ODR_100HZ,
        .mag_odr = BMX160_ACC_ODR_25HZ
    };
    
    ret = bmx160_configure(&default_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BMX160");
        return ret;
    }
    
    // Set power modes to normal
    ret = bmx160_set_power_mode(BMX160_PMU_NORMAL, BMX160_PMU_NORMAL, BMX160_PMU_NORMAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power modes");
        return ret;
    }
    
    g_initialized = true;
    ESP_LOGI(TAG, "BMX160 initialized successfully");
    
    return ESP_OK;
}

esp_err_t bmx160_read_accel_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[6];
    esp_err_t ret = bmx160_read_regs(BMX160_REG_DATA_8, data, 6); // Accel data starts at DATA_8
    if (ret != ESP_OK) return ret;
    
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    
    return ESP_OK;
}

esp_err_t bmx160_read_gyro_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[6];
    esp_err_t ret = bmx160_read_regs(BMX160_REG_DATA_14, data, 6); // Gyro data starts at DATA_14
    if (ret != ESP_OK) return ret;
    
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    
    return ESP_OK;
}

esp_err_t bmx160_read_mag_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[8];
    esp_err_t ret = bmx160_read_regs(BMX160_REG_DATA_0, data, 8); // Mag data starts at DATA_0
    if (ret != ESP_OK) return ret;
    
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    
    return ESP_OK;
}

static float apply_accel_calibration(float value, float offset, float scale_lo, float scale_hi) {
    if (!g_calibration) return value;
    
    if (value < 0) {
        return -(value - offset) / (scale_lo - offset);
    } else {
        return (value - offset) / (scale_hi - offset);
    }
}

esp_err_t bmx160_get_accel(bmx160_vector_t *accel) {
    if (!accel) return ESP_ERR_INVALID_ARG;
    
    int16_t raw_x, raw_y, raw_z;
    esp_err_t ret = bmx160_read_accel_raw(&raw_x, &raw_y, &raw_z);
    if (ret != ESP_OK) return ret;
    
    // Convert to g values
    float x = (float)raw_x / g_accel_scale;
    float y = (float)raw_y / g_accel_scale;
    float z = (float)raw_z / g_accel_scale;
    
    // Apply calibration if available
    if (g_calibration) {
        accel->x = apply_accel_calibration(x, g_calibration->accel_offset.x, 
                                          g_calibration->accel_scale_lo.x, 
                                          g_calibration->accel_scale_hi.x);
        accel->y = apply_accel_calibration(y, g_calibration->accel_offset.y, 
                                          g_calibration->accel_scale_lo.y, 
                                          g_calibration->accel_scale_hi.y);
        accel->z = apply_accel_calibration(z, g_calibration->accel_offset.z, 
                                          g_calibration->accel_scale_lo.z, 
                                          g_calibration->accel_scale_hi.z);
    } else {
        accel->x = x;
        accel->y = y;
        accel->z = z;
    }
    
    return ESP_OK;
}

esp_err_t bmx160_get_gyro(bmx160_vector_t *gyro) {
    if (!gyro) return ESP_ERR_INVALID_ARG;
    
    int16_t raw_x, raw_y, raw_z;
    esp_err_t ret = bmx160_read_gyro_raw(&raw_x, &raw_y, &raw_z);
    if (ret != ESP_OK) return ret;
    
    // Convert to degrees per second and apply bias correction
    gyro->x = (float)raw_x / g_gyro_scale;
    gyro->y = (float)raw_y / g_gyro_scale;
    gyro->z = (float)raw_z / g_gyro_scale;
    
    if (g_calibration) {
        gyro->x += g_calibration->gyro_bias_offset.x;
        gyro->y += g_calibration->gyro_bias_offset.y;
        gyro->z += g_calibration->gyro_bias_offset.z;
    }
    
    return ESP_OK;
}

esp_err_t bmx160_get_mag(bmx160_vector_t *mag) {
    if (!mag) return ESP_ERR_INVALID_ARG;
    
    int16_t raw_x, raw_y, raw_z;
    esp_err_t ret = bmx160_read_mag_raw(&raw_x, &raw_y, &raw_z);
    if (ret != ESP_OK) return ret;
    
    // Convert to µT (microtesla) - BMX160 magnetometer has fixed scale of 0.3µT/LSB
    float x = (float)raw_x * 0.3f;
    float y = (float)raw_y * 0.3f;
    float z = (float)raw_z * 0.3f;
    
    // Apply calibration if available
    if (g_calibration) {
        mag->x = (x - g_calibration->mag_offset.x) * g_calibration->mag_scale.x;
        mag->y = (y - g_calibration->mag_offset.y) * g_calibration->mag_scale.y;
        mag->z = (z - g_calibration->mag_offset.z) * g_calibration->mag_scale.z;
    } else {
        mag->x = x;
        mag->y = y;
        mag->z = z;
    }
    
    return ESP_OK;
}

esp_err_t bmx160_get_accel_gyro_mag(bmx160_vector_t *accel, bmx160_vector_t *gyro, bmx160_vector_t *mag) {
    esp_err_t ret;
    
    ret = bmx160_get_accel(accel);
    if (ret != ESP_OK) return ret;
    
    ret = bmx160_get_gyro(gyro);
    if (ret != ESP_OK) return ret;
    
    ret = bmx160_get_mag(mag);
    return ret;
}

esp_err_t bmx160_get_temperature_celsius(float *temperature) {
    if (!temperature || !g_initialized) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    esp_err_t ret = bmx160_read_regs(BMX160_REG_TEMPERATURE_0, data, 2);
    if (ret != ESP_OK) return ret;
    
    int16_t temp_raw = (int16_t)((data[1] << 8) | data[0]);
    
    // BMX160 temperature conversion: 23°C + (temp_raw / 512)
    *temperature = 23.0f + ((float)temp_raw / 512.0f);
    
    return ESP_OK;
}

// Compatibility functions with existing MPU9250 interface
esp_err_t i2c_bmx160_init(const bmx160_calibration_t *cal) {
    return bmx160_init(I2C_NUM_0, cal);
}

esp_err_t get_accel_gyro_mag(bmx160_vector_t *va, bmx160_vector_t *vg, bmx160_vector_t *vm) {
    return bmx160_get_accel_gyro_mag(va, vg, vm);
}

esp_err_t get_accel(bmx160_vector_t *v) {
    return bmx160_get_accel(v);
}

esp_err_t get_gyro(bmx160_vector_t *v) {
    return bmx160_get_gyro(v);
}

esp_err_t get_mag(bmx160_vector_t *v) {
    return bmx160_get_mag(v);
}

esp_err_t get_temperature_celsius(float *val) {
    return bmx160_get_temperature_celsius(val);
}

// Calibration functions (simplified versions)
void bmx160_calibrate_gyro(void) {
    ESP_LOGI(TAG, "BMX160 Gyroscope calibration");
    ESP_LOGW(TAG, "Keep the sensor very still during calibration...");
    
    bmx160_vector_t gyro_sum = {0, 0, 0};
    const int num_samples = 1000;
    
    for (int i = 0; i < num_samples; i++) {
        bmx160_vector_t gyro;
        if (bmx160_get_gyro(&gyro) == ESP_OK) {
            gyro_sum.x += gyro.x;
            gyro_sum.y += gyro.y;
            gyro_sum.z += gyro.z;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Calculate bias (negative values to subtract from readings)
    ESP_LOGI(TAG, "Gyro bias: X=%.3f, Y=%.3f, Z=%.3f", 
             -gyro_sum.x / num_samples, 
             -gyro_sum.y / num_samples, 
             -gyro_sum.z / num_samples);
}

void bmx160_calibrate_accel(void) {
    ESP_LOGI(TAG, "BMX160 Accelerometer calibration");
    ESP_LOGW(TAG, "Rotate the sensor through all orientations...");
    // Simplified version - full implementation would need user interaction
}

void bmx160_calibrate_mag(void) {
    ESP_LOGI(TAG, "BMX160 Magnetometer calibration");
    ESP_LOGW(TAG, "Rotate the sensor around all axes...");
    // Simplified version - full implementation would track min/max values
}