#include "bme280_driver.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 34
#define I2C_MASTER_FREQ_HZ 100000
#define BME280_ADDR 0x76

static const char *TAG = "BME280";
esp_err_t i2c_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, dev_addr, &reg_addr, 1, data, len, 100 / portTICK_PERIOD_MS);
}

void bme280_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "BME280 initialized");

     // Check if the device is found
     if (bme280_check_device()) {
        ESP_LOGI(TAG, "BME280 device found!");
    } else {
        ESP_LOGE(TAG, "BME280 NOT detected! Check wiring.");
    }
    uint8_t ctrl_meas = 0b00100111; // osrs_t=1, osrs_p=1, mode=normal (0b11)
    bme280_read(0xF4, &ctrl_meas, 1);

}

esp_err_t bme280_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bme280_read_sensor_data() {
    uint8_t status;
    
    do {
        bme280_read(0xF3, &status, 1);
        if (status & 0x08) {  // Bit 3: Measuring flag
            ESP_LOGW("BME280", "Sensor is still measuring, waiting...");
            vTaskDelay(pdMS_TO_TICKS(10));  // Wait a bit and retry
        }
    } while (status & 0x08);
    uint8_t temp_msb, temp_lsb, temp_xlsb;
    bme280_read(0xFA, &temp_msb, 1);
    bme280_read(0xFB, &temp_lsb, 1);
    bme280_read(0xFC, &temp_xlsb, 1);
    

uint32_t raw_temp = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | ((uint32_t)temp_xlsb >> 4);
ESP_LOGI("BME280", "Raw Temperature Registers: 0xFA=%02X  0xFB=%02X  0xFC=%02X", temp_msb, temp_lsb, temp_xlsb);
ESP_LOGI("BME280", "Raw Temperature Value: %lu", (unsigned long)raw_temp);


}

bool bme280_check_device() {
    uint8_t chip_id = 0;
    esp_err_t err;
    
    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xD0, true);  // Register to read the chip ID
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &chip_id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK && chip_id == 0x60) {
        ESP_LOGI(TAG, "BME280 found! Chip ID: 0x%02X", chip_id);
        return true;
    } else {
        ESP_LOGE(TAG, "BME280 not detected! Chip ID: 0x%02X", chip_id);
        return false;
    }
}