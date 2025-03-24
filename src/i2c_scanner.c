#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define TAG "I2C_SCANNER"

#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define I2C_MASTER_SDA 21         // SDA pin
#define I2C_MASTER_SCL 22         // SCL pin
#define I2C_MASTER_FREQ_HZ 100000 // I2C frequency

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void i2c_scan() {
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", address);
        }
    }
    ESP_LOGI(TAG, "I2C scan complete.");
}
