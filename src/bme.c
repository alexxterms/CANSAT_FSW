#include "esp_log.h"
#include "bmx280.h"
#include "driver/i2c.h"

#define BMX280_SDA_NUM GPIO_NUM_13
#define BMX280_SCL_NUM GPIO_NUM_14
#define BMX280_I2C_PORT I2C_NUM_0

static const char* TAG = "BME280";

/*void app_main(void)
{
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
        ESP_LOGE(TAG, "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));

    // Configure the BME280
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));

    float temp = 0, pres = 0, hum = 0;

    while (1) {
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (bmx280_isSampling(bmx280));

        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
        ESP_LOGI(TAG, "Temperature: %.2f°C, Pressure: %.2f hPa, Humidity: %.2f%%", temp, pres, hum);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }
}
*/