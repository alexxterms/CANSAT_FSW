#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

#include "esp_err.h"
#include <stdbool.h>  

void bme280_init();
esp_err_t bme280_read(uint8_t reg, uint8_t *data, size_t len);
void bme280_read_sensor_data();
bool bme280_check_device();
void i2c_master_read_register();

#endif
