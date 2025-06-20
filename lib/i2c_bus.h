// i2c_bus.h
#pragma once
#include "driver/i2c_master.h"

esp_err_t shared_i2c_bus_init(void);
i2c_master_bus_handle_t get_i2c_bus_handle(void);
