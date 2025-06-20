// i2c_bus.c
#include "i2c_bus.h"

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t shared_i2c_bus_init(void) {
    if (bus_handle != NULL) return ESP_OK;

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 8,
        .sda_io_num = 10,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_config, &bus_handle);
}

i2c_master_bus_handle_t get_i2c_bus_handle(void) {
    return bus_handle;
}
