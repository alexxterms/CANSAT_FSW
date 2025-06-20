/*#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"

#define GPS_UART_PORT   UART_NUM_1  // Choose UART1 or any available UART port
#define TXD_PIN         (17)        // ESP32 TX -> GPS RX
#define RXD_PIN         (16)        // ESP32 RX <- GPS TX
#define BAUD_RATE       9600        // Default baud rate for GPS
#define BUF_SIZE        1024        // Buffer size for UART

static const char *TAG = "GPS";

void gps_task(void *arg) {
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate the received data
            ESP_LOGI(TAG, "GPS Data: %s", data);
        }
    }
}
*/
/*void app_main(void) {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install and configure UART
    uart_driver_install(GPS_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Start GPS read task
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
*/