/*#include "mics5524.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main() {
    printf("Initializing MICS5524...\n");
    mics5524_init();

    while (1) {
        uint32_t gas_value = mics5524_read();
        printf("Gas Sensor Value: %lu\n", (unsigned long)gas_value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1-second delay
    }
}
*/