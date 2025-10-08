#include "DShotWrapper.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

void motor_control_task(void *pvParameters) {
    // Initialize the watchdog timer for this task
    esp_err_t err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        printf("Failed to add task to watchdog timer: %d\n", err);
    }
    
    printf("Creating DShot handle...\n");
    // Create a DShot instance
    dshot_handle_t motor = dshot_create();
    if (!motor) {
        printf("Failed to create DShot handle\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("Initializing DShot on pin 21...\n");
    // Initialize the DShot on pin 21 with DSHOT600 protocol
    dshot_begin_with_pin(motor, 21, DSHOT600_C, NO_BIDIRECTION_C, 14);
    
    // Send 0 throttle value for 1 second to initialize the ESC
    printf("Initializing ESC with 0 throttle...\n");
    for (int i = 0; i < 3000 / 10; i++) { // 1000ms / 10ms = 100 iterations
        if (motor) {
            dshot_send_value(motor, 0, NO_TELEMETRIC_C); // Send 0 throttle
        } else {
            printf("Motor handle is NULL!\n");
            break;
        }
        
        // Reset the watchdog timer
        err = esp_task_wdt_reset();
        if (err != ESP_OK) {
            printf("Failed to reset watchdog timer: %d\n", err);
        }
        
        // Use a longer delay to reduce stress on the system
        vTaskDelay(pdMS_TO_TICKS(6));
    }
    
    printf("ESC initialization complete. Starting motor control...\n");
    
    // Continuously send throttle value at 10ms intervals
    uint16_t throttle_value = 500;
    uint32_t counter = 0;
    
    while (1) {
        // Reset the watchdog timer on each iteration
        err = esp_task_wdt_reset();
        if (err != ESP_OK && counter % 100 == 0) {
            printf("Failed to reset watchdog timer: %d\n", err);
        }
        
        // Send the throttle value
        if (motor) {
            dshot_send_packet_exit_mode_c_t result = dshot_send_value(motor, throttle_value, NO_TELEMETRIC_C);
            if (result != SEND_SUCCESS_C && counter % 100 == 0) {
                printf("Failed to send DShot value\n");
            }
        } else {
            printf("Motor handle is NULL!\n");
            break;
        }
        
        // Print status every ~1 second
        counter++;
        if (counter % 100 == 0) {  // About once per second (100 * 10ms = 1000ms)
            printf("Running at throttle: %u\n", throttle_value);
        }
        
        // Use a longer delay to reduce stress on the system
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // This part won't be reached due to the infinite loop,
    // but it's good practice to include it
    if (motor) {
        dshot_destroy(motor);
    }
    
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}
/*
// Main function for ESP-IDF
void app_main(void) {
    printf("Running DShot motor control example\n");
    
    // Configure the watchdog timer with a timeout of 5 seconds
    // and set panic=false to avoid system restart on timeout
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = false,
    };
    
    
    
    // Create the motor control task with increased stack size and priority
    xTaskCreate(motor_control_task, "motor_task", 8192, NULL, 5, NULL);
}

*/