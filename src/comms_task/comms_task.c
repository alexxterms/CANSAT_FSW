#include "comms_task.h"
#include "communications.h" // For packet system and telemetry functions
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern SensorData sensorData; // Shared sensor data structure

static const char *TAG_COMMS = "COMMS_TASK";

void comms_task(void *pvParameters) {
    // Initialize both packet systems
    packet_system_init();
    differential_packet_init();
    
    uint32_t heartbeat_counter = 0;
    uint32_t stats_counter = 0;
    packet_context_t packet_stats;
    uint32_t diff_total, diff_partial, diff_full, diff_bytes_saved;

    ESP_LOGI(TAG_COMMS, "Enhanced differential communication system started");

    while (1) {
        // Send differential telemetry burst with real sensor data
        esp_err_t ret = send_differential_telemetry_burst(&sensorData);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_COMMS, "Failed to send differential telemetry: %s", esp_err_to_name(ret));
        }

        // Send heartbeat every 10 seconds (50 cycles at 200ms each)
        if (++heartbeat_counter >= 50) {
            ret = send_heartbeat_packet();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_COMMS, "Failed to send heartbeat: %s", esp_err_to_name(ret));
            }
            heartbeat_counter = 0;
        }

        // Log statistics every 30 seconds (150 cycles at 200ms each)
        if (++stats_counter >= 150) {
            // Log basic packet statistics
            if (get_packet_stats(&packet_stats) == ESP_OK) {
                ESP_LOGI(TAG_COMMS, "Basic Stats - Sent: %lu, Received: %lu, CRC Errors: %lu, Frame Errors: %lu",
                         packet_stats.packets_sent, packet_stats.packets_received, 
                         packet_stats.crc_errors, packet_stats.frame_errors);
            }
            
            // Log differential packetization statistics
            if (get_differential_stats(&diff_total, &diff_partial, &diff_full, &diff_bytes_saved) == ESP_OK) {
                ESP_LOGI(TAG_COMMS, "Differential Stats - Total: %lu, Partial: %lu, Full: %lu, Bytes Saved: ~%lu",
                         diff_total, diff_partial, diff_full, diff_bytes_saved);
                
                if (diff_total > 0) {
                    float efficiency = ((float)diff_partial / diff_total) * 100.0f;
                    ESP_LOGI(TAG_COMMS, "Efficiency: %.1f%% packets were differential (smaller)", efficiency);
                }
            }
            
            stats_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz telemetry rate
    }
}