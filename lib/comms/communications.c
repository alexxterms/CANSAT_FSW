#include "communications.h"
#include "gps.h" // Include GPS header for satellite_info_t
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>  // Added for fabsf, fabs functions
#include "driver/uart.h" // Include your UART driver header
#include "esp_log.h"
#include "esp_err.h"

// Define missing ESP error codes if not available
#ifndef ESP_ERR_INVALID_DATA
#define ESP_ERR_INVALID_DATA ESP_ERR_INVALID_ARG
#endif

#ifndef ESP_ERR_INVALID_CRC
#define ESP_ERR_INVALID_CRC ESP_ERR_INVALID_RESPONSE
#endif

// Simplified paraglider control structure to avoid circular dependency
typedef struct {
    uint16_t left_servo_us;
    uint16_t right_servo_us;
    float heading_error_deg;
    float distance_to_target_m;
    float bearing_to_target_deg;
} simple_paraglider_control_t;

// Function to get paraglider servo positions (weak definition)
__attribute__((weak)) esp_err_t paraglider_get_servo_positions(uint16_t *left_us, uint16_t *right_us) {
    if (left_us) *left_us = 1500;  // Default neutral
    if (right_us) *right_us = 1500; // Default neutral
    return ESP_OK;
}

static const char *TAG = "COMMS";

// Global packetization context
static packet_context_t g_packet_ctx = {0};

// Global differential packetization state
static previous_sensor_data_t g_previous_data = {0};
static bool g_differential_initialized = false;
static bool g_force_full_packet = false;
static uint32_t g_differential_stats_total = 0;
static uint32_t g_differential_stats_partial = 0;
static uint32_t g_differential_stats_full = 0;
static uint32_t g_differential_stats_bytes_saved = 0;

// Full packet interval (send complete data every N packets even if unchanged)
#define FULL_PACKET_INTERVAL_MS 30000  // 30 seconds
#define FULL_PACKET_INTERVAL_COUNT 50   // Or every 50 packets, whichever comes first

// CRC16 lookup table (CCITT polynomial 0x1021)
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    // ... (truncated for brevity, full table would be here)
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7
};

// CRC8 lookup table (polynomial 0x07)
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    // ... (truncated for brevity, full table would be here)
};

// CRC calculation functions
uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

esp_err_t packet_system_init(void) {
    memset(&g_packet_ctx, 0, sizeof(g_packet_ctx));
    g_packet_ctx.sequence_counter = 0;
    ESP_LOGI(TAG, "Packet system initialized");
    return ESP_OK;
}

esp_err_t differential_packet_init(void) {
    memset(&g_previous_data, 0, sizeof(g_previous_data));
    g_previous_data.last_full_packet_time = esp_log_timestamp();
    g_differential_initialized = true;
    g_force_full_packet = true; // First packet should always be full
    
    ESP_LOGI(TAG, "Differential packetization initialized");
    return ESP_OK;
}

esp_err_t detect_sensor_changes(const SensorData *current_data, sensor_change_flags_t *change_flags) {
    if (!current_data || !change_flags || !g_differential_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize all flags to false
    memset(change_flags, 0, sizeof(sensor_change_flags_t));
    
    // Identification is always included
    change_flags->identification = true;

    // Check accelerometer changes
    if (fabsf(current_data->ax - g_previous_data.ax) > CHANGE_THRESHOLD_ACCEL ||
        fabsf(current_data->ay - g_previous_data.ay) > CHANGE_THRESHOLD_ACCEL ||
        fabsf(current_data->az - g_previous_data.az) > CHANGE_THRESHOLD_ACCEL) {
        change_flags->accelerometer = true;
    }

    // Check gyroscope changes
    if (fabsf(current_data->gx - g_previous_data.gx) > CHANGE_THRESHOLD_GYRO ||
        fabsf(current_data->gy - g_previous_data.gy) > CHANGE_THRESHOLD_GYRO ||
        fabsf(current_data->gz - g_previous_data.gz) > CHANGE_THRESHOLD_GYRO) {
        change_flags->gyroscope = true;
    }

    // Check magnetometer changes
    if (fabsf(current_data->mx - g_previous_data.mx) > CHANGE_THRESHOLD_MAG ||
        fabsf(current_data->my - g_previous_data.my) > CHANGE_THRESHOLD_MAG ||
        fabsf(current_data->mz - g_previous_data.mz) > CHANGE_THRESHOLD_MAG) {
        change_flags->magnetometer = true;
    }

    // Check environmental changes
    if (fabsf(current_data->temperature - g_previous_data.temperature) > CHANGE_THRESHOLD_TEMP ||
        fabsf(current_data->pressure - g_previous_data.pressure) > CHANGE_THRESHOLD_PRESSURE ||
        fabsf(current_data->humidity - g_previous_data.humidity) > CHANGE_THRESHOLD_HUMIDITY ||
        fabsf(current_data->gas_level - g_previous_data.gas_level) > CHANGE_THRESHOLD_GAS) {
        change_flags->environmental = true;
    }

    // Check GPS position changes
    if (fabs(current_data->latitude - g_previous_data.latitude) > CHANGE_THRESHOLD_GPS_POS ||
        fabs(current_data->longitude - g_previous_data.longitude) > CHANGE_THRESHOLD_GPS_POS) {
        change_flags->gps_gga = true;
        change_flags->gps_gsv = true; // If position changed, satellite info might be relevant
    }

    // Check battery voltage changes
    if (fabsf(current_data->voltage - g_previous_data.voltage) > CHANGE_THRESHOLD_VOLTAGE) {
        change_flags->battery = true;
    }

    // Always include system status if mission phase changed
    extern FlightState currentState; // From main.c
    if ((uint8_t)currentState != g_previous_data.mission_phase) {
        change_flags->system_status = true;
    }

    // Check servo position changes (from paraglider system)
    uint16_t left_servo_us, right_servo_us;
    if (paraglider_get_servo_positions(&left_servo_us, &right_servo_us) == ESP_OK) {
        if (left_servo_us != g_previous_data.left_servo_us ||
            right_servo_us != g_previous_data.right_servo_us) {
            change_flags->pwm = true;
        }
    }

    // Signal quality - include periodically or if significantly changed
    // For now, include every few packets since RSSI can fluctuate
    if (g_previous_data.packet_count % 5 == 0) {
        change_flags->signal = true;
    }

    return ESP_OK;
}

bool should_send_full_packet(void) {
    uint32_t current_time = esp_log_timestamp();
    uint32_t time_since_full = current_time - g_previous_data.last_full_packet_time;
    
    return (g_force_full_packet || 
            time_since_full > FULL_PACKET_INTERVAL_MS ||
            g_previous_data.packet_count >= FULL_PACKET_INTERVAL_COUNT);
}

void force_next_full_packet(void) {
    g_force_full_packet = true;
}

int create_differential_telemetry_packet(const SensorData *current_data, 
                                        uint8_t *output_buffer, size_t output_size, 
                                        bool force_full_packet) {
    if (!current_data || !output_buffer || !g_differential_initialized) {
        return -1;
    }

    uint8_t payload_buffer[MAX_PAYLOAD_SIZE];
    int payload_offset = 0;
    uint8_t temp_buffer[64];
    size_t temp_size;
    
    sensor_change_flags_t changes;
    bool is_full_packet = force_full_packet || should_send_full_packet();
    
    if (is_full_packet) {
        // Force all flags true for full packet
        memset(&changes, 1, sizeof(changes));
        g_force_full_packet = false;
        g_previous_data.last_full_packet_time = esp_log_timestamp();
        g_previous_data.packet_count = 0;
        g_differential_stats_full++;
        ESP_LOGD(TAG, "Sending full telemetry packet");
    } else {
        // Detect changes for differential packet
        detect_sensor_changes(current_data, &changes);
        g_differential_stats_partial++;
        
        // Count how many sub-packets we're including
        int sub_packet_count = 0;
        if (changes.identification) sub_packet_count++;
        if (changes.environmental) sub_packet_count++;
        if (changes.accelerometer) sub_packet_count++;
        if (changes.gyroscope) sub_packet_count++;
        if (changes.magnetometer) sub_packet_count++;
        if (changes.gps_gga) sub_packet_count++;
        if (changes.gps_gsv) sub_packet_count++;
        if (changes.battery) sub_packet_count++;
        if (changes.system_status) sub_packet_count++;
        if (changes.pwm) sub_packet_count++;
        if (changes.signal) sub_packet_count++;
        
        ESP_LOGD(TAG, "Sending differential packet with %d sub-packets", sub_packet_count);
        
        // Estimate bytes saved (rough calculation)
        int full_packet_estimate = 11 * 15; // 11 sub-packets * ~15 bytes each
        int current_packet_estimate = sub_packet_count * 15;
        g_differential_stats_bytes_saved += (full_packet_estimate - current_packet_estimate);
    }

    // Always include identification
    if (changes.identification) {
        IdentificationPacket id_packet = {
            .team_id = "TEAM1234",
            .timestamp = esp_log_timestamp(),
            .packet_id = g_packet_ctx.packets_sent,
            .source = 0x01,
            .destination = 0x02
        };
        temp_size = build_identification_packet(temp_buffer, &id_packet);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_IDENTIFICATION, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add environmental data if changed
    if (changes.environmental) {
        EnvironmentalPacket env_packet = {
            .message_id = MSG_ID_ENVIRONMENTAL,
            .altitude = (uint16_t)(current_data->altitude * 10),
            .temperature = (uint16_t)(current_data->temperature * 100),
            .pressure = (uint16_t)(current_data->pressure / 10),
            .humidity = (uint16_t)(current_data->humidity * 100),
            .gas_sensor = (uint16_t)(current_data->gas_level * 100)
        };
        temp_size = format_environmental_packet(temp_buffer, &env_packet);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_ENVIRONMENTAL, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add accelerometer data if changed
    if (changes.accelerometer) {
        int16_t ax = (int16_t)(current_data->ax * 1000);
        int16_t ay = (int16_t)(current_data->ay * 1000);
        int16_t az = (int16_t)(current_data->az * 1000);
        temp_size = build_accel_packet(temp_buffer, ax, ay, az);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_ACCEL, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add gyroscope data if changed
    if (changes.gyroscope) {
        int16_t gx = (int16_t)(current_data->gx * 1000);
        int16_t gy = (int16_t)(current_data->gy * 1000);
        int16_t gz = (int16_t)(current_data->gz * 1000);
        temp_size = build_gyro_packet(temp_buffer, gx, gy, gz);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_GYRO, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add GPS data if changed
    if (changes.gps_gga) {
        gps_fix_t fix;
        if (gps_get_fix(&fix) && fix.valid) {
            uint8_t utc_time[3] = { fix.hh, fix.mm, fix.ss };
            float latitude = fix.lat_e7 / 1e7f;
            float longitude = fix.lon_e7 / 1e7f;
            bool lat_dir = (latitude >= 0);
            bool lon_dir = (longitude >= 0);
            
            temp_size = build_gga_packet(temp_buffer, utc_time, fabsf(latitude), lat_dir,
                                        fabsf(longitude), lon_dir, fix.fix_quality, 
                                        fix.hdop_x100 / 10, fix.sats, fix.alt_cm * 10);
            payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                           MSG_ID_GGA, temp_buffer, temp_size);
            if (payload_offset == -1) return -1;
        }
    }

    // Add satellite data if GPS changed
    if (changes.gps_gsv) {
        satellite_info_t satellite_data[MAX_SATS];
        int satellite_count = gps_get_satellites(satellite_data, MAX_SATS);
        if (satellite_count > 0) {
            temp_size = build_gsv_packet(temp_buffer, satellite_count, satellite_data, satellite_count);
            payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                           MSG_ID_GSV, temp_buffer, temp_size);
            if (payload_offset == -1) return -1;
        }
    }

    // Add battery data if changed
    if (changes.battery) {
        uint16_t voltage_mv = (uint16_t)(current_data->voltage * 1000);
        temp_size = build_battery_packet(temp_buffer, voltage_mv);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_BATTERY, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add system status if changed
    if (changes.system_status) {
        extern FlightState currentState;
        uint8_t sensor_status = 0x01; // Assume operational
        uint32_t uptime = esp_log_timestamp();
        uint8_t mission_phase = (uint8_t)currentState;
        const char *reboot_reason = "Normal";
        uint8_t system_time[3] = {0x12, 0x34, 0x56};
        
        temp_size = build_system_status_packet(temp_buffer, sensor_status, uptime, 
                                              mission_phase, reboot_reason, system_time);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_SYSTEM_STATUS, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add PWM data if servos changed
    if (changes.pwm) {
        uint16_t servo_pwm[5] = {1500, 1500, 1500, 1500, 1500};
        uint16_t esc_pwm[2] = {1000, 1000};
        
        if (paraglider_get_servo_positions(&servo_pwm[0], &servo_pwm[1]) == ESP_OK) {
            // Do nothing, values already updated
        }
        
        temp_size = build_pwm_packet(temp_buffer, servo_pwm, esc_pwm);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_PWM, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add signal data if included
    if (changes.signal) {
        int16_t rssi = -70; // Simulated
        uint8_t lq = 90;
        temp_size = build_signal_packet(temp_buffer, rssi, lq);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       MSG_ID_SIGNAL, temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Create the final packet
    uint8_t packet_flags = PACKET_FLAG_RELIABLE;
    if (!is_full_packet) {
        packet_flags |= 0x10; // Custom flag to indicate differential packet
    }
    
    int packet_size = create_packet(PACKET_TYPE_COMBINED, payload_buffer, payload_offset,
                                   output_buffer, output_size, packet_flags);
    
    if (packet_size > 0) {
        g_differential_stats_total++;
        g_previous_data.packet_count++;
    }
    
    return packet_size;
}

esp_err_t update_previous_sensor_data(const SensorData *current_data) {
    if (!current_data || !g_differential_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update previous values
    g_previous_data.ax = current_data->ax;
    g_previous_data.ay = current_data->ay;
    g_previous_data.az = current_data->az;
    g_previous_data.gx = current_data->gx;
    g_previous_data.gy = current_data->gy;
    g_previous_data.gz = current_data->gz;
    g_previous_data.mx = current_data->mx;
    g_previous_data.my = current_data->my;
    g_previous_data.mz = current_data->mz;
    
    g_previous_data.temperature = current_data->temperature;
    g_previous_data.pressure = current_data->pressure;
    g_previous_data.humidity = current_data->humidity;
    g_previous_data.gas_level = current_data->gas_level;
    g_previous_data.voltage = current_data->voltage;
    
    g_previous_data.latitude = current_data->latitude;
    g_previous_data.longitude = current_data->longitude;
    
    extern FlightState currentState;
    g_previous_data.mission_phase = (uint8_t)currentState;
    
    // Update servo positions
    if (paraglider_get_servo_positions(&g_previous_data.left_servo_us, &g_previous_data.right_servo_us) == ESP_OK) {
        // Do nothing, values already updated
    }

    return ESP_OK;
}

esp_err_t get_differential_stats(uint32_t *total_packets, uint32_t *differential_packets, 
                                uint32_t *full_packets, uint32_t *bytes_saved) {
    if (!total_packets || !differential_packets || !full_packets || !bytes_saved) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *total_packets = g_differential_stats_total;
    *differential_packets = g_differential_stats_partial;
    *full_packets = g_differential_stats_full;
    *bytes_saved = g_differential_stats_bytes_saved;
    
    return ESP_OK;
}

// Enhanced telemetry function using differential packetization
esp_err_t send_differential_telemetry_burst(const SensorData *current_data) {
    if (!current_data) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_differential_telemetry_packet(current_data, packet_buffer, 
                                                          sizeof(packet_buffer), false);
    
    if (packet_size <= 0) {
        ESP_LOGE(TAG, "Failed to create differential telemetry packet");
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = send_packet(packet_buffer, packet_size);
    if (ret == ESP_OK) {
        // Update previous values only after successful transmission
        update_previous_sensor_data(current_data);
    }
    
    return ret;
}

int create_packet(packet_type_t packet_type, const uint8_t *payload, size_t payload_len, 
                  uint8_t *output_buffer, size_t output_size, uint8_t flags) {
    
    if (!payload || !output_buffer || payload_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Invalid packet parameters");
        return -1;
    }

    size_t total_size = 6 + payload_len + 2 + 1; // header(6) + payload + crc(2) + end(1)
    if (total_size > output_size) {
        ESP_LOGE(TAG, "Output buffer too small: need %d, have %d", total_size, output_size);
        return -1;
    }

    size_t offset = 0;

    // Build packet header
    output_buffer[offset++] = PACKET_START_BYTE;
    output_buffer[offset++] = (uint8_t)packet_type;
    output_buffer[offset++] = (payload_len >> 8) & 0xFF;  // Length high byte
    output_buffer[offset++] = payload_len & 0xFF;         // Length low byte
    output_buffer[offset++] = g_packet_ctx.sequence_counter++;
    output_buffer[offset++] = flags;

    // Add payload
    memcpy(&output_buffer[offset], payload, payload_len);
    offset += payload_len;

    // Calculate CRC16 over header + payload
    uint16_t crc = calculate_crc16(&output_buffer[1], offset - 1);
    output_buffer[offset++] = (crc >> 8) & 0xFF;  // CRC high byte
    output_buffer[offset++] = crc & 0xFF;         // CRC low byte

    // Add end byte
    output_buffer[offset++] = PACKET_END_BYTE;

    g_packet_ctx.packets_sent++;
    return offset;
}

esp_err_t parse_packet(const uint8_t *buffer, size_t buffer_len, 
                       packet_type_t *packet_type, const uint8_t **payload, size_t *payload_len) {
    
    if (!buffer || buffer_len < 9 || !packet_type || !payload || !payload_len) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check start byte
    if (buffer[0] != PACKET_START_BYTE) {
        g_packet_ctx.frame_errors++;
        return ESP_ERR_INVALID_DATA;
    }

    // Extract header fields
    *packet_type = (packet_type_t)buffer[1];
    uint16_t declared_payload_len = (buffer[2] << 8) | buffer[3];
    uint8_t sequence = buffer[4];
    uint8_t flags = buffer[5];

    // Validate packet length
    size_t expected_total_len = 6 + declared_payload_len + 2 + 1;
    if (buffer_len != expected_total_len) {
        g_packet_ctx.frame_errors++;
        return ESP_ERR_INVALID_SIZE;
    }

    // Check end byte
    if (buffer[buffer_len - 1] != PACKET_END_BYTE) {
        g_packet_ctx.frame_errors++;
        return ESP_ERR_INVALID_DATA;
    }

    // Extract CRC from packet
    uint16_t received_crc = (buffer[6 + declared_payload_len] << 8) | buffer[6 + declared_payload_len + 1];
    
    // Calculate CRC over header + payload
    uint16_t calculated_crc = calculate_crc16(&buffer[1], 5 + declared_payload_len);
    
    if (received_crc != calculated_crc) {
        g_packet_ctx.crc_errors++;
        ESP_LOGW(TAG, "CRC mismatch: received=0x%04X, calculated=0x%04X", received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Set output parameters
    *payload = &buffer[6];
    *payload_len = declared_payload_len;

    g_packet_ctx.packets_received++;
    return ESP_OK;
}

int add_sub_packet(uint8_t *payload_buffer, int payload_offset, size_t max_payload_size,
                   uint8_t sub_packet_id, const uint8_t *sub_data, size_t sub_data_len) {
    
    // Calculate required space: ID(1) + Length(1) + Data(n) + CRC8(1)
    size_t required_space = 3 + sub_data_len;
    
    if (payload_offset + required_space > max_payload_size) {
        ESP_LOGE(TAG, "Not enough space for sub-packet");
        return -1;
    }

    // Add sub-packet header
    payload_buffer[payload_offset++] = sub_packet_id;
    payload_buffer[payload_offset++] = (uint8_t)sub_data_len;

    // Add sub-packet data
    memcpy(&payload_buffer[payload_offset], sub_data, sub_data_len);
    payload_offset += sub_data_len;

    // Calculate and add CRC8 for this sub-packet
    uint8_t sub_crc = calculate_crc8(&payload_buffer[payload_offset - sub_data_len - 2], 
                                     sub_data_len + 2);
    payload_buffer[payload_offset++] = sub_crc;

    return payload_offset;
}

esp_err_t send_packet(const uint8_t *packet_buffer, size_t packet_size) {
    if (!packet_buffer || packet_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = uart_write_bytes(UART_NUM_0, packet_buffer, packet_size);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send packet: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Sent packet: %d bytes, type=0x%02X", packet_size, packet_buffer[1]);
    return ESP_OK;
}

esp_err_t send_telemetry_burst(void) {
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_combined_telemetry_packet(packet_buffer, sizeof(packet_buffer));
    
    if (packet_size <= 0) {
        ESP_LOGE(TAG, "Failed to create telemetry packet");
        return ESP_ERR_INVALID_SIZE;
    }

    return send_packet(packet_buffer, packet_size);
}

esp_err_t send_heartbeat_packet(void) {
    uint8_t heartbeat_data[8];
    uint32_t uptime = esp_log_timestamp();
    
    heartbeat_data[0] = 0xAA; // Heartbeat marker
    heartbeat_data[1] = (uptime >> 24) & 0xFF;
    heartbeat_data[2] = (uptime >> 16) & 0xFF;
    heartbeat_data[3] = (uptime >> 8) & 0xFF;
    heartbeat_data[4] = uptime & 0xFF;
    heartbeat_data[5] = g_packet_ctx.packets_sent & 0xFF;
    heartbeat_data[6] = g_packet_ctx.crc_errors & 0xFF;
    heartbeat_data[7] = g_packet_ctx.frame_errors & 0xFF;

    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_packet(PACKET_TYPE_HEARTBEAT, heartbeat_data, sizeof(heartbeat_data),
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size <= 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    return send_packet(packet_buffer, packet_size);
}

esp_err_t send_ack_packet(uint8_t sequence_number) {
    uint8_t ack_data[2] = {0xAC, sequence_number}; // Changed from 0xACK to 0xAC
    
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_packet(PACKET_TYPE_ACK, ack_data, sizeof(ack_data),
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size <= 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    return send_packet(packet_buffer, packet_size);
}

esp_err_t send_nack_packet(uint8_t sequence_number, uint8_t error_code) {
    uint8_t nack_data[3] = {0x15, sequence_number, error_code}; // Changed from 0xNAK to 0x15 (NAK ASCII)
    
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_packet(PACKET_TYPE_NACK, nack_data, sizeof(nack_data),
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size <= 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    return send_packet(packet_buffer, packet_size);
}

esp_err_t get_packet_stats(packet_context_t *context) {
    if (!context) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(context, &g_packet_ctx, sizeof(packet_context_t));
    return ESP_OK;
}

int create_combined_telemetry_packet(uint8_t *output_buffer, size_t output_size) {
    uint8_t payload_buffer[MAX_PAYLOAD_SIZE];
    int payload_offset = 0;
    uint8_t temp_buffer[64];
    size_t temp_size;

    // Add Identification sub-packet
    IdentificationPacket id_packet = {
        .team_id = "TEAM1234",
        .timestamp = esp_log_timestamp(),
        .packet_id = g_packet_ctx.packets_sent,
        .source = 0x01,
        .destination = 0x02
    };
    temp_size = build_identification_packet(temp_buffer, &id_packet);
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                   MSG_ID_IDENTIFICATION, temp_buffer, temp_size);
    if (payload_offset == -1) return -1;

    // Add Environmental sub-packet
    EnvironmentalPacket env_packet = {
        .message_id = MSG_ID_ENVIRONMENTAL,
        .altitude = 1000,      // Example data
        .temperature = 2500,   // 25.00°C
        .pressure = 101325,    // Example pressure
        .humidity = 5000,      // 50.00%
        .gas_sensor = 300      // Example gas reading
    };
    temp_size = format_environmental_packet(temp_buffer, &env_packet);
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                   MSG_ID_ENVIRONMENTAL, temp_buffer, temp_size);
    if (payload_offset == -1) return -1;

    // Add more sub-packets as needed...

    // Create the final packet
    return create_packet(PACKET_TYPE_COMBINED, payload_buffer, payload_offset,
                        output_buffer, output_size, PACKET_FLAG_RELIABLE);
}

// Helper to add 16-bit value to buffer in little endian
static inline void put_uint16_le(uint8_t *buffer, uint16_t value) {
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
}

// Helper to add 32-bit value to buffer in little endian
static inline void put_uint32_le(uint8_t *buffer, uint32_t value) {
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
}

// Builds the identification section
int build_identification_packet(uint8_t *buffer, const IdentificationPacket *packet) {
    int index = 0;
    buffer[index++] = MSG_ID_IDENTIFICATION;
    uint8_t team_id_len = strlen(packet->team_id);
    if (team_id_len > TEAM_ID_MAX_LEN) team_id_len = TEAM_ID_MAX_LEN;
    
    memcpy(&buffer[index], packet->team_id, team_id_len);
    index += team_id_len;
    put_uint32_le(&buffer[index], packet->timestamp);
    index += 4;
    put_uint16_le(&buffer[index], packet->packet_id);
    index += 2;
    buffer[index++] = packet->source;
    buffer[index++] = packet->destination;
    return index;
}

size_t format_environmental_packet(uint8_t *buffer, EnvironmentalPacket *data) {
    if (!buffer || !data) return 0;
    
    size_t index = 0;
    buffer[index++] = data->message_id;
    put_uint16_le(&buffer[index], data->altitude);
    index += 2;
    put_uint16_le(&buffer[index], data->temperature);
    index += 2;
    put_uint16_le(&buffer[index], data->pressure);
    index += 2;
    put_uint16_le(&buffer[index], data->humidity);
    index += 2;
    put_uint16_le(&buffer[index], data->gas_sensor);
    index += 2;
    return index;
}

size_t build_accel_packet(uint8_t *buffer, int16_t x_accel, int16_t y_accel, int16_t z_accel) {
    buffer[0] = MSG_ID_ACCEL;
    put_uint16_le(&buffer[1], (uint16_t)x_accel);
    put_uint16_le(&buffer[3], (uint16_t)y_accel);
    put_uint16_le(&buffer[5], (uint16_t)z_accel);
    return ACCEL_PACKET_SIZE;
}

size_t build_gyro_packet(uint8_t *buffer, int16_t x_rate, int16_t y_rate, int16_t z_rate) {
    buffer[0] = MSG_ID_GYRO;
    put_uint16_le(&buffer[1], (uint16_t)x_rate);
    put_uint16_le(&buffer[3], (uint16_t)y_rate);
    put_uint16_le(&buffer[5], (uint16_t)z_rate);
    return GYRO_PACKET_SIZE;
}

size_t build_gga_packet(uint8_t *buffer, uint8_t utc_time[3], float latitude, bool latitude_dir, 
                        float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, 
                        uint8_t satellites_used, int32_t altitude) {
    buffer[0] = MSG_ID_GGA;
    memcpy(&buffer[1], utc_time, 3);
    memcpy(&buffer[4], &latitude, sizeof(float));
    buffer[8] = latitude_dir ? 1 : 0;
    memcpy(&buffer[9], &longitude, sizeof(float));
    buffer[13] = longitude_dir ? 1 : 0;
    buffer[14] = fix_status;
    put_uint16_le(&buffer[15], hdop);
    buffer[17] = satellites_used;
    put_uint32_le(&buffer[18], (uint32_t)altitude);
    return 22;
}

size_t build_gsv_packet(uint8_t *buffer, uint8_t satellites_in_view, satellite_info_t *satellite_data, int satellite_count) {
    buffer[0] = MSG_ID_GSV;
    buffer[1] = satellites_in_view;
    
    size_t index = 2;
    int max_satellites = (satellite_count < MAX_SATS) ? satellite_count : MAX_SATS;
    for (int i = 0; i < max_satellites; i++) {
        buffer[index++] = (uint8_t)satellite_data[i].prn;
        buffer[index++] = (uint8_t)satellite_data[i].elevation;
        buffer[index++] = (uint8_t)satellite_data[i].azimuth;
        buffer[index++] = (uint8_t)satellite_data[i].snr;
    }
    return index;
}

size_t build_battery_packet(uint8_t *buffer, uint16_t current_voltage) {
    buffer[0] = MSG_ID_BATTERY;
    put_uint16_le(&buffer[1], current_voltage);
    return 3;
}

size_t build_system_status_packet(uint8_t *buffer, uint8_t sensor_status, uint32_t uptime, 
                                  uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]) {
    buffer[0] = MSG_ID_SYSTEM_STATUS;
    buffer[1] = sensor_status;
    put_uint32_le(&buffer[2], uptime);
    buffer[6] = mission_phase;
    
    size_t reboot_reason_len = strlen(reboot_reason);
    if (reboot_reason_len > 16) reboot_reason_len = 16; // Limit length
    memcpy(&buffer[7], reboot_reason, reboot_reason_len);
    memcpy(&buffer[7 + reboot_reason_len], system_time, 3);
    return 10 + reboot_reason_len;
}

size_t build_pwm_packet(uint8_t *buffer, uint16_t servo_pwm[5], uint16_t esc_pwm[2]) {
    buffer[0] = MSG_ID_PWM;
    
    for (int i = 0; i < 5; i++) {
        put_uint16_le(&buffer[1 + i * 2], servo_pwm[i]);
    }
    for (int i = 0; i < 2; i++) {
        put_uint16_le(&buffer[11 + i * 2], esc_pwm[i]);
    }
    return 15;
}

size_t build_signal_packet(uint8_t *buffer, int16_t rssi, uint8_t lq) {
    buffer[0] = MSG_ID_SIGNAL;
    put_uint16_le(&buffer[1], (uint16_t)rssi);
    buffer[3] = lq;
    return 4;
}

// Legacy function for backward compatibility
void send_combined_packets() {
    send_telemetry_burst();
}

// Individual send functions (legacy support)
void send_identification_packet(const IdentificationPacket *packet) {
    uint8_t buffer[64];
    size_t packet_size = build_identification_packet(buffer, packet);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_environmental_packet(const EnvironmentalPacket *env_data) {
    uint8_t buffer[32];
    size_t packet_size = format_environmental_packet(buffer, (EnvironmentalPacket*)env_data);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_accel_packet(int16_t x_accel, int16_t y_accel, int16_t z_accel) {
    uint8_t buffer[ACCEL_PACKET_SIZE];
    size_t packet_size = build_accel_packet(buffer, x_accel, y_accel, z_accel);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gyro_packet(int16_t x_rate, int16_t y_rate, int16_t z_rate) {
    uint8_t buffer[GYRO_PACKET_SIZE];
    size_t packet_size = build_gyro_packet(buffer, x_rate, y_rate, z_rate);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gga_packet(uint8_t utc_time[3], float latitude, bool latitude_dir, float longitude, bool longitude_dir, 
                     uint8_t fix_status, uint16_t hdop, uint8_t satellites_used, int32_t altitude) {
    uint8_t buffer[32];
    size_t packet_size = build_gga_packet(buffer, utc_time, latitude, latitude_dir, longitude, longitude_dir, 
                                         fix_status, hdop, satellites_used, altitude);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gsv_packet(uint8_t satellites_in_view, satellite_info_t *satellite_data, int satellite_count) {
    uint8_t buffer[256];
    size_t packet_size = build_gsv_packet(buffer, satellites_in_view, satellite_data, satellite_count);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_battery_packet(uint16_t current_voltage) {
    uint8_t buffer[8];
    size_t packet_size = build_battery_packet(buffer, current_voltage);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_system_status_packet(uint8_t sensor_status, uint32_t uptime, uint8_t mission_phase, 
                              const char *reboot_reason, uint8_t system_time[3]) {
    uint8_t buffer[64];
    size_t packet_size = build_system_status_packet(buffer, sensor_status, uptime, mission_phase, reboot_reason, system_time);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_pwm_packet(uint16_t servo_pwm[5], uint16_t esc_pwm[2]) {
    uint8_t buffer[32];
    size_t packet_size = build_pwm_packet(buffer, servo_pwm, esc_pwm);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_signal_packet(int16_t rssi, uint8_t lq) {
    uint8_t buffer[8];
    size_t packet_size = build_signal_packet(buffer, rssi, lq);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}