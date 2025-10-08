#include "communications.h"
#include "gps.h" // Include GPS header for satellite_info_t
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>  // Added for fabsf, fabs functions
#include "driver/uart.h" // Include your UART driver header
#include "esp_log.h"
#include "esp_err.h"

// Remove zlib dependency - we'll implement our own CRC32

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
  
};

// CRC8 lookup table (polynomial 0x07)
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    // ... (truncated for brevity, full table would be here)
};

// CRC32 lookup table (IEEE 802.3 polynomial 0xEDB88320)
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
    0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
    0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xDEBB9EC5,
    0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8,
    0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

// CRC calculation functions
uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value as per ground station
    
    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
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

// CRC32 calculation function with direct calculation method
uint32_t calculate_crc32(const uint8_t *data, size_t length) {
    // Use direct calculation (no lookup table) for debugging
    const uint32_t polynomial = 0xEDB88320; // IEEE 802.3 polynomial (reversed)
    uint32_t crc = 0xFFFFFFFF; // Initial value
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc ^ 0xFFFFFFFF; // Final XOR value
}

// Helper function to write uint16 in little endian format
static void put_uint16_le(uint8_t *buffer, uint16_t value) {
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
}

// Helper function to write uint32 in little endian format
static void put_uint32_le(uint8_t *buffer, uint32_t value) {
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
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
            .team_id = "CANSAT014",
            .timestamp = esp_log_timestamp(),
            .packet_id = g_packet_ctx.packets_sent,
            .source = 0x00,
            .destination = 0x01
        };
        temp_size = build_identification_packet(temp_buffer, &id_packet);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
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
            
        };
        temp_size = format_environmental_packet(temp_buffer, &env_packet);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add accelerometer data if changed
    if (changes.accelerometer) {
        int16_t ax = (int16_t)(current_data->ax * 1000);
        int16_t ay = (int16_t)(current_data->ay * 1000);
        int16_t az = (int16_t)(current_data->az * 1000);
        temp_size = build_accel_packet(temp_buffer, ax, ay, az);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                      temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add gyroscope data if changed
    if (changes.gyroscope) {
        int16_t gx = (int16_t)(current_data->gx * 1000);
        int16_t gy = (int16_t)(current_data->gy * 1000);
        int16_t gz = (int16_t)(current_data->gz * 1000);
        temp_size = build_gyro_packet(temp_buffer, gx, gy, gz);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add magnetometer data if changed
    if (changes.magnetometer) {
        int16_t mx = (int16_t)(current_data->mx * 1000);
        int16_t my = (int16_t)(current_data->my * 1000);
        int16_t mz = (int16_t)(current_data->mz * 1000);
        temp_size = build_magnetometer_packet(temp_buffer, mx, my, mz);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                       temp_buffer, temp_size);
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
                                            temp_buffer, temp_size);
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
                                           temp_buffer, temp_size);
            if (payload_offset == -1) return -1;
        }
    }

    // Add battery data if changed
    /*
    if (changes.battery) {
        uint16_t voltage_mv = (uint16_t)(current_data->voltage * 1000);
        uint16_t current_ma = 0; // Simulated
        temp_size = build_battery_packet(temp_buffer, voltage_mv);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }
    */
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
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add PWM data if servos changed
    if (changes.pwm) {
        uint16_t servo_pwm[3] = {1500, 1500, 1500};
        uint16_t motor_rpm_x = 0;
        uint16_t motor_rpm_y = 0;
        uint16_t esc_pwm[2] = {1000, 1000};
        
        if (paraglider_get_servo_positions(&servo_pwm[0], &servo_pwm[1]) == ESP_OK) {
            // Do nothing, values already updated
        }
        
        temp_size = build_pwm_packet(temp_buffer, servo_pwm, motor_rpm_x, motor_rpm_y, esc_pwm);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }

    // Add signal data if included
    /*
    if (changes.signal) {
        int16_t rssi = -70; // Simulated
        uint8_t lq = 90;
        temp_size = build_signal_packet(temp_buffer, rssi, lq);
        payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                        temp_buffer, temp_size);
        if (payload_offset == -1) return -1;
    }
   */
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

esp_err_t parse_packet(const uint8_t *buffer, size_t buffer_len, 
                       packet_type_t *packet_type, const uint8_t **payload, size_t *payload_len) {
    
    if (!buffer || buffer_len < 11 || !packet_type || !payload || !payload_len) {
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

    // Validate packet length (header + payload + CRC32 + end byte)
    size_t expected_total_len = 6 + declared_payload_len + 4;
    if (buffer_len != expected_total_len) {
        g_packet_ctx.frame_errors++;
        return ESP_ERR_INVALID_SIZE;
    }

    // Extract CRC32 from packet
    uint32_t received_crc = (buffer[6 + declared_payload_len] << 24) | 
                           (buffer[6 + declared_payload_len + 1] << 16) |
                           (buffer[6 + declared_payload_len + 2] << 8) |
                           buffer[6 + declared_payload_len + 3];
    
    // Calculate CRC32 over header + payload
    uint32_t calculated_crc = calculate_crc32(&buffer[1], 5 + declared_payload_len);
    
    if (received_crc != calculated_crc) {
        g_packet_ctx.crc_errors++;
        ESP_LOGW(TAG, "CRC32 mismatch: received=0x%08X, calculated=0x%08X", received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Set output parameters
    *payload = &buffer[6];
    *payload_len = declared_payload_len;

    g_packet_ctx.packets_received++;
    return ESP_OK;
}

size_t build_accel_packet(uint8_t *buffer, int16_t x_accel, int16_t y_accel, int16_t z_accel) {
    buffer[0] = MSG_ID_ACCEL;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    put_uint16_le(&buffer[5], (uint16_t)x_accel);
    put_uint16_le(&buffer[7], (uint16_t)y_accel);
    put_uint16_le(&buffer[9], (uint16_t)z_accel);
    // CRC16 will be added by sub-packet handler
    return 11; // MSG_ID + packet_id(4) + 3 * uint16_t
}


size_t build_environmental_packet(uint8_t *buffer, uint16_t altitude, uint16_t temperature, 
                                    uint16_t pressure, uint16_t humidity, 
                                    uint16_t voc, uint16_t ethanol, uint16_t h2, 
                                    uint16_t nh3, uint16_t ch4, uint16_t aqi) {
    buffer[0] = MSG_ID_ENVIRONMENTAL;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    put_uint16_le(&buffer[5], altitude);
    put_uint16_le(&buffer[7], temperature);
    put_uint16_le(&buffer[9], pressure);
    put_uint16_le(&buffer[11], humidity);
    put_uint16_le(&buffer[13], voc);
    put_uint16_le(&buffer[15], ethanol);
    put_uint16_le(&buffer[17], h2);
    put_uint16_le(&buffer[19], nh3);
    put_uint16_le(&buffer[21], ch4);
    put_uint16_le(&buffer[23], aqi);
    // CRC16 will be added by sub-packet handler
    return 25; // MSG_ID + packet_id(4) + 10 * uint16_t
}
size_t build_gyro_packet(uint8_t *buffer, int16_t x_rate, int16_t y_rate, int16_t z_rate) {
    buffer[0] = MSG_ID_GYRO;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    put_uint16_le(&buffer[5], (uint16_t)x_rate);
    put_uint16_le(&buffer[7], (uint16_t)y_rate);
    put_uint16_le(&buffer[9], (uint16_t)z_rate);
    // CRC16 will be added by sub-packet handler
    return 11; // MSG_ID + packet_id(4) + 3 * uint16_t
}

size_t build_magnetometer_packet(uint8_t *buffer, int16_t x_mag, int16_t y_mag, int16_t z_mag) {
    buffer[0] = MSG_ID_MAGNETOMETER;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    put_uint16_le(&buffer[5], (uint16_t)x_mag);
    put_uint16_le(&buffer[7], (uint16_t)y_mag);
    put_uint16_le(&buffer[9], (uint16_t)z_mag);
    // CRC16 will be added by sub-packet handler
    return 11; // MSG_ID + packet_id(4) + 3 * uint16_t
}

size_t build_battery_packet(uint8_t *buffer, uint16_t current_voltage, uint16_t voltage_draw, 
                            uint16_t current_draw, uint16_t current_drawn) {
    buffer[0] = MSG_ID_BATTERY;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    put_uint16_le(&buffer[5], current_voltage);
    // Add additional battery fields as per spec (using default values for now)
    put_uint16_le(&buffer[7], voltage_draw); // voltage_draw in mV, scaled
    put_uint16_le(&buffer[9], current_draw); // current_draw in mA, scaled
    put_uint16_le(&buffer[11], current_drawn); // current_drawn in mA, scaled
    // CRC16 will be added by sub-packet handler
    return 13; // MSG_ID + packet_id(4) + 4 * uint16_t
}

size_t build_system_status_packet(uint8_t *buffer, uint8_t sensor_status, uint32_t uptime, 
                                  uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]) {
    int index = 0;
    buffer[index++] = MSG_ID_SYSTEM_STATUS;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[index], g_packet_ctx.packets_sent);
    index += 4;
    buffer[index++] = sensor_status;
    put_uint32_le(&buffer[index], uptime);
    index += 4;
    buffer[index++] = mission_phase;
    size_t reboot_reason_len = strlen(reboot_reason);
    if (reboot_reason_len > 100) reboot_reason_len = 100; // Limit length to fit packet
    memcpy(&buffer[index], reboot_reason, reboot_reason_len);
    index += reboot_reason_len;
    buffer[index++] = '\0'; // Null terminator
    memcpy(&buffer[index], system_time, 3);
    index += 3;
   

    return index;
}

// Builds the identification section
int build_identification_packet(uint8_t *buffer, const IdentificationPacket *packet) {
    int index = 0;
    buffer[index++] = MSG_ID_IDENTIFICATION;
    
    // Add 4-byte packet ID field in little-endian format
    put_uint32_le(&buffer[index], packet->packet_id);
    index += 4;
    
    // Add team ID as string with null terminator
    uint8_t team_id_len = strlen(packet->team_id);
    if (team_id_len > TEAM_ID_MAX_LEN) team_id_len = TEAM_ID_MAX_LEN;
    
    memcpy(&buffer[index], packet->team_id, team_id_len);
    index += team_id_len;
    buffer[index++] = '\0'; // Add null terminator explicitly
    
    // Add timestamp in little-endian format
    put_uint32_le(&buffer[index], packet->timestamp);
    index += 4;
    
    // Add source and destination fields
    buffer[index++] = packet->source;
    buffer[index++] = packet->destination;
    
    // CRC16 will be added by the caller in big-endian format
    return index;
}

size_t format_environmental_packet(uint8_t *buffer, EnvironmentalPacket *data) {
    if (!buffer || !data) return 0;
    
    size_t index = 0;
    buffer[index++] = data->message_id;
    
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[index], g_packet_ctx.packets_sent);
    index += 4;
    
    put_uint16_le(&buffer[index], data->altitude);
    index += 2;
    put_uint16_le(&buffer[index], data->temperature);
    index += 2;
    put_uint16_le(&buffer[index], data->pressure);
    index += 2;
    put_uint16_le(&buffer[index], data->humidity);
    index += 2;
    
    // Add all gas concentration fields as per spec
    put_uint16_le(&buffer[index], data->voc_concentration);
    index += 2;
    put_uint16_le(&buffer[index], data->ethanol_concentration);
    index += 2;
    put_uint16_le(&buffer[index], data->h2_concentration);
    index += 2;
    put_uint16_le(&buffer[index], data->nh3_concentration);
    index += 2;
    put_uint16_le(&buffer[index], data->ch4_concentration);
    index += 2;
    put_uint16_le(&buffer[index], data->air_quality_index);
    index += 2;
    
    // CRC16 will be added by sub-packet handler
    return index;
}

size_t build_gga_packet(uint8_t *buffer, uint8_t utc_time[3], float latitude, bool latitude_dir, 
                        float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, 
                        uint8_t satellites_used, uint16_t altitude) {
    buffer[0] = MSG_ID_GGA;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    memcpy(&buffer[5], utc_time, 3);
    memcpy(&buffer[8], &latitude, sizeof(float));
    buffer[12] = latitude_dir ? 1 : 0;
    memcpy(&buffer[13], &longitude, sizeof(float));
    buffer[17] = longitude_dir ? 1 : 0;
    buffer[18] = fix_status;
    put_uint16_le(&buffer[19], hdop);
    buffer[21] = satellites_used;
    put_uint16_le(&buffer[22], (uint16_t)altitude);
    // Add velocity and heading fields as per spec (using defaults for now)
    
    // CRC16 will be added by sub-packet handler
    return 24; // Updated size with all fields
}

size_t build_gsv_packet(uint8_t *buffer, uint8_t satellites_in_view, satellite_info_t *satellite_data, uint16_t satellite_count) {
    buffer[0] = MSG_ID_GSV;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    buffer[5] = satellites_in_view;
    
    // Add SNR and Signal ID fields as per spec
    uint16_t average_snr = 0;
    if (satellite_count > 0 && satellite_data) {
        int total_snr = 0;
        for (int i = 0; i < satellite_count; i++) {
            total_snr += satellite_data[i].snr;
        }
        average_snr = total_snr / satellite_count;
    }
    put_uint16_le(&buffer[6], average_snr);
    put_uint16_le(&buffer[8], 0x0001); // Default signal ID
    
    // CRC16 will be added by sub-packet handler
    return 10; // MSG_ID + packet_id(4) + satellites_in_view + snr + signal_id
}

size_t build_pwm_packet(uint8_t *buffer, uint16_t servo_pwm[3], uint16_t motor_rpm_x, uint16_t motor_rpm_y, uint16_t esc_pwm[2]) {
    buffer[0] = MSG_ID_PWM;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    
    for (int i = 0; i < 3; i++) {
        put_uint16_le(&buffer[3 + i * 2], servo_pwm[i]);
    }
    put_uint16_le(&buffer[9], motor_rpm_x);
    put_uint16_le(&buffer[11], motor_rpm_y);
    for (int i = 0; i < 2; i++) {
        put_uint16_le(&buffer[13 + i * 2], esc_pwm[i]);
    }
    // CRC16 will be added by sub-packet handler
    // MSG_ID + packet_id(4) + 3*servo + 4bytes + 2*esc
    return 19;
}

size_t build_signal_packet(uint8_t *buffer, uint16_t lora_rssi, uint16_t lora_snr, uint16_t xbee_rssi, uint16_t xbee_snr, uint8_t link_quality) {
    buffer[0] = MSG_ID_SIGNAL;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    
    // Add all signal fields as per spec
    put_uint16_le(&buffer[5], lora_rssi);  // lora_rssi
    put_uint16_le(&buffer[7], lora_snr);             // lora_snr (default)
    put_uint16_le(&buffer[9], xbee_rssi);  // xbee_rssi (same as lora for now)
    put_uint16_le(&buffer[11], xbee_snr);             // xbee_snr (default)
    buffer[13] = link_quality;                            // link_quality
    
    // CRC16 will be added by sub-packet handler
    return 14; // MSG_ID + packet_id(4) + 4*int16 + 1*uint8
}

size_t build_internal_system_packet(uint8_t *buffer, uint16_t cpu_core_0_load, uint16_t cpu_core_1_load, uint8_t arm_status) {
    buffer[0] = MSG_ID_INTERNAL_SYSTEM;
    // Add 4-byte packet ID field as per spec
    put_uint32_le(&buffer[1], g_packet_ctx.packets_sent);
    
    put_uint16_le(&buffer[5], cpu_core_0_load);
    put_uint16_le(&buffer[7], cpu_core_1_load);
    buffer[9] = arm_status;
    
    // CRC16 will be added by sub-packet handler
    return 10; // MSG_ID + packet_id(4) + 2*uint16 + 1*uint8
}

int add_sub_packet(uint8_t *payload_buffer, int payload_offset, size_t max_payload_size, const uint8_t *sub_data, size_t sub_data_len) {
    // Calculate required space: ID(1) + Length(1) + Data(n) + CRC16(2)
    size_t required_space = 4 + sub_data_len;
    if (payload_offset + required_space > max_payload_size) {
        ESP_LOGE(TAG, "Not enough space for sub-packet");
        return -1;
    }
    
    // Add sub-packet header
    //payload_buffer[payload_offset++] = sub_packet_id;
    //payload_buffer[payload_offset++] = (uint8_t)sub_data_len;
    
    // Add sub-packet data
    memcpy(&payload_buffer[payload_offset], sub_data, sub_data_len);
    payload_offset += sub_data_len;
    
    // Calculate and add CRC16 for this sub-packet
    uint16_t sub_crc = calculate_crc16(&payload_buffer[payload_offset - sub_data_len], sub_data_len);
    payload_buffer[payload_offset++] = (sub_crc >> 8) & 0xFF;
    payload_buffer[payload_offset++] = sub_crc & 0xFF;
    
    return payload_offset;
}

int create_packet(packet_type_t packet_type, const uint8_t *payload, size_t payload_len, 
                  uint8_t *output_buffer, size_t output_size, uint8_t flags) {
    if (!payload || !output_buffer || payload_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Invalid packet parameters");
        return -1;
    }
    
    // We need room for: start byte + payload + CRC32 (4 bytes)
    size_t total_size = 1 + payload_len + 4; 
    if (total_size > output_size) {
        ESP_LOGE(TAG, "Output buffer too small: need %d, have %d", total_size, output_size);
        return -1;
    }
    
    size_t offset = 0;
    
    // Keep the start byte
    output_buffer[offset++] = PACKET_START_BYTE;
    
    // Add payload directly after start byte
    memcpy(&output_buffer[offset], payload, payload_len);
    offset += payload_len;
    
    // Calculate CRC32 over payload only
    uint32_t crc = calculate_crc32(payload, payload_len);
    output_buffer[offset++] = (crc >> 24) & 0xFF;
    output_buffer[offset++] = (crc >> 16) & 0xFF;
    output_buffer[offset++] = (crc >> 8) & 0xFF;
    output_buffer[offset++] = crc & 0xFF;
    
    g_packet_ctx.packets_sent++;
    return offset;
}

esp_err_t update_previous_sensor_data(const SensorData *current_data) {
    if (!current_data || !g_differential_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update IMU data
    g_previous_data.ax = current_data->ax;
    g_previous_data.ay = current_data->ay;
    g_previous_data.az = current_data->az;
    g_previous_data.gx = current_data->gx;
    g_previous_data.gy = current_data->gy;
    g_previous_data.gz = current_data->gz;
    g_previous_data.mx = current_data->mx;
    g_previous_data.my = current_data->my;
    g_previous_data.mz = current_data->mz;
    
    // Update environmental data
    g_previous_data.temperature = current_data->temperature;
    g_previous_data.pressure = current_data->pressure;
    g_previous_data.humidity = current_data->humidity;
    g_previous_data.altitude = current_data->altitude;
    g_previous_data.gas_level = current_data->gas_level;
    
    // Update power data
    g_previous_data.voltage = current_data->voltage;
    
    // Update GPS data
    g_previous_data.latitude = current_data->latitude;
    g_previous_data.longitude = current_data->longitude;
    
    // Update servo positions
    paraglider_get_servo_positions(&g_previous_data.left_servo_us, &g_previous_data.right_servo_us);
    
    // Update mission phase
    extern FlightState currentState;
    g_previous_data.mission_phase = (uint8_t)currentState;
    
    return ESP_OK;
}

esp_err_t send_differential_telemetry_burst(const SensorData *current_data) {
    if (!current_data || !g_differential_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    int packet_size = create_differential_telemetry_packet(current_data, packet_buffer, 
                                                          sizeof(packet_buffer), false);
    
    if (packet_size > 0) {
        esp_err_t result = send_packet(packet_buffer, packet_size);
        if (result == ESP_OK) {
            update_previous_sensor_data(current_data);
        }
        return result;
    }
    
    return ESP_ERR_INVALID_SIZE;
}

// Function pointer for send_packet to allow overriding
esp_err_t (*send_packet_ptr)(const uint8_t*, size_t) = NULL;

esp_err_t send_packet(const uint8_t *packet_buffer, size_t packet_size) {
    if (!packet_buffer || packet_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // If function pointer is set, use it
    if (send_packet_ptr) {
        return send_packet_ptr(packet_buffer, packet_size);
    }
    
    // Default implementation
    int bytes_written = uart_write_bytes(UART_NUM_0, packet_buffer, packet_size);
    if (bytes_written == packet_size) {
        ESP_LOGD(TAG, "Sent packet: %d bytes", packet_size);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send packet: wrote %d of %d bytes", bytes_written, packet_size);
        return ESP_ERR_INVALID_RESPONSE;
    }
}

esp_err_t get_packet_stats(packet_context_t *context) {
    if (!context) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(context, &g_packet_ctx, sizeof(packet_context_t));
    return ESP_OK;
}

esp_err_t get_differential_stats(uint32_t *total_packets, uint32_t *differential_packets, 
                                uint32_t *full_packets, uint32_t *bytes_saved) {
    if (total_packets) *total_packets = g_differential_stats_total;
    if (differential_packets) *differential_packets = g_differential_stats_partial;
    if (full_packets) *full_packets = g_differential_stats_full;
    if (bytes_saved) *bytes_saved = g_differential_stats_bytes_saved;
    
    return ESP_OK;
}

esp_err_t send_telemetry_burst(void) {
    // This function would typically get current sensor data and send it
    // For now, create a dummy implementation
    SensorData dummy_data = {0};
    return send_differential_telemetry_burst(&dummy_data);
}

esp_err_t send_heartbeat_packet(void) {
    uint8_t heartbeat_payload[] = {0x01, 0x02, 0x03, 0x04}; // Simple heartbeat data
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    
    int packet_size = create_packet(PACKET_TYPE_HEARTBEAT, heartbeat_payload, 
                                   sizeof(heartbeat_payload), packet_buffer, 
                                   sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size > 0) {
        return send_packet(packet_buffer, packet_size);
    }
    
    return ESP_ERR_INVALID_SIZE;
}

esp_err_t send_ack_packet(uint8_t sequence_number) {
    uint8_t ack_payload[] = {sequence_number};
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    
    int packet_size = create_packet(PACKET_TYPE_ACK, ack_payload, sizeof(ack_payload), 
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size > 0) {
        return send_packet(packet_buffer, packet_size);
    }
    
    return ESP_ERR_INVALID_SIZE;
}

esp_err_t send_nack_packet(uint8_t sequence_number, uint8_t error_code) {
    uint8_t nack_payload[] = {sequence_number, error_code};
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    
    int packet_size = create_packet(PACKET_TYPE_NACK, nack_payload, sizeof(nack_payload), 
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_NONE);
    
    if (packet_size > 0) {
        return send_packet(packet_buffer, packet_size);
    }
    
    return ESP_ERR_INVALID_SIZE;
}

// Individual send packet functions
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
    uint8_t buffer[16];
    size_t packet_size = build_accel_packet(buffer, x_accel, y_accel, z_accel);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gyro_packet(int16_t x_rate, int16_t y_rate, int16_t z_rate) {
    uint8_t buffer[16];
    size_t packet_size = build_gyro_packet(buffer, x_rate, y_rate, z_rate);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gga_packet(uint8_t utc_time[3], float latitude, bool latitude_dir, float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, uint8_t satellites_used, int32_t altitude) {
    uint8_t buffer[32];
    size_t packet_size = build_gga_packet(buffer, utc_time, latitude, latitude_dir, longitude, longitude_dir, fix_status, hdop, satellites_used, altitude);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}

void send_gsv_packet(uint8_t satellites_in_view, satellite_info_t *satellite_data, u_int16_t satellite_count) {
    uint8_t buffer[256];
    size_t packet_size = build_gsv_packet(buffer, satellites_in_view, satellite_data, satellite_count);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}
/*
void send_battery_packet(uint16_t current_voltage) {
    uint8_t buffer[16];
    size_t packet_size = build_battery_packet(buffer, current_voltage);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}
*/
void send_system_status_packet(uint8_t sensor_status, uint32_t uptime, uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]) {
    uint8_t buffer[64];
    size_t packet_size = build_system_status_packet(buffer, sensor_status, uptime, mission_phase, reboot_reason, system_time);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}
/*
void send_pwm_packet(uint16_t servo_pwm[3], uint16_t motor_rpm_x, uint16_t motor_rpm_y, uint16_t esc_pwm[2]) {
    uint8_t buffer[32];
    size_t packet_size = build_pwm_packet(buffer, servo_pwm, motor_rpm_x, motor_rpm_y, esc_pwm);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}
*/
/*
void send_signal_packet(int16_t rssi, uint8_t lq) {
    uint8_t buffer[16];
    size_t packet_size = build_signal_packet(buffer, rssi, lq);
    uart_write_bytes(UART_NUM_0, buffer, packet_size);
}
*/
void send_combined_packets() {
    // Implementation would collect current sensor data and send differential packet
    SensorData current_data = {0};
    send_differential_telemetry_burst(&current_data);
}

// Basic communication parser (placeholder implementation)
void comms_parser(const uint8_t *data, uint16_t length) {
    if (!data || length == 0) return;
    
    packet_type_t packet_type;
    const uint8_t *payload;
    size_t payload_len;
    
    esp_err_t result = parse_packet(data, length, &packet_type, &payload, &payload_len);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Parsed packet type: %d, payload length: %d", packet_type, payload_len);
        // Handle different packet types here
        switch (packet_type) {
            case PACKET_TYPE_COMMAND:
                // Handle command packets
                break;
            case PACKET_TYPE_ACK:
                // Handle acknowledgment packets
                break;
            case PACKET_TYPE_NACK:
                // Handle negative acknowledgment packets
                break;
            default:
                ESP_LOGW(TAG, "Unknown packet type: %d", packet_type);
                break;
        }
    } else {
        ESP_LOGE(TAG, "Failed to parse packet: %d", result);
    }
}