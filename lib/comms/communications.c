#include "communications.h"
#include "gps.h" // Include GPS header for satellite_info_t
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "driver/uart.h" // Include your UART driver header

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

    return index; // Total bytes written
}

size_t format_environmental_packet(uint8_t *buffer, EnvironmentalPacket *data) {
    if (!buffer || !data) return 0;

    size_t index = 0;
    buffer[index++] = data->message_id;

    buffer[index++] = data->altitude >> 8;
    buffer[index++] = data->altitude & 0xFF;

    buffer[index++] = data->temperature >> 8;
    buffer[index++] = data->temperature & 0xFF;

    buffer[index++] = data->pressure >> 8;
    buffer[index++] = data->pressure & 0xFF;

    buffer[index++] = data->humidity >> 8;
    buffer[index++] = data->humidity & 0xFF;

    buffer[index++] = data->gas_sensor >> 8;
    buffer[index++] = data->gas_sensor & 0xFF;

    return index; // Should return 11
}

size_t build_accel_packet(uint8_t *buffer, int16_t x_accel, int16_t y_accel, int16_t z_accel) {
    buffer[0] = MSG_ID_ACCEL;
    buffer[1] = (uint8_t)(x_accel >> 8);
    buffer[2] = (uint8_t)(x_accel & 0xFF);
    buffer[3] = (uint8_t)(y_accel >> 8);
    buffer[4] = (uint8_t)(y_accel & 0xFF);
    buffer[5] = (uint8_t)(z_accel >> 8);
    buffer[6] = (uint8_t)(z_accel & 0xFF);
    return ACCEL_PACKET_SIZE;
}

size_t build_gyro_packet(uint8_t *buffer, int16_t x_rate, int16_t y_rate, int16_t z_rate) {
    buffer[0] = MSG_ID_GYRO;
    buffer[1] = (uint8_t)(x_rate >> 8);
    buffer[2] = (uint8_t)(x_rate & 0xFF);
    buffer[3] = (uint8_t)(y_rate >> 8);
    buffer[4] = (uint8_t)(y_rate & 0xFF);
    buffer[5] = (uint8_t)(z_rate >> 8);
    buffer[6] = (uint8_t)(z_rate & 0xFF);
    return 7;
}

size_t build_gga_packet(uint8_t *buffer, uint8_t utc_time[3], float latitude, bool latitude_dir, float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, uint8_t satellites_used, int32_t altitude) {
    buffer[0] = 0xE7; // Message ID for GGA

    // UTC Time (HHMMSS.ss)
    memcpy(&buffer[1], utc_time, 3);

    // Latitude
    memcpy(&buffer[4], &latitude, sizeof(float));

    // Latitude Direction
    buffer[8] = latitude_dir ? 1 : 0;

    // Longitude
    memcpy(&buffer[9], &longitude, sizeof(float));

    // Longitude Direction
    buffer[13] = longitude_dir ? 1 : 0;

    // Fix Status
    buffer[14] = fix_status;

    // HDOP
    put_uint16_le(&buffer[15], hdop);

    // Satellites Used
    buffer[17] = satellites_used;

    // Altitude
    put_uint32_le(&buffer[18], altitude);

    return 22; // Total bytes written
}

size_t build_gsv_packet(uint8_t *buffer, uint8_t satellites_in_view, satellite_info_t *satellite_data, int satellite_count) {
    buffer[0] = 0xAB; // Message ID for GSV

    // Satellites in View
    buffer[1] = satellites_in_view;

    // Add satellite information
    size_t index = 2;
    int max_satellites = (satellite_count < MAX_SATS) ? satellite_count : MAX_SATS;
    for (int i = 0; i < max_satellites; i++) {
        buffer[index++] = (uint8_t)satellite_data[i].prn;
        buffer[index++] = (uint8_t)satellite_data[i].elevation;
        buffer[index++] = (uint8_t)satellite_data[i].azimuth;
        buffer[index++] = (uint8_t)satellite_data[i].snr;
    }

    return index; // Total bytes written
}

size_t build_battery_packet(uint8_t *buffer, uint16_t current_voltage) {
    buffer[0] = 0x8B; // Message ID for Battery

    // Current Voltage
    put_uint16_le(&buffer[1], current_voltage);

    return 3; // Total bytes written
}

size_t build_system_status_packet(uint8_t *buffer, uint8_t sensor_status, uint32_t uptime, uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]) {
    buffer[0] = 0xE2; // Message ID for System Status

    // Sensor Operational Status
    buffer[1] = sensor_status;

    // Uptime
    put_uint32_le(&buffer[2], uptime);

    // Flight Software Current Mission
    buffer[6] = mission_phase;

    // Last Reboot Reason
    size_t reboot_reason_len = strlen(reboot_reason);
    memcpy(&buffer[7], reboot_reason, reboot_reason_len);

    // System Time (HHMMSS.ss)
    memcpy(&buffer[7 + reboot_reason_len], system_time, 3);

    return 10 + reboot_reason_len; // Total bytes written
}

size_t build_pwm_packet(uint8_t *buffer, uint16_t servo_pwm[5], uint16_t esc_pwm[2]) {
    buffer[0] = 0x99; // Message ID for PWM

    // Servo PWM values
    for (int i = 0; i < 5; i++) {
        put_uint16_le(&buffer[1 + i * 2], servo_pwm[i]);
    }

    // ESC PWM values
    for (int i = 0; i < 2; i++) {
        put_uint16_le(&buffer[11 + i * 2], esc_pwm[i]);
    }

    return 15; // Total bytes written
}

size_t build_signal_packet(uint8_t *buffer, int16_t rssi, uint8_t lq) {
    buffer[0] = 0xA5; // Message ID for Signal

    // RSSI
    put_uint16_le(&buffer[1], rssi);

    // Link Quality
    buffer[3] = lq;

    return 4; // Total bytes written
}

// Stub parser
void comms_parser(const uint8_t *data, uint16_t length) {
    // Implement later
}

void send_combined_packets() {
    uint8_t buffer[COMM_BUFFER_SIZE]; // Combined buffer
    size_t index = 0;

    // Add a start byte for framing
    buffer[index++] = 0x7E;

    // Temporary packet buffers
    uint8_t temp_packet[64];

    // Build and append Identification packet
    IdentificationPacket id_packet = {
        .team_id = "TEAM1234",
        .timestamp = 123456789,
        .packet_id = 1,
        .source = 0x01,
        .destination = 0x02
    };
    size_t id_size = build_identification_packet(temp_packet, &id_packet);
    memcpy(&buffer[index], temp_packet, id_size);
    index += id_size;

    // Build and append Environmental packet
    EnvironmentalPacket env_packet = {
        .message_id = MSG_ID_ENVIRONMENTAL,
        .altitude = 1000,
        .temperature = 2500,
        .pressure = 101325,
        .humidity = 5000,
        .gas_sensor = 300
    };
    size_t env_size = format_environmental_packet(temp_packet, &env_packet);
    memcpy(&buffer[index], temp_packet, env_size);
    index += env_size;

    // Build and append Accel packet
    size_t accel_size = build_accel_packet(temp_packet, 100, 200, -300);
    memcpy(&buffer[index], temp_packet, accel_size);
    index += accel_size;

    // Build and append Gyro packet
    size_t gyro_size = build_gyro_packet(temp_packet, 400, -500, 600);
    memcpy(&buffer[index], temp_packet, gyro_size);
    index += gyro_size;

    // Build and append GGA packet
    uint8_t utc_time[3] = {0x12, 0x34, 0x56}; // Example UTC time
    size_t gga_size = build_gga_packet(temp_packet, utc_time, 37.7749, true, -122.4194, false, 1, 50, 8, 100);
    memcpy(&buffer[index], temp_packet, gga_size);
    index += gga_size;

    // Build and append GSV packet
    size_t gsv_size = build_gsv_packet(temp_packet, 10, 45, 123);
    memcpy(&buffer[index], temp_packet, gsv_size);
    index += gsv_size;

    // Build and append Battery packet
    size_t battery_size = build_battery_packet(temp_packet, 3700);
    memcpy(&buffer[index], temp_packet, battery_size);
    index += battery_size;

    // Build and append System Status packet
    const char *reboot_reason = "Power Cycle";
    uint8_t system_time[3] = {0x12, 0x34, 0x56};
    size_t system_status_size = build_system_status_packet(temp_packet, 1, 123456, 2, reboot_reason, system_time);
    memcpy(&buffer[index], temp_packet, system_status_size);
    index += system_status_size;

    // Build and append PWM packet
    uint16_t servo_pwm[5] = {1500, 1500, 1500, 1500, 1500};
    uint16_t esc_pwm[2] = {1000, 1000};
    size_t pwm_size = build_pwm_packet(temp_packet, servo_pwm, esc_pwm);
    memcpy(&buffer[index], temp_packet, pwm_size);
    index += pwm_size;

    // Build and append Signal packet
    size_t signal_size = build_signal_packet(temp_packet, -70, 90);
    memcpy(&buffer[index], temp_packet, signal_size);
    index += signal_size;

    // Add total length to the header
    buffer[1] = (uint8_t)(index - 2); // Total length excluding start byte and length byte

    // Send the combined buffer over UART
    uart_write(buffer, index);
}
