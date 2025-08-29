#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <stdint.h>
#include <stdbool.h>

// Message IDs
#define MSG_ID_IDENTIFICATION 0x01
#define MSG_ID_ENVIRONMENTAL 0xE5
#define MSG_ID_ACCEL     0xA4
#define MSG_ID_GYRO      0x95
#define MSG_ID_GGA 0xE7
#define MSG_ID_GSV 0xAB
#define MSG_ID_BATTERY 0x8B
#define MSG_ID_SYSTEM_STATUS 0xE2
#define MSG_ID_PWM 0x99
#define MSG_ID_SIGNAL 0xA5

// Constants
#define TEAM_ID_MAX_LEN 16
#define COMM_BUFFER_SIZE 128
#define ACCEL_PACKET_SIZE 7
#define GYRO_PACKET_SIZE   7
#define MAX_SATS 32

// Structures
typedef struct {
    char team_id[TEAM_ID_MAX_LEN];
    uint32_t timestamp;
    uint16_t packet_id;
    uint8_t source;
    uint8_t destination;
} IdentificationPacket;

// Environmental packet struct
typedef struct {
    uint8_t message_id;     // 0xE5
    uint16_t altitude;      // in meters, scaled
    uint16_t temperature;   // in °C * 100 or appropriate scale
    uint16_t pressure;      // in Pa / 10 or similar
    uint16_t humidity;      // % * 100
    uint16_t gas_sensor;    // scaled value
} __attribute__((packed)) EnvironmentalPacket;

// Satellite information structure
typedef struct {
    int prn;
    int elevation;
    int azimuth;
    int snr;
} satellite_info_t;

size_t format_environmental_packet(uint8_t *buffer, EnvironmentalPacket *data);

// Function declarations
int build_identification_packet(uint8_t *buffer, const IdentificationPacket *packet);
void comms_parser(const uint8_t *data, uint16_t length);
size_t build_accel_packet(uint8_t *buffer, int16_t x_accel, int16_t y_accel, int16_t z_accel);
size_t build_gyro_packet(uint8_t *buffer, int16_t x_rate, int16_t y_rate, int16_t z_rate);
size_t build_gga_packet(uint8_t *buffer, uint8_t utc_time[3], float latitude, bool latitude_dir, float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, uint8_t satellites_used, int32_t altitude);
size_t build_gsv_packet(uint8_t *buffer, uint8_t satellites_in_view, satellite_info_t *satellite_data, int satellite_count);
size_t build_battery_packet(uint8_t *buffer, uint16_t current_voltage);
size_t build_system_status_packet(uint8_t *buffer, uint8_t sensor_status, uint32_t uptime, uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]);
size_t build_pwm_packet(uint8_t *buffer, uint16_t servo_pwm[5], uint16_t esc_pwm[2]);
size_t build_signal_packet(uint8_t *buffer, int16_t rssi, uint8_t lq);
void send_combined_packets();

#endif // COMMUNICATIONS_H
