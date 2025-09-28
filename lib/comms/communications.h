#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Forward declarations and includes
typedef struct gps_fix gps_fix_t;
typedef struct paraglider_nav paraglider_nav_t;

// Flight state enum (matching main.c)
typedef enum {
    PRE_LAUNCH,
    ASCENT,
    APOGEE,
    DESCENT,
    LANDING
} FlightState;

// Sensor data structure (matching main.c)
typedef struct {
    float roll, pitch, yaw;  
    float ax, ay, az;        // Acceleration    
    float gx, gy, gz;        // Gyroscope
    float mx, my, mz;        // Magnetometer
    float altitude, pressure; // Barometer
    float voltage;           // Voltage
    float latitude, longitude; // GPS
    float gas_level;         // Gas sensor reading
    float temperature;  
    float humidity;
} SensorData;

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

#define MSG_ID_COMMAND 0xCC
#define MSG_ID_ACTIVE_GUIDE 0xDD
#define MSG_ID_DEPLOY_PARACHUTE 0xEE
#define MSG_ID_DEPLOY_PARAGLIDER 0xFF
#define MSG_ID_PREFLIGHT_CHECK 0xBA
#define MSG_ID_CALIBRATE_SENSORS 0xBB
#define MSG_ID_SET_MISSION_PHASE 0xBC
#define MSG_ID_START_TELEMETRY 0xBD
#define MSG_ID_SET_HOME 0xBE

// Constants
#define TEAM_ID_MAX_LEN 16
#define COMM_BUFFER_SIZE 512  // Increased buffer size for proper packetization
#define ACCEL_PACKET_SIZE 7
#define GYRO_PACKET_SIZE   7
#define MAX_SATS 32

// Packetization constants
#define PACKET_START_BYTE 0x7E
#define PACKET_END_BYTE 0x7F
#define CRC_SIZE 2
#define MAX_PACKET_SIZE 256
#define MAX_PAYLOAD_SIZE (MAX_PACKET_SIZE - 10) // Reserve space for headers, CRC, etc.

// Differential packetization constants
#define CHANGE_THRESHOLD_ACCEL 0.1f     // g
#define CHANGE_THRESHOLD_GYRO 1.0f      // deg/s
#define CHANGE_THRESHOLD_MAG 5.0f       // µT
#define CHANGE_THRESHOLD_TEMP 0.5f      // °C
#define CHANGE_THRESHOLD_PRESSURE 10.0f // Pa
#define CHANGE_THRESHOLD_ALTITUDE 1.0f  // m
#define CHANGE_THRESHOLD_HUMIDITY 2.0f  // %
#define CHANGE_THRESHOLD_VOLTAGE 0.1f   // V
#define CHANGE_THRESHOLD_GAS 10.0f      // units
#define CHANGE_THRESHOLD_GPS_POS 0.000001f // degrees (about 0.1m)

// Packet frame structure
typedef struct {
    uint8_t start_byte;      // 0x7E
    uint8_t packet_type;     // Packet type identifier
    uint16_t payload_length; // Length of payload in bytes
    uint8_t sequence_number; // Packet sequence number
    uint8_t flags;           // Control flags
    uint8_t payload[];       // Variable length payload
    // CRC16 follows payload (2 bytes)
    // End byte 0x7F follows CRC
} __attribute__((packed)) packet_frame_t;

// Packet types
typedef enum {
    PACKET_TYPE_TELEMETRY = 0x01,
    PACKET_TYPE_COMMAND = 0x02,
    PACKET_TYPE_ACK = 0x03,
    PACKET_TYPE_NACK = 0x04,
    PACKET_TYPE_HEARTBEAT = 0x05,
    PACKET_TYPE_COMBINED = 0x06
} packet_type_t;

// Packet flags
#define PACKET_FLAG_NONE 0x00
#define PACKET_FLAG_RELIABLE 0x01
#define PACKET_FLAG_COMPRESSED 0x02
#define PACKET_FLAG_ENCRYPTED 0x04
#define PACKET_FLAG_FRAGMENTED 0x08

// Sub-packet structure (for individual sensor data within combined packets)
typedef struct {
    uint8_t sub_packet_id;   // Message ID (e.g., MSG_ID_ENVIRONMENTAL)
    uint8_t sub_packet_len;  // Length of sub-packet data
    uint8_t data[];          // Sub-packet data
    // CRC8 follows data (1 byte)
} __attribute__((packed)) sub_packet_t;

// Packetization context
typedef struct {
    uint8_t sequence_counter;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t crc_errors;
    uint32_t frame_errors;
} packet_context_t;

// Previous sensor values for change detection
typedef struct {
    // IMU data
    float ax, ay, az;           // Accelerometer
    float gx, gy, gz;           // Gyroscope  
    float mx, my, mz;           // Magnetometer
    
    // Environmental data
    float temperature;
    float pressure;
    float humidity;
    float altitude;
    float gas_level;
    
    // Power data
    float voltage;
    
    // GPS data
    double latitude, longitude;
    float gps_altitude;
    uint8_t fix_quality;
    uint8_t satellites_used;
    
    // System data
    uint8_t mission_phase;
    
    // Servo data
    uint16_t left_servo_us;
    uint16_t right_servo_us;
    
    // Packet metadata
    uint32_t last_full_packet_time;
    uint32_t packet_count;
} previous_sensor_data_t;

// Change detection flags
typedef struct {
    bool identification;    // Always true
    bool environmental;
    bool accelerometer;
    bool gyroscope;
    bool magnetometer;
    bool gps_gga;
    bool gps_gsv;
    bool battery;
    bool system_status;
    bool pwm;
    bool signal;
} sensor_change_flags_t;

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

// Individual send function declarations
void send_identification_packet(const IdentificationPacket *packet);
void send_environmental_packet(const EnvironmentalPacket *env_data);
void send_accel_packet(int16_t x_accel, int16_t y_accel, int16_t z_accel);
void send_gyro_packet(int16_t x_rate, int16_t y_rate, int16_t z_rate);
void send_gga_packet(uint8_t utc_time[3], float latitude, bool latitude_dir, float longitude, bool longitude_dir, uint8_t fix_status, uint16_t hdop, uint8_t satellites_used, int32_t altitude);
void send_gsv_packet(uint8_t satellites_in_view, satellite_info_t *satellite_data, int satellite_count);
void send_battery_packet(uint16_t current_voltage);
void send_system_status_packet(uint8_t sensor_status, uint32_t uptime, uint8_t mission_phase, const char *reboot_reason, uint8_t system_time[3]);
void send_pwm_packet(uint16_t servo_pwm[5], uint16_t esc_pwm[2]);
void send_signal_packet(int16_t rssi, uint8_t lq);

void send_combined_packets();

// New packetization function declarations

/**
 * Calculate CRC16 for packet integrity
 * @param data Pointer to data
 * @param length Length of data
 * @return CRC16 value
 */
uint16_t calculate_crc16(const uint8_t *data, size_t length);

/**
 * Calculate CRC8 for sub-packet integrity
 * @param data Pointer to data
 * @param length Length of data
 * @return CRC8 value
 */
uint8_t calculate_crc8(const uint8_t *data, size_t length);

/**
 * Initialize packetization system
 * @return ESP_OK on success
 */
esp_err_t packet_system_init(void);

/**
 * Create a properly formatted packet with headers and CRC
 * @param packet_type Type of packet
 * @param payload Payload data
 * @param payload_len Length of payload
 * @param output_buffer Output buffer for complete packet
 * @param output_size Size of output buffer
 * @param flags Packet flags
 * @return Size of complete packet, or -1 on error
 */
int create_packet(packet_type_t packet_type, const uint8_t *payload, size_t payload_len, 
                  uint8_t *output_buffer, size_t output_size, uint8_t flags);

/**
 * Parse and validate received packet
 * @param buffer Received data buffer
 * @param buffer_len Length of received data
 * @param packet_type Output packet type
 * @param payload Output payload pointer
 * @param payload_len Output payload length
 * @return ESP_OK if packet is valid, error code otherwise
 */
esp_err_t parse_packet(const uint8_t *buffer, size_t buffer_len, 
                       packet_type_t *packet_type, const uint8_t **payload, size_t *payload_len);

/**
 * Create a combined telemetry packet with multiple sub-packets
 * @param output_buffer Output buffer for complete packet
 * @param output_size Size of output buffer
 * @return Size of complete packet, or -1 on error
 */
int create_combined_telemetry_packet(uint8_t *output_buffer, size_t output_size);

/**
 * Add sub-packet to combined packet payload
 * @param payload_buffer Payload buffer being built
 * @param payload_offset Current offset in payload buffer
 * @param max_payload_size Maximum payload size
 * @param sub_packet_id Sub-packet ID
 * @param sub_data Sub-packet data
 * @param sub_data_len Length of sub-packet data
 * @return New payload offset, or -1 on error
 */
int add_sub_packet(uint8_t *payload_buffer, int payload_offset, size_t max_payload_size,
                   uint8_t sub_packet_id, const uint8_t *sub_data, size_t sub_data_len);

/**
 * Send packet with proper framing and error handling
 * @param packet_buffer Complete packet buffer
 * @param packet_size Size of packet
 * @return ESP_OK on success
 */
esp_err_t send_packet(const uint8_t *packet_buffer, size_t packet_size);

/**
 * Get packetization statistics
 * @param context Output context structure
 * @return ESP_OK on success
 */
esp_err_t get_packet_stats(packet_context_t *context);

// Enhanced telemetry functions
esp_err_t send_telemetry_burst(void);
esp_err_t send_heartbeat_packet(void);
esp_err_t send_ack_packet(uint8_t sequence_number);
esp_err_t send_nack_packet(uint8_t sequence_number, uint8_t error_code);

// New differential packetization function declarations

/**
 * Initialize differential packetization system
 * @return ESP_OK on success
 */
esp_err_t differential_packet_init(void);

/**
 * Detect which sensor values have changed significantly
 * @param current_data Current sensor readings
 * @param change_flags Output flags indicating which sensors changed
 * @return ESP_OK on success
 */
esp_err_t detect_sensor_changes(const SensorData *current_data, sensor_change_flags_t *change_flags);

/**
 * Create differential telemetry packet with only changed sub-packets
 * @param current_data Current sensor readings
 * @param output_buffer Output buffer for complete packet
 * @param output_size Size of output buffer
 * @param force_full_packet Force sending all sub-packets
 * @return Size of complete packet, or -1 on error
 */
int create_differential_telemetry_packet(const SensorData *current_data, 
                                        uint8_t *output_buffer, size_t output_size, 
                                        bool force_full_packet);

/**
 * Update previous sensor values after successful packet transmission
 * @param current_data Current sensor readings that were sent
 * @return ESP_OK on success
 */
esp_err_t update_previous_sensor_data(const SensorData *current_data);

/**
 * Get differential packetization statistics
 * @param total_packets Output total packets sent
 * @param differential_packets Output differential (partial) packets sent
 * @param full_packets Output full packets sent
 * @param bytes_saved Output estimated bytes saved
 * @return ESP_OK on success
 */
esp_err_t get_differential_stats(uint32_t *total_packets, uint32_t *differential_packets, 
                                uint32_t *full_packets, uint32_t *bytes_saved);

/**
 * Force next packet to be a full packet (send all sub-packets)
 * Useful for initialization or periodic full updates
 */
void force_next_full_packet(void);

/**
 * Check if it's time to send a full packet
 * Full packets are sent periodically even if no changes detected
 * @return true if full packet should be sent
 */
bool should_send_full_packet(void);

/**
 * Send differential telemetry burst with only changed sensor data
 * @param current_data Current sensor readings
 * @return ESP_OK on success
 */
esp_err_t send_differential_telemetry_burst(const SensorData *current_data);

#endif // COMMUNICATIONS_H
