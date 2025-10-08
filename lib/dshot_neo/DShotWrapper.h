// filepath: /home/ali/Documents/PlatformIO/Projects/CANSAT_FSW/lib/dshot_neo/DShotWrapper.h
#ifndef DSHOT_WRAPPER_H
#define DSHOT_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// C-compatible enums that mirror the C++ ones
typedef enum {
    DSHOT_OFF_C = 0,
    DSHOT150_C,
    DSHOT300_C,
    DSHOT600_C,
    DSHOT1200_C
} dshot_mode_c_t;

typedef enum {
    NO_TELEMETRIC_C = 0,
    ENABLE_TELEMETRIC_C
} telemetric_request_c_t;

typedef enum {
    NO_BIDIRECTION_C = 0,
    ENABLE_BIDIRECTION_C
} bidirectional_mode_c_t;

typedef enum {
    SEND_SUCCESS_C = 0,
    ERR_RMT_DISABLE_FAILURE_C
} dshot_send_packet_exit_mode_c_t;

typedef enum {
    DECODE_SUCCESS_C = 0,
    ERR_EMPTY_QUEUE_C,
    ERR_NO_PACKETS_C,
    ERR_CHECKSUM_FAIL_C,
    ERR_BIDIRECTION_DISABLED_C
} dshot_get_packet_exit_mode_c_t;

typedef enum {
    TELEM_TYPE_ERPM_C = 0x1,
    TELEM_TYPE_TEMPRATURE_C = 0x2,
    TELEM_TYPE_VOLTAGE_C = 0x4,
    TELEM_TYPE_CURRENT_C = 0x6,
    TELEM_TYPE_DEBUG_A_C = 0x8,
    TELEM_TYPE_DEBUG_B_C = 0xA,
    TELEM_TYPE_STRESS_LEVEL_C = 0xC,
    TELEM_TYPE_STATUS_C = 0xE
} extended_telem_type_c_t;

// Opaque handle to represent a DShot instance
typedef struct DShotHandle* dshot_handle_t;

// Constructor/destructor equivalent functions
dshot_handle_t dshot_create(void);
dshot_handle_t dshot_create_with_pin(uint8_t pin);
void dshot_destroy(dshot_handle_t handle);

// Begin functions
void dshot_begin(dshot_handle_t handle, dshot_mode_c_t dshot_mode, bidirectional_mode_c_t is_bidirectional, uint16_t magnet_count);
void dshot_begin_with_pin(dshot_handle_t handle, uint8_t pin, dshot_mode_c_t dshot_mode, bidirectional_mode_c_t is_bidirectional, uint16_t magnet_count);

// Send DShot value functions
dshot_send_packet_exit_mode_c_t dshot_send_value(dshot_handle_t handle, uint16_t throttle_value, telemetric_request_c_t telemetric_request);
void dshot_prepare_value(dshot_handle_t handle, uint16_t throttle_value, telemetric_request_c_t telemetric_request);
dshot_send_packet_exit_mode_c_t dshot_send_last_value(dshot_handle_t handle);

// Packet reading functions
dshot_get_packet_exit_mode_c_t dshot_get_packet(dshot_handle_t handle, uint32_t* value, extended_telem_type_c_t* packet_type);

// Utility functions
float dshot_convert_packet_to_volts(dshot_handle_t handle, uint8_t value);
float dshot_get_telem_success_rate(dshot_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // DSHOT_WRAPPER_H