// filepath: /home/ali/Documents/PlatformIO/Projects/CANSAT_FSW/lib/dshot_neo/DShotWrapper.cpp
#include "DShotWrapper.h"
#include "DShotRMT.h"

// Opaque struct to hold the C++ class instance
struct DShotHandle {
    DShotRMT* instance;
};

// Helper functions to convert between C and C++ enums
static dshot_mode_t convert_dshot_mode(dshot_mode_c_t mode) {
    switch(mode) {
        case DSHOT_OFF_C: return DSHOT_OFF;
        case DSHOT150_C: return DSHOT150;
        case DSHOT300_C: return DSHOT300;
        case DSHOT600_C: return DSHOT600;
        case DSHOT1200_C: return DSHOT1200;
        default: return DSHOT_OFF;
    }
}

static bidirectional_mode_t convert_bidirectional_mode(bidirectional_mode_c_t mode) {
    return mode == ENABLE_BIDIRECTION_C ? ENABLE_BIDIRECTION : NO_BIDIRECTION;
}

static telemetric_request_t convert_telemetric_request(telemetric_request_c_t request) {
    return request == ENABLE_TELEMETRIC_C ? ENABLE_TELEMETRIC : NO_TELEMETRIC;
}

static dshot_send_packet_exit_mode_c_t convert_send_exit_mode(dshot_send_packet_exit_mode_t mode) {
    return mode == SEND_SUCCESS ? SEND_SUCCESS_C : ERR_RMT_DISABLE_FAILURE_C;
}

static dshot_get_packet_exit_mode_c_t convert_get_exit_mode(dshot_get_packet_exit_mode_t mode) {
    switch(mode) {
        case DECODE_SUCCESS: return DECODE_SUCCESS_C;
        case ERR_EMPTY_QUEUE: return ERR_EMPTY_QUEUE_C;
        case ERR_NO_PACKETS: return ERR_NO_PACKETS_C;
        case ERR_CHECKSUM_FAIL: return ERR_CHECKSUM_FAIL_C;
        case ERR_BIDIRECTION_DISABLED: return ERR_BIDIRECTION_DISABLED_C;
        default: return ERR_NO_PACKETS_C;
    }
}

static extended_telem_type_t convert_telem_type(extended_telem_type_c_t type) {
    switch(type) {
        case TELEM_TYPE_ERPM_C: return TELEM_TYPE_ERPM;
        case TELEM_TYPE_TEMPRATURE_C: return TELEM_TYPE_TEMPRATURE;
        case TELEM_TYPE_VOLTAGE_C: return TELEM_TYPE_VOLTAGE;
        case TELEM_TYPE_CURRENT_C: return TELEM_TYPE_CURRENT;
        case TELEM_TYPE_DEBUG_A_C: return TELEM_TYPE_DEBUG_A;
        case TELEM_TYPE_DEBUG_B_C: return TELEM_TYPE_DEBUG_B;
        case TELEM_TYPE_STRESS_LEVEL_C: return TELEM_TYPE_STRESS_LEVEL;
        case TELEM_TYPE_STATUS_C: return TELEM_TYPE_STATUS;
        default: return TELEM_TYPE_ERPM;
    }
}

static extended_telem_type_c_t convert_telem_type_to_c(extended_telem_type_t type) {
    switch(type) {
        case TELEM_TYPE_ERPM: return TELEM_TYPE_ERPM_C;
        case TELEM_TYPE_TEMPRATURE: return TELEM_TYPE_TEMPRATURE_C;
        case TELEM_TYPE_VOLTAGE: return TELEM_TYPE_VOLTAGE_C;
        case TELEM_TYPE_CURRENT: return TELEM_TYPE_CURRENT_C;
        case TELEM_TYPE_DEBUG_A: return TELEM_TYPE_DEBUG_A_C;
        case TELEM_TYPE_DEBUG_B: return TELEM_TYPE_DEBUG_B_C;
        case TELEM_TYPE_STRESS_LEVEL: return TELEM_TYPE_STRESS_LEVEL_C;
        case TELEM_TYPE_STATUS: return TELEM_TYPE_STATUS_C;
        default: return TELEM_TYPE_ERPM_C;
    }
}

// Constructor/destructor equivalent functions
extern "C" dshot_handle_t dshot_create() {
    DShotHandle* handle = new DShotHandle();
    handle->instance = new DShotRMT();
    return handle;
}

extern "C" dshot_handle_t dshot_create_with_pin(uint8_t pin) {
    DShotHandle* handle = new DShotHandle();
    handle->instance = new DShotRMT(pin);
    return handle;
}

extern "C" void dshot_destroy(dshot_handle_t handle) {
    if (handle) {
        delete handle->instance;
        delete handle;
    }
}

// Begin functions
extern "C" void dshot_begin(dshot_handle_t handle, dshot_mode_c_t dshot_mode, bidirectional_mode_c_t is_bidirectional, uint16_t magnet_count) {
    if (handle && handle->instance) {
        handle->instance->begin(
            convert_dshot_mode(dshot_mode),
            convert_bidirectional_mode(is_bidirectional),
            magnet_count
        );
    }
}

extern "C" void dshot_begin_with_pin(dshot_handle_t handle, uint8_t pin, dshot_mode_c_t dshot_mode, bidirectional_mode_c_t is_bidirectional, uint16_t magnet_count) {
    if (handle && handle->instance) {
        handle->instance->begin(
            pin,
            convert_dshot_mode(dshot_mode),
            convert_bidirectional_mode(is_bidirectional),
            magnet_count
        );
    }
}

// Send DShot value functions
extern "C" dshot_send_packet_exit_mode_c_t dshot_send_value(dshot_handle_t handle, uint16_t throttle_value, telemetric_request_c_t telemetric_request) {
    if (handle && handle->instance) {
        dshot_send_packet_exit_mode_t result = handle->instance->send_dshot_value(
            throttle_value,
            convert_telemetric_request(telemetric_request)
        );
        return convert_send_exit_mode(result);
    }
    return ERR_RMT_DISABLE_FAILURE_C;
}

extern "C" void dshot_prepare_value(dshot_handle_t handle, uint16_t throttle_value, telemetric_request_c_t telemetric_request) {
    if (handle && handle->instance) {
        handle->instance->prepare_dshot_value(
            throttle_value,
            convert_telemetric_request(telemetric_request)
        );
    }
}

extern "C" dshot_send_packet_exit_mode_c_t dshot_send_last_value(dshot_handle_t handle) {
    if (handle && handle->instance) {
        dshot_send_packet_exit_mode_t result = handle->instance->send_last_value();
        return convert_send_exit_mode(result);
    }
    return ERR_RMT_DISABLE_FAILURE_C;
}

// Packet reading functions
extern "C" dshot_get_packet_exit_mode_c_t dshot_get_packet(dshot_handle_t handle, uint32_t* value, extended_telem_type_c_t* packet_type) {
    if (handle && handle->instance && value) {
        extended_telem_type_t cpp_packet_type;
        dshot_get_packet_exit_mode_t result = handle->instance->get_dshot_packet(value, packet_type ? &cpp_packet_type : nullptr);
        
        if (packet_type && result == DECODE_SUCCESS) {
            *packet_type = convert_telem_type_to_c(cpp_packet_type);
        }
        
        return convert_get_exit_mode(result);
    }
    return ERR_NO_PACKETS_C;
}

// Utility functions
extern "C" float dshot_convert_packet_to_volts(dshot_handle_t handle, uint8_t value) {
    if (handle && handle->instance) {
        return handle->instance->convert_packet_to_volts(value);
    }
    return 0.0f;
}

extern "C" float dshot_get_telem_success_rate(dshot_handle_t handle) {
    if (handle && handle->instance) {
        return handle->instance->get_telem_success_rate();
    }
    return 0.0f;
}