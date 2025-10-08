#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h" // Add this include

// ---- Config (edit to match your wiring) ----
#define GPS_UART_NUM      UART_NUM_1
#define GPS_UART_BAUD     9600
#define GPS_RX_PIN        GPIO_NUM_17   // GPS TX -> ESP RX
#define GPS_TX_PIN        GPIO_NUM_16   // optional (not used by most GPS modules)
#define GPS_RX_BUF_BYTES  1024
#define MAX_SATS 32

// Satellite information structure - match the forward declaration in communications.h
struct satellite_info {
    int prn;
    int elevation;
    int azimuth;
    uint16_t snr;
};

// Latest GNSS fix, scaled for easy packing - match the forward declaration in communications.h
struct gps_fix {
    bool    valid;          // true if fix_quality > 0
    uint8_t hh, mm, ss;     // UTC time
    uint16_t ms;            // milliseconds (from .ss)
    int32_t lat_e7;         // latitude  degrees * 1e7
    int32_t lon_e7;         // longitude degrees * 1e7
    int32_t alt_cm;         // altitude  meters * 100
    uint16_t hdop_x100;     // HDOP * 100
    uint8_t sats;           // satellites in use
    uint8_t fix_quality;    // 0=no fix, 1=GPS, 2=DGPS/RTK, etc
};

// Compatibility typedef for main.c
typedef struct {
    uint8_t utc_time[3];
    float latitude;
    bool latitude_dir;
    float longitude;
    bool longitude_dir;
    uint8_t fix_status;
    uint16_t hdop;
    uint8_t satellites_used;
    int32_t altitude;
} GPSData;

// Task that initializes UART and continuously parses NMEA sentences
void gps_task(void *pvParameters);

// Copy latest fix (returns true if valid; out still filled even if false)
bool gps_get_fix(struct gps_fix *out);

// Get satellite information
int gps_get_satellites(struct satellite_info *out, int max);

// Compatibility function for main.c
esp_err_t gps_get_data(GPSData *data);

// Initialize GPS (for compatibility)
void gps_init(void);
