#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---- Config (edit to match your wiring) ----
#define GPS_UART_NUM      UART_NUM_1
#define GPS_UART_BAUD     9600
#define GPS_RX_PIN        GPIO_NUM_17   // GPS TX -> ESP RX
#define GPS_TX_PIN        GPIO_NUM_16   // optional (not used by most GPS modules)
#define GPS_RX_BUF_BYTES  1024

// Latest GNSS fix, scaled for easy packing
typedef struct {
    bool    valid;          // true if fix_quality > 0
    uint8_t hh, mm, ss;     // UTC time
    uint16_t ms;            // milliseconds (from .ss)

    int32_t lat_e7;         // latitude  degrees * 1e7
    int32_t lon_e7;         // longitude degrees * 1e7
    int32_t alt_cm;         // altitude  meters * 100

    uint16_t hdop_x100;     // HDOP * 100
    uint8_t sats;           // satellites in use
    uint8_t fix_quality;    // 0=no fix, 1=GPS, 2=DGPS/RTK, etc
} gps_fix_t;

// Task that initializes UART and continuously parses NMEA sentences
void gps_task(void *pvParameters);

// Copy latest fix (returns true if valid; out still filled even if false)
bool gps_get_fix(gps_fix_t *out);
