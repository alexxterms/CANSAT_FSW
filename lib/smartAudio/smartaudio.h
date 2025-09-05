#ifndef SMARTAUDIO_H
#define SMARTAUDIO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ...existing code...

// Configure default UART and baud (can be overridden at init)
#define SMARTAUDIO_DEFAULT_UART_NUM  UART_NUM_1
#define SMARTAUDIO_DEFAULT_BAUD     115200

// SmartAudio protocol version selection (adjust if you want v1)
typedef enum {
    SMARTAUDIO_VERSION_V1 = 1,
    SMARTAUDIO_VERSION_V2 = 2,
} smartaudio_version_t;

// High-level API
bool smartaudio_init(int uart_num, int tx_gpio_num, bool invert_tx, smartaudio_version_t version, int baud);
void smartaudio_deinit(void);

// Convenience init for AKK X2 Ultimate: selects SmartAudio v2 and enables TX inversion
static inline bool smartaudio_init_akk_x2(int uart_num, int tx_gpio_num) {
    return smartaudio_init(uart_num, tx_gpio_num, true, SMARTAUDIO_VERSION_V2, SMARTAUDIO_DEFAULT_BAUD);
}

// Set VTX power. 'level' meaning depends on VTX (0..N). For AKK X2 common levels are 0..4 (check your VTX).
esp_err_t smartaudio_set_power(uint8_t level);

// Enable/disable PIT (True = enable PIT / blind/no-video). Some VTXes use PIT=1 for off video; check your hardware.
esp_err_t smartaudio_set_pit(bool enable);

// Send an arbitrary raw command (useful for testing exact bytes)
esp_err_t smartaudio_send_raw(const uint8_t *data, size_t len);

// ...existing code...

#endif // SMARTAUDIO_H
