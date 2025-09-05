#include "smartaudio.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "smartaudio";

static int sa_uart_num = SMARTAUDIO_DEFAULT_UART_NUM;
static smartaudio_version_t sa_version = SMARTAUDIO_VERSION_V2;

// Simple XOR checksum helper
static uint8_t xor_checksum(const uint8_t *data, size_t len)
{
	uint8_t chk = 0;
	for (size_t i = 0; i < len; i++) {
		chk ^= data[i];
	}
	return chk;
}

// Default frame builder: [CMD][PAYLOAD...][CHK = XOR(CMD..last payload)]
// NOTE: Many SmartAudio implementations use different framing. If your VTX requires
// a different frame (start byte, length, inverted bits, CRC), replace this function.
static size_t build_frame(uint8_t cmd, const uint8_t *payload, size_t payload_len, uint8_t *out_buf, size_t out_size)
{
	if (!out_buf || out_size < (1 + payload_len + 1)) {
		return 0;
	}
	out_buf[0] = cmd;
	if (payload_len && payload) {
		memcpy(&out_buf[1], payload, payload_len);
	}
	uint8_t chk = xor_checksum(out_buf, 1 + payload_len);
	out_buf[1 + payload_len] = chk;
	return 1 + payload_len + 1;
}

bool smartaudio_init(int uart_num, int tx_gpio_num, bool invert_tx, smartaudio_version_t version, int baud)
{
	uart_config_t uart_config = {
		.baud_rate = baud > 0 ? baud : SMARTAUDIO_DEFAULT_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	sa_uart_num = uart_num;
	sa_version = version;

	esp_err_t err = uart_param_config(sa_uart_num, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config failed: %d", err);
		return false;
	}
	err = uart_set_pin(sa_uart_num, tx_gpio_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
		return false;
	}
	err = uart_driver_install(sa_uart_num, 1024, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
		return false;
	}
	// Optionally invert TX/RX lines for VTXes that expect inverted UART
	if (invert_tx) {
		// invert TX and RX signals if requested. Use UART signal inversion flags.
		uart_set_line_inverse(sa_uart_num, UART_SIGNAL_TXD_INV);
	} else {
		uart_set_line_inverse(sa_uart_num, 0);
	}

	ESP_LOGI(TAG, "SmartAudio initialized on UART %d TX pin %d (invert=%d) version=%d baud=%d", sa_uart_num, tx_gpio_num, invert_tx, sa_version, uart_config.baud_rate);
	return true;
}

void smartaudio_deinit(void)
{
	uart_driver_delete(sa_uart_num);
}

esp_err_t smartaudio_send_raw(const uint8_t *data, size_t len)
{
	if (!data || len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	int written = uart_write_bytes(sa_uart_num, (const char *)data, len);
	if (written < 0) {
		return ESP_FAIL;
	}
	// Wait for transmission to finish (blocking short time)
	uart_wait_tx_done(sa_uart_num, pdMS_TO_TICKS(100));
	return ESP_OK;
}

// High-level command implementations
// For SmartAudio v2 (AKK X2 Ultimate) common command IDs used by community implementations:
#define SMARTAUDIO_V2_CMD_SET_POWER   0x10  // set tx power (payload: single byte level)
#define SMARTAUDIO_V2_CMD_SET_PIT     0x11  // set PIT (payload: 0 = disable, 1 = enable)

esp_err_t smartaudio_set_power(uint8_t level)
{
	// Payload for set power: single byte level (0..N). AKK X2 commonly uses 0..4.
	uint8_t frame[4];
	size_t frame_len = build_frame(SMARTAUDIO_V2_CMD_SET_POWER, &level, 1, frame, sizeof(frame));
	if (frame_len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	return smartaudio_send_raw(frame, frame_len);
}

esp_err_t smartaudio_set_pit(bool enable)
{
	uint8_t payload = enable ? 1 : 0;
	uint8_t frame[4];
	size_t frame_len = build_frame(SMARTAUDIO_V2_CMD_SET_PIT, &payload, 1, frame, sizeof(frame));
	if (frame_len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	return smartaudio_send_raw(frame, frame_len);
}
