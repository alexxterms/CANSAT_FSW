#include "gps.h"
#include "esp_err.h" // Add this include for esp_err_t
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GPS";

// Shared state
static struct gps_fix g_fix;
static SemaphoreHandle_t g_fix_mux;

// Use the satellite_info struct defined in gps.h
static struct satellite_info g_sats[MAX_SATS];
static int g_sats_in_view = 0;

// ----------------- Helpers -----------------

// XOR checksum of payload (between '$' and '*'), returns true if matches hex at end
static bool nmea_verify_checksum(const char *sentence) {
    const char *p = sentence;
    if (*p != '$') return false;
    p++;

    uint8_t sum = 0;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        sum ^= (uint8_t)*p++;
    }
    if (*p != '*') return false;
    p++; // move to hex

    char hex[3] = {0,0,0};
    if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1])) return false;
    hex[0] = p[0];
    hex[1] = p[1];

    uint8_t expected = (uint8_t)strtoul(hex, NULL, 16);
    return sum == expected;
}

// Convert NMEA ddmm.mmmm (lat) / dddmm.mmmm (lon) into degrees as double
static bool nmea_dm_to_deg(const char *dm, bool is_lat, double *out_deg) {
    if (!dm || !*dm) return false;

    char *dot = strchr(dm, '.');
    int len = (int) (dot ? (dot - dm) : strlen(dm));

    // Latitude: 2 deg digits, Longitude: 3 deg digits
    int deg_digits = is_lat ? 2 : 3;
    if (len < deg_digits) return false;

    char deg_str[4] = {0};
    strncpy(deg_str, dm, deg_digits);
    int deg = atoi(deg_str);

    const char *min_str = dm + deg_digits;
    double minutes = atof(min_str);

    *out_deg = (double)deg + (minutes / 60.0);
    return true;
}

// Parse HHMMSS.ss into components
static void nmea_parse_time(const char *t, uint8_t *hh, uint8_t *mm, uint8_t *ss, uint16_t *ms) {
    *hh = *mm = *ss = 0; *ms = 0;
    if (!t || strlen(t) < 6) return;

    char buf[16] = {0};
    strncpy(buf, t, sizeof(buf) - 1);

    // Split on '.'
    char *dot = strchr(buf, '.');
    if (dot) *dot = '\0';

    if (isdigit((unsigned char)buf[0])) {
        *hh = (uint8_t)((buf[0]-'0')*10 + (buf[1]-'0'));
        *mm = (uint8_t)((buf[2]-'0')*10 + (buf[3]-'0'));
        *ss = (uint8_t)((buf[4]-'0')*10 + (buf[5]-'0'));
    }

    if (dot) {
        const char *frac = dot + 1; // e.g., "23" for .23
        // scale to milliseconds (two digits -> hundredths of a second)
        int hundredths = atoi(frac); // rough parse
        // normalize length (".ss" where ss are hundredths)
        if (strlen(frac) == 1) hundredths *= 10;
        if (hundredths < 0) hundredths = 0;
        if (hundredths > 99) hundredths = 99;
        *ms = (uint16_t)(hundredths * 10); // 0..990 ms
    }
}

// String to uint helpers with scaling
static uint16_t parse_hdop_x100(const char *s) {
    if (!s || !*s) return 0;
    double v = atof(s);
    long scaled = lround(v * 100.0);
    if (scaled < 0) scaled = 0;
    if (scaled > 65535) scaled = 65535;
    return (uint16_t)scaled;
}

static int32_t meters_to_cm(const char *s) {
    if (!s || !*s) return 0;
    double v = atof(s);
    long cm = lround(v * 100.0);
    if (cm < INT32_MIN) cm = INT32_MIN;
    if (cm > INT32_MAX) cm = INT32_MAX;
    return (int32_t)cm;
}

static int32_t deg_to_e7(double deg) {
    double scaled = deg * 1e7;
    if (scaled >  2147483647.0) scaled =  2147483647.0;
    if (scaled < -2147483648.0) scaled = -2147483648.0;
    return (int32_t)llround(scaled);
}

// Very small CSV tokenizer (in-place)
static int split_commas(char *s, char *fields[], int max_fields) {
    int n = 0;
    char *p = s;
    while (p && *p && n < max_fields) {
        fields[n++] = p;
        char *c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

// Parse GSV sentence and update satellite info
static void parse_gsv(char *payload) {
    // payload starts at "GPGSV,..."
    // Fields: 0 type, 1 total_msgs, 2 msg_num, 3 total_sats, [4..] sat blocks (4 fields each)
    char *f[32] = {0};
    int nf = split_commas(payload, f, 32);
    if (nf < 4) return;

    int total_msgs = atoi(f[1] ? f[1] : "0");
    int msg_num    = atoi(f[2] ? f[2] : "0");
    int total_sats = atoi(f[3] ? f[3] : "0");

    if (msg_num == 1) { // new sequence
        memset(g_sats, 0, sizeof(g_sats));
        g_sats_in_view = total_sats;
    }

    int sat_block_start = 4;
    int sats_in_this_msg = (nf - sat_block_start) / 4;
    for (int i = 0; i < sats_in_this_msg; i++) {
        int idx = (msg_num - 1) * 4 + i;
        if (idx >= MAX_SATS) break;

        g_sats[idx].prn       = atoi(f[sat_block_start + i*4 + 0] ? f[sat_block_start + i*4 + 0] : "0");
        g_sats[idx].elevation = atoi(f[sat_block_start + i*4 + 1] ? f[sat_block_start + i*4 + 1] : "0");
        g_sats[idx].azimuth   = atoi(f[sat_block_start + i*4 + 2] ? f[sat_block_start + i*4 + 2] : "0");
        g_sats[idx].snr       = atoi(f[sat_block_start + i*4 + 3] ? f[sat_block_start + i*4 + 3] : "0");
    }

    // Optionally: update g_fix.sats with the number in view
    if (msg_num == total_msgs) { // last message in sequence
        if (g_fix_mux && xSemaphoreTake(g_fix_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_fix.sats = (uint8_t)g_sats_in_view;
            xSemaphoreGive(g_fix_mux);
        } else {
            g_fix.sats = (uint8_t)g_sats_in_view;
        }
    }
}


// Parse GGA sentence and update global fix
static void parse_gga(char *payload) {
    // payload starts at "GPGGA,..." (no leading $)
    // Expected fields:
    // 0 talker+type, 1 time, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 fix, 7 sats, 8 hdop, 9 alt, 10 'M', ...
    char *f[20] = {0};
    int nf = split_commas(payload, f, 20);
    if (nf < 11) return;

    const char *time_str = f[1];
    const char *lat_dm   = f[2];
    const char *ns       = f[3];
    const char *lon_dm   = f[4];
    const char *ew       = f[5];
    const char *fixq     = f[6];
    const char *sats_str = f[7];
    const char *hdop_str = f[8];
    const char *alt_str  = f[9];

    uint8_t hh, mm, ss; uint16_t ms;
    nmea_parse_time(time_str, &hh, &mm, &ss, &ms);

    double lat_deg = 0.0, lon_deg = 0.0;
    if (!nmea_dm_to_deg(lat_dm, true,  &lat_deg)) return;
    if (!nmea_dm_to_deg(lon_dm, false, &lon_deg)) return;
    if (ns && (*ns == 'S' || *ns == 's')) lat_deg = -lat_deg;
    if (ew && (*ew == 'W' || *ew == 'w')) lon_deg = -lon_deg;

    uint8_t fix_quality = (uint8_t)atoi(fixq ? fixq : "0");
    uint8_t sats = (uint8_t)atoi(sats_str ? sats_str : "0");
    uint16_t hdop_x100 = parse_hdop_x100(hdop_str);
    int32_t alt_cm = meters_to_cm(alt_str);

    struct gps_fix tmp = {0};
    tmp.valid       = (fix_quality > 0);
    tmp.hh          = hh;
    tmp.mm          = mm;
    tmp.ss          = ss;
    tmp.ms          = ms;
    tmp.lat_e7      = deg_to_e7(lat_deg);
    tmp.lon_e7      = deg_to_e7(lon_deg);
    tmp.alt_cm      = alt_cm;
    tmp.hdop_x100   = hdop_x100;
    tmp.sats        = sats;
    tmp.fix_quality = fix_quality;

    if (g_fix_mux && xSemaphoreTake(g_fix_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_fix = tmp;
        xSemaphoreGive(g_fix_mux);
    } else {
        g_fix = tmp; // fallback (rare)
    }
}

static void parse_sentence(char *line) {
    // Expect "$xxxxx,....*HH"
    if (line[0] != '$') return;
    if (!nmea_verify_checksum(line)) {
        ESP_LOGW(TAG, "Bad checksum: %s", line);
        return;
    }

    // Make a mutable copy without leading '$' and trailing *HH part
    char buf[128];
    size_t n = strnlen(line, sizeof(buf) - 1);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    strncpy(buf, line + 1, n); // skip '$'
    buf[n] = '\0';

    // Trim "*HH" and any CRLF
    char *star = strchr(buf, '*');
    if (star) *star = '\0';
    for (int i = (int)strlen(buf) - 1; i >= 0 && (buf[i] == '\r' || buf[i] == '\n'); --i) buf[i] = '\0';

    // Identify type
    if (strncmp(buf, "GPGGA", 5) == 0 || strncmp(buf, "GNGGA", 5) == 0) {
        parse_gga(buf);
    }
    else if (strncmp(buf, "GPGSV", 5) == 0 || strncmp(buf, "GNGSV", 5) == 0) {
        parse_gsv(buf);
    }
    // You can add RMC/GSV/etc. here later
}

// ----------------- Task -----------------

void gps_task(void *pvParameters) {
    memset(&g_fix, 0, sizeof(g_fix));
    g_fix_mux = xSemaphoreCreateMutex();

    // UART init
    const uart_config_t cfg = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_RX_BUF_BYTES, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d init @ %d baud (RX=%d, TX=%d)",
             (int)GPS_UART_NUM, GPS_UART_BAUD, GPS_RX_PIN, GPS_TX_PIN);

    // Line assembly
    char line[128];
    size_t idx = 0;

    uint8_t rx[128];

    for (;;) {
        int n = uart_read_bytes(GPS_UART_NUM, rx, sizeof(rx), pdMS_TO_TICKS(200));
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            char c = (char)rx[i];
            if (c == '\r') continue;

            if (c == '\n') {
                if (idx > 0) {
                    line[idx] = '\0';
                    parse_sentence(line);
                    idx = 0;
                }
            } else {
                if (idx < sizeof(line) - 1) {
                    line[idx++] = c;
                } else {
                    // overflow -> reset
                    idx = 0;
                }
            }
        }
    }
}

// ----------------- API -----------------

bool gps_get_fix(struct gps_fix *out) {
    if (!out) return false;
    bool valid;
    if (g_fix_mux && xSemaphoreTake(g_fix_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = g_fix;
        valid = g_fix.valid;
        xSemaphoreGive(g_fix_mux);
    } else {
        *out = g_fix; // best effort
        valid = g_fix.valid;
    }
    return valid;
}

// Return number of satellites in view and copy info into provided buffer.
// Returns actual count copied (<= max).
int gps_get_satellites(struct satellite_info *out, int max) {
    if (!out || max <= 0) return 0;
    int copied = 0;

    if (g_fix_mux && xSemaphoreTake(g_fix_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
        copied = (g_sats_in_view < max) ? g_sats_in_view : max;
        memcpy(out, g_sats, copied * sizeof(struct satellite_info));
        xSemaphoreGive(g_fix_mux);
    } else {
        copied = (g_sats_in_view < max) ? g_sats_in_view : max;
        memcpy(out, g_sats, copied * sizeof(struct satellite_info));
    }

    return copied;
}

// Compatibility function for main.c
esp_err_t gps_get_data(GPSData *data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    
    struct gps_fix fix;
    bool valid = gps_get_fix(&fix);
    
    // Convert from gps_fix_t to GPSData format
    data->utc_time[0] = fix.hh;
    data->utc_time[1] = fix.mm; 
    data->utc_time[2] = fix.ss;
    
    data->latitude = (float)(fix.lat_e7 / 1e7);
    data->latitude_dir = (fix.lat_e7 >= 0);
    
    data->longitude = (float)(fix.lon_e7 / 1e7);
    data->longitude_dir = (fix.lon_e7 >= 0);
    
    data->fix_status = fix.fix_quality;
    data->hdop = fix.hdop_x100;
    data->satellites_used = fix.sats;
    data->altitude = fix.alt_cm;
    
    return valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// Initialize GPS (for compatibility)
void gps_init(void) {
    // GPS initialization is handled in gps_task
    // This function exists for compatibility with main.c
}
