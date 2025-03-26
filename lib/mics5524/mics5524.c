#include "mics5524.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define MICS5524_ADC_CHANNEL ADC1_CHANNEL_0  // Change GPIO if needed

void mics5524_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit resolution
    adc1_config_channel_atten(MICS5524_ADC_CHANNEL, ADC_ATTEN_DB_11);  // 0-3.9V range
}

uint32_t mics5524_read() {
    return adc1_get_raw(MICS5524_ADC_CHANNEL);  // Get raw ADC value
}
