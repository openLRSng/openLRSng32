#pragma once

typedef struct drv_adc_config_t {
    uint8_t analogEnable;
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
