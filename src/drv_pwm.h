#pragma once

void setPWM(uint8_t ch, uint16_t value);
void configurePWMs(uint8_t channels);
void enablePPMout(bool enable);
void configureServoPWM(uint8_t);
void configurePPM(uint8_t);
void configureRssiPWM(void);
void set_RSSI_output(uint8_t);
