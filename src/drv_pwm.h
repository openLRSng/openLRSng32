#pragma once

void setPWM(uint8_t ch, uint16_t value);
void configurePWMs(uint8_t channels);
void enablePWMout(uint8_t ch, bool enable, bool polarity);
void configureServoPWM(uint8_t);
void configurePPM(uint8_t);
void configureRssiPWM(void);
void configureLbeepPWM(void);
void enableLbeep(bool enable);
void set_RSSI_output(uint8_t);
