#pragma once

#define PIN_MOSI GPIO_Pin_15
#define PIN_MISO GPIO_Pin_14
#define PIN_SCK GPIO_Pin_13

uint8_t rfmReadRegister(uint8_t which, uint8_t reg);
void rfmWriteRegister(uint8_t which, uint8_t reg, uint8_t data);
void configureSPI();
void configureSPIBitBang();



