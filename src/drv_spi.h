#pragma once

uint8_t rfmReadRegister(uint8_t which, uint8_t reg);
void rfmWriteRegister(uint8_t which, uint8_t reg, uint8_t data);
void configureSPI();



