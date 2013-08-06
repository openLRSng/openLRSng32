#pragma once

void rfmSetChannel(uint8_t unit, uint8_t ch);
uint8_t rfmGetRSSI(uint8_t unit);
void rfmSetCarrierFrequency(uint8_t unit, uint32_t f);
void init_rfm(uint8_t unit, uint8_t isbind);
void to_rx_mode(uint8_t unit);
void rx_ready(uint8_t unit);
void rx_reset(uint8_t unit);
void rfmReceive(uint8_t unit, uint8_t *data, uint8_t size);
void tx_packet(uint8_t unit, uint8_t* pkt, uint8_t size);
void beacon_send(uint8_t unit);
void rfmPreInit();
int8_t rfmCheckInt(uint8_t unit);
