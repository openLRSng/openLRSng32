#include "board.h"

uint8_t default_hop_list[] = {DEFAULT_HOPLIST};
struct bind_data bind_data;

struct rfm22_modem_regs modem_params[] = {
  { 4800, 0x1a, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x1b, 0x1e, 0x27, 0x52, 0x2c, 0x23, 0x30 }, // 50000 0x00
  { 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 }, // 25000 0x00
  { 19200,0x06, 0x40, 0x0a, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x7b, 0x28, 0x9d, 0x49, 0x2c, 0x23, 0x30 }  // 25000 0x01
};


struct rfm22_modem_regs bind_params =
  { 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 };

const static uint8_t pktsizes[8] = {0, 7, 11, 12, 16, 17, 21, 0};

//const static char *chConfStr[8] = {"N/A", "4+4", "8", "8+4", "12", "12+4", "16", "N/A"};

uint8_t getPacketSize()
{
  return pktsizes[(bind_data.flags & 0x07)];
}

uint8_t getChannelCount()
{
  return (((bind_data.flags & 7)/2) + 1 + (bind_data.flags & 1)) * 4;
}

uint32_t getInterval()
{
  uint32_t ret;
  // Sending a x byte packet on bps y takes about (emperical)
  // usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes,bps) ((uint32_t)((bytes)+15) * 8200000L / (uint32_t)(bps))

  ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(), modem_params[bind_data.modem_params].bps)+2000);

  if (bind_data.flags & TELEMETRY_ENABLED) {
    ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE,modem_params[bind_data.modem_params].bps)+1000);
  }

  // round up to ms
  ret= ((ret+999) / 1000) * 1000;

  // not faster than 50Hz
  if (ret < 20000) {
    ret = 20000;
  }

  return ret;
}

int16_t bindReadEeprom()
{
  uint32_t temp = 0;
  uint8_t i;
  for (i = 0; i < 4; i++) {
    //    temp = (temp<<8) + EEPROM.read(EEPROM_OFFSET + i);
  }
  if (temp!=BIND_MAGIC) {
    return 0;
  }

  for (i = 0; i < sizeof(bind_data); i++) {
    //    *((uint8_t*)&bind_data + i) = EEPROM.read(EEPROM_OFFSET + 4 + i);
  }

  if (bind_data.version != BINDING_VERSION) {
    return 0;
  }

  return 1;
}

void bindWriteEeprom(void)
{
  uint8_t i;
  for (i = 0; i < 4; i++) {
    //EEPROM.write(EEPROM_OFFSET + i, (BIND_MAGIC >> ((3-i) * 8))& 0xff);
  }

  for (i = 0; i < sizeof(bind_data); i++) {
    //EEPROM.write(EEPROM_OFFSET + 4 + i, *((uint8_t*)&bind_data + i));
  }
}

void bindInitDefaults(void)
{
  uint8_t c;

  bind_data.version = BINDING_VERSION;
  bind_data.rf_power = DEFAULT_RF_POWER;
  bind_data.rf_frequency = DEFAULT_CARRIER_FREQUENCY;
  bind_data.rf_channel_spacing = DEFAULT_CHANNEL_SPACING;

  bind_data.rf_magic = DEFAULT_RF_MAGIC;

  bind_data.hopcount = sizeof(default_hop_list) / sizeof(default_hop_list[0]);

  for (c = 0; c < 8; c++) {
    bind_data.hopchannel[c] = (c < bind_data.hopcount) ? default_hop_list[c] : 0;
  }

  bind_data.modem_params = DEFAULT_DATARATE;
  bind_data.flags = DEFAULT_FLAGS;
  bind_data.beacon_frequency = DEFAULT_BEACON_FREQUENCY;
  bind_data.beacon_interval = DEFAULT_BEACON_INTERVAL;
  bind_data.beacon_deadtime = DEFAULT_BEACON_DEADTIME;
}


