#include "board.h"

struct flash_head {
  uint32_t magic;
  uint32_t size;
  uint32_t checksum;
  uint32_t reserved;
};

struct bind_data bind_data;
struct rx_config rx_config;

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
//  if (ret < 20000) {
//    ret = 20000;
//  }

  return ret;
}

static uint32_t checksum(uint8_t *data, uint32_t size) {
  uint32_t ret = 0;
  while (size--) {
    if (ret & 0x80000000) {
      ret = (ret<<1) | 1;
    } else {
      ret = (ret<<1);
    }
    ret^=*(data++);
  }
  return ret;
}

int16_t bindReadEeprom()
{
  const struct flash_head *head = (const struct flash_head*)(FLASH_WRITE_ADDR);
  const struct bind_data *temp = (const struct bind_data*)(FLASH_WRITE_ADDR + sizeof(struct flash_head));
  if (head->magic != BIND_MAGIC) {
    printf("FLASH MAGIC FAIL\r\n");
    return 0;
  }
  
  if (head->size != sizeof(bind_data)) {
    printf("FLASH SIZE FAIL\r\n");
    return 0;
  }
  
  if (head->checksum != checksum((uint8_t*)temp, sizeof(bind_data))) {
    printf("FLASH CHECKSUM FAIL\r\n");
    return 0;
  }
  
  if (temp->version != BINDING_VERSION) {
    printf("FLASH VERSION FAIL\r\n");
    return 0;
  }
  
  memcpy(&bind_data, temp, sizeof(bind_data));
  
  return 1;
}

void bindWriteEeprom(void)
{
  FLASH_Status status;
  uint32_t i;

  struct flash_head head;
  
  head.magic = BIND_MAGIC;
  head.size = sizeof(bind_data);
  head.checksum = checksum((uint8_t*)(&bind_data), sizeof(bind_data));
  
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
    for (i = 0; i < sizeof(struct flash_head); i += 4) {
      status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *)&head + i));
      if (status != FLASH_COMPLETE) {
	FLASH_Lock();
	printf("FLASH ERROR on head!!!\r\n");
	while(1);
      }
    }
    for (i = 0; i < sizeof(struct bind_data); i += 4) {
      status = FLASH_ProgramWord(FLASH_WRITE_ADDR + sizeof(struct flash_head) + i, *(uint32_t *) ((char *)&bind_data + i));
      if (status != FLASH_COMPLETE) {
	FLASH_Lock();
	printf("FLASH ERROR on bind!!!\r\n");
	while(1);
      }
    }

  } else {
    printf("FLASH ERASE FAILED\r\n");
  }
  FLASH_Lock();
}

void bindInitDefaults(void)
{
  bind_data.version = BINDING_VERSION;
  bind_data.rf_power = DEFAULT_RF_POWER;
  bind_data.rf_frequency = DEFAULT_CARRIER_FREQUENCY;
  bind_data.rf_channel_spacing = DEFAULT_CHANNEL_SPACING;

  bind_data.rf_magic = DEFAULT_RF_MAGIC;

  //bind_data.hopcount = sizeof(default_hop_list) / sizeof(default_hop_list[0]);
  uint8_t default_hop_list[] = {DEFAULT_HOPLIST};
  uint8_t c;

  for (c = 0; c < MAXHOPS; c++) {
    bind_data.hopchannel[c] = ((c < sizeof(default_hop_list)) ? default_hop_list[c] : 0);
  }

  bind_data.modem_params = DEFAULT_DATARATE;
  bind_data.flags = DEFAULT_FLAGS;
}

void bindRandomize(void)
{
  uint8_t c;

  bind_data.rf_magic = 0;
  for (c = 0; c < 4; c++)
  {
    bind_data.rf_magic = (bind_data.rf_magic << 8) + (rand() % 255);
  }

  for (c = 0; (c < MAXHOPS) & (bind_data.hopchannel[c] != 0); c++) //TODO: make sure this doesn't fuck up
  {
    uint8_t ch = (rand() % 50);

    // don't allow same channel twice
    uint8_t i;
    for (i = 0; i < c; i++)
    {
      if (bind_data.hopchannel[i] == ch)
      {
    	  c--;
      }
      else
      {
    	  bind_data.hopchannel[c] = ch;
      }
    }
  }
}

void rxInitDefaults()
{
  uint8_t i;
//#if (BOARD_TYPE == 3)
//  rx_config.rx_type = RX_FLYTRON8CH;
//  rx_config.pinMapping[0] = PINMAP_RSSI; // the CH0 on 8ch RX
//  for (i=1; i < 9; i++) {
//    rx_config.pinMapping[i] = i-1; // default to PWM out
//  }
//#elif (BOARD_TYPE == 5)
//  rx_config.rx_type = RX_OLRSNG4CH;
//  for (i=0; i<5; i++) {
//    rx_config.pinMapping[i]=i; // default to PWM out
//  }
#if (BOARD_TYPE == DTFUHF10CH)
  rx_config.rx_type = RX_DTFUHF10CH;
  for (i=0; i < 8; i++) {
      rx_config.pinMapping[i] = i; // default to PWM out
    }

  rx_config.pinMapping[RSSI_PIN] = PINMAP_RSSI;
  rx_config.pinMapping[PPM_PIN] = PINMAP_PPM;

#endif

  rx_config.flags = ALWAYS_BIND;
  rx_config.RSSIpwm = 255; //rssi injection disabled
  rx_config.beacon_frequency = DEFAULT_BEACON_FREQUENCY;
  rx_config.beacon_deadtime = DEFAULT_BEACON_DEADTIME;
  rx_config.beacon_interval = DEFAULT_BEACON_INTERVAL;
  rx_config.minsync = 3000;
  rx_config.failsafe_delay = 10;
}

void rxWriteEeprom()
{
	FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_RXWRITE_ADDR) == FLASH_COMPLETE)
    {
        status = FLASH_ProgramWord(FLASH_RXWRITE_ADDR, BIND_MAGIC);
        if (status != FLASH_COMPLETE)
        {
        	FLASH_Lock();
        	printf("FLASH ERROR on rx_config!!!\r\n");
        	while(1);
        }
        else
        {
        	printf("flash success rx_bind_magic\r\n");
        }
        for (i = 0; i < sizeof(rx_config); i += 4)
        {
        	status = FLASH_ProgramWord(FLASH_RXWRITE_ADDR + 4 + i, *(uint32_t *) ((char *)&rx_config + i));
        	if (status != FLASH_COMPLETE)
        	{
        		FLASH_Lock();
        		printf("FLASH ERROR on rx_config!!!\r\n");
        		while(1);
        	}
        }
    }
    else
    {
    	printf("FLASH ERASE FAILED\r\n");
    }
    FLASH_Lock();
}

void rxReadEeprom()
{
  uint32_t * tempb = (uint32_t *)FLASH_RXWRITE_ADDR;

  if (*tempb != BIND_MAGIC)
  {
    printf("RXconf reinit\r\n");
    rxInitDefaults();
    rxWriteEeprom();
  }
  else
  {
	  const struct rx_config *temp = (const struct rx_config*)(FLASH_RXWRITE_ADDR + 4);
	  memcpy(&rx_config, temp, sizeof(rx_config));
    printf("RXconf loaded\r\n");
  }
}

void printRXconf()
{
  uint8_t i;
  printf("Type: %d\r\n",rx_config.rx_type);
  for (i=0; i<13; i++) {
    printf("pmap%d: %d\r\n",i,rx_config.pinMapping[i]);
  }
  printf("Flag: %d\r\n",rx_config.flags);
  printf("rssi: %d\r\n",rx_config.RSSIpwm);
  printf("Bfre: %d\r\n",rx_config.beacon_frequency);
  printf("Bdea: %d\r\n",rx_config.beacon_deadtime);
  printf("Bint: %d\r\n",rx_config.beacon_interval);
  printf("msyc: %d\r\n",rx_config.minsync);
  printf("fsfe: %d\r\n",rx_config.failsafe_delay);
}

