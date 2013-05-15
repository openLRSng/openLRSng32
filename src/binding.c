#include "board.h"

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE  ((uint16_t)0x400)
#define FLASH_WRITE_ADDR (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1)) // use the last page

struct flash_head {
  uint32_t magic;
  uint32_t size;
  uint32_t checksum;
  uint32_t reserved;
};

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



