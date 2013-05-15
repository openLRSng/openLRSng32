#include "board.h"

static void _putc(void *p, char c)
{
    uartWrite(c);
}


void checkReflash()
{
  uint32_t i;
  LEDB_ON;
  for (i=0; i<50; i++) {
    delay(100);
    uartWrite('?');
    if (i&1) {
      LEDG_ON;
      LEDR_OFF;
    } else {
      LEDG_OFF;
      LEDR_ON;
    }

    if (uartAvailable() && ('R' == uartRead())) {
      systemReset(true);      // reboot to bootloader
    }
  }
  LEDB_OFF;
  LEDG_OFF;
}

#define PPM_CHANNELS 16
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 ,512,512,512,512,512,512,512,512};

uint8_t twoBitfy(uint16_t in)
{
  if (in<256) {
    return 0;
  } else if (in<512) {
    return 1;
  } else if (in<768) {
    return 2;
  } else {
    return 3;
  }
}

void packChannels(uint8_t config, volatile uint16_t PPM[], uint8_t *p)
{
  uint8_t i;
  for (i=0; i<=(config/2); i++) { // 4ch packed in 5 bytes
    p[0] = (PPM[0] & 0xff);
    p[1] = (PPM[1] & 0xff);
    p[2] = (PPM[2] & 0xff);
    p[3] = (PPM[3] & 0xff);
    p[4] = ((PPM[0] >> 8) & 3) | (((PPM[1] >> 8) & 3) << 2) | (((PPM[2] >> 8) & 3) << 4) | (((PPM[3] >> 8) & 3) << 6);
    p+=5;
    PPM+=4;
  }
  if (config & 1) { // 4ch packed in 1 byte;
    p[0] = (twoBitfy(PPM[0])<<6) | (twoBitfy(PPM[1])<<4) | (twoBitfy(PPM[2])<<2) | twoBitfy(PPM[3]);
  }
}

void unpackChannels(uint8_t config, volatile uint16_t PPM[], uint8_t *p)
{
  uint8_t i;
  for (i=0; i<=(config/2); i++) { // 4ch packed in 5 bytes
    PPM[0] = (((uint16_t)p[4] & 0x03) << 8) + p[0];
    PPM[1] = (((uint16_t)p[4] & 0x0c) << 6) + p[1];
    PPM[2] = (((uint16_t)p[4] & 0x30) << 4) + p[2];
    PPM[3] = (((uint16_t)p[4] & 0xc0) << 2) + p[3];
    p+=5;
    PPM+=4;
  }
  if (config & 1) { // 4ch packed in 1 byte;
    PPM[0] = (((uint16_t)p[0]>>6)&3)*333+12;
    PPM[1] = (((uint16_t)p[0]>>4)&3)*333+12;
    PPM[2] = (((uint16_t)p[0]>>2)&3)*333+12;
    PPM[3] = (((uint16_t)p[0]>>0)&3)*333+12;
  }
}

// conversion between microseconds 800-2200 and value 0-1023
// 808-1000 == 0 - 11     (16us per step)
// 1000-1999 == 12 - 1011 ( 1us per step)
// 2000-2192 == 1012-1023 (16us per step)

uint16_t servoUs2Bits(uint16_t x)
{
  uint16_t ret;

  if (x < 800) {
    ret = 0;
  } else if (x < 1000) {
    ret = (x - 799) / 16;
  } else if (x < 2000) {
    ret = (x - 988);
  } else if (x < 2200) {
    ret = (x - 1992) / 16 + 1011;
  } else {
    ret = 1023;
  }

  return ret;
}

uint16_t servoBits2Us(uint16_t x)
{
  uint16_t ret;

  if (x < 12) {
    ret = 808 + x * 16;
  } else if (x < 1012) {
    ret = x + 988;
  } else if (x < 1024) {
    ret = 2000 + (x - 1011) * 16;
  } else {
    ret = 2192;
  }

  return ret;
}

// Spectrum analyser 'submode'
void scannerMode(void)
{
  char c;
  uint32_t nextConfig[4] = {0, 0, 0, 0};
  uint32_t startFreq = 430000000, endFreq = 440000000, nrSamples = 500, stepSize = 50000;
  uint32_t currentFrequency = startFreq;
  uint32_t currentSamples = 0;
  uint8_t nextIndex = 0;
  uint8_t rssiMin = 0, rssiMax = 0;
  uint32_t rssiSum = 0;
  LEDR_OFF;
  LEDR_OFF;
  printf("scanner mode\n");
  to_rx_mode(1);

  while (1) {
    while (uartAvailable()) {
      c = uartRead();

      switch (c) {
      case '#':
        nextIndex = 0;
        nextConfig[0] = 0;
        nextConfig[1] = 0;
        nextConfig[2] = 0;
        nextConfig[3] = 0;
        break;

      case ',':
        nextIndex++;

        if (nextIndex == 4) {
	  uint8_t reg1c;
          nextIndex = 0;
          startFreq = nextConfig[0] * 1000000UL; // MHz
          endFreq   = nextConfig[1] * 1000000UL; // MHz
          nrSamples = nextConfig[2]; // count
          stepSize  = nextConfig[3] * 10000UL;   // 10kHz
          currentFrequency = startFreq;
          currentSamples = 0;

          // set IF filtter BW (kha)
          if (stepSize < 20000) {
            reg1c = 0x32;   // 10.6kHz
          } else if (stepSize < 30000) {
            reg1c = 0x22;   // 21.0kHz
          } else if (stepSize < 40000) {
            reg1c = 0x26;   // 32.2kHz
          } else if (stepSize < 50000) {
            reg1c = 0x12;   // 41.7kHz
          } else if (stepSize < 60000) {
            reg1c = 0x15;   // 56.2kHz
          } else if (stepSize < 70000) {
            reg1c = 0x01;   // 75.2kHz
          } else if (stepSize < 100000) {
            reg1c = 0x03;   // 90.0kHz
          } else {
            reg1c = 0x05;   // 112.1kHz
          }
	  rfmWriteRegister(1, 0x1c, reg1c);
        }

        break;

      default:
        if ((c >= '0') && (c <= '9')) {
          c -= '0';
          nextConfig[nextIndex] = nextConfig[nextIndex] * 10 + c;
        }
      }
    }

    if (currentSamples == 0) {
      // retune base
      rfmSetCarrierFrequency(1,currentFrequency);
      rssiMax = 0;
      rssiMin = 255;
      rssiSum = 0;
      delay(1);
    }

    if (currentSamples < nrSamples) {
      uint8_t val = rfmGetRSSI(1);
      rssiSum += val;

      if (val > rssiMax) {
        rssiMax = val;
      }

      if (val < rssiMin) {
        rssiMin = val;
      }

      currentSamples++;
    } else {
      printf("%d,%d,%d,%d,\n",
	     currentFrequency / 10000UL, rssiMax, rssiSum / currentSamples, rssiMin);
      currentFrequency += stepSize;

      if (currentFrequency > endFreq) {
        currentFrequency = startFreq;
      }

      currentSamples = 0;
    }
  }

  //never exit!!
}


/****************************************************
 * OpenLRSng receiver code
 ****************************************************/

uint8_t RF_channel = 0;

uint32_t time;
uint32_t last_pack_time = 0;
uint32_t last_rssi_time = 0;
uint32_t fs_time; // time when failsafe activated

uint32_t last_beacon;

uint8_t  RSSI_count = 0;
uint16_t RSSI_sum = 0;
uint8_t  last_rssi_value = 0;

uint8_t  ppmCountter = 0;
uint16_t ppmSync = 40000;
uint8_t  ppmChannels = 8;

bool PPM_output = 0; // set if PPM output is desired

uint8_t firstpack = 0;
uint8_t lostpack = 0;

bool willhop = 0, fs_saved = 0;

  
void save_failsafe_values(void)
{
  FLASH_Status status;
  uint32_t i;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  if (FLASH_ErasePage(FLASH_FSWRITE_ADDR) == FLASH_COMPLETE) {
    for (i = 0; i < sizeof(PPM); i += 4) {
      status = FLASH_ProgramWord(FLASH_FSWRITE_ADDR + i, *(uint32_t *) ((char *)&PPM + i));
      if (status != FLASH_COMPLETE) {
	break;
      }
    }
  }
  FLASH_Lock();
}

void load_failsafe_values(void)
{
  uint8_t i;
  memcpy(&PPM, (char *)FLASH_FSWRITE_ADDR, sizeof(PPM));
  for (i=0; i<ppmChannels; i++) {
    setPWM(i,servoBits2Us(PPM[i]));
  }
}

uint8_t bindReceive(uint32_t timeout)
{
  uint32_t start = millis();
  init_rfm(1,1);
  to_rx_mode(1);
  printf("Waiting bind\r\n");

  while ((!timeout) || ((millis() - start) < timeout)) {
    //    if (RF_Mode == Received) {   // RFM22B int16_t pin Enabled by received Data
    if (rfmCheckInt(1)) {   // RFM22B int16_t pin Enabled by received Data
      printf("Got pkt\r\n");

      rfmReceive(1, (uint8_t *)&bind_data, sizeof(bind_data));

      if (bind_data.version == BINDING_VERSION) {
        printf("data good\r\n");
        return 1;
      } else {
        rx_reset(1);
      }
    }
  }
  return 0;
}

uint8_t rx_buf[21]; // RX buffer

//############ MAIN LOOP ##############
void loop()
{
  uint32_t time;
  uint8_t  rfm1,rfm2;
  
  if (rfmReadRegister(1, 0x0C) == 0) {     // detect the locked module and reboot
    printf("RX hang\r\n");
    init_rfm(1,0);
    to_rx_mode(1);
  }
  
  //time = micros();
  rfm1=rfmCheckInt(1);
  rfm2=rfmCheckInt(2);
  if (rfm1 || rfm2) {
    uint8_t rfmRXed = rfm1?1:2;

    last_pack_time = micros(); // record last package time
    lostpack = 0;
    
    LEDR_OFF;
    LEDG_ON;
    
    rfmReceive(rfmRXed, rx_buf, getPacketSize());
    
    if ((rx_buf[0] == 0x5E) || (rx_buf[0] == 0xF5)) {
      uint8_t i;
      unpackChannels(bind_data.flags & 7, PPM, rx_buf + 1);
      for (i=0; i<ppmChannels; i++) {
	setPWM(i,servoBits2Us(PPM[i]));
      }
    }
    
    if (firstpack == 0) {
      firstpack = 1;
      LEDB_ON;
      configurePWMs(ppmChannels);
    } else {
      if (bind_data.flags & FAILSAFE_NOPPM) {
	enablePPMout(1);
      }
    }

    if (rx_buf[0] == 0xF5) {
      if (!fs_saved) {
	save_failsafe_values();
        fs_saved = 1;
      }
    } else if (fs_saved) {
      fs_saved = 0;
    }

    rfm1=rfmCheckInt(1);
    rfm2=rfmCheckInt(2);
    if (bind_data.flags & TELEMETRY_ENABLED) {
      // reply with telemetry
      uint8_t telemetry_packet[4];
      telemetry_packet[0] = last_rssi_value;
      rx_ready((rfmRXed==1)?2:1); // shutdown other module
      tx_packet(rfmRXed,telemetry_packet, 4);
    }

    rx_reset(1);
    rx_reset(2);

    willhop = 1;

    LEDG_OFF;
  }

  time = micros();

  // sample RSSI when packet is in the 'air'
  if ((lostpack < 2) && (last_rssi_time != last_pack_time) &&
      (time - last_pack_time) > (getInterval(&bind_data) - 1500)) {
    last_rssi_time = last_pack_time;
    last_rssi_value = rfmGetRSSI(1); // Read the RSSI value
    RSSI_sum += last_rssi_value;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 20) {
      RSSI_sum /= RSSI_count;
      //      set_RSSI_output(map(constrain(RSSI_sum, 45, 200), 40, 200, 0, 255));
      printf("RSSI %d\r\n",RSSI_sum);
      RSSI_sum = 0;
      RSSI_count = 0;
    }
  }

  time = micros();

  if (firstpack) {
    if ((lostpack < 5) && (time - last_pack_time) > (getInterval(&bind_data) + 1000)) {
      // we packet, hop to next channel
      lostpack++;
      last_pack_time += getInterval(&bind_data);
      willhop = 1;
      LEDR_ON;
      //      set_RSSI_output(0);
    } else if ((time - last_pack_time) > (getInterval(&bind_data) * bind_data.hopcount)) {
      // hop slowly to allow resync with TX
      last_pack_time = time;

      if (lostpack < 10) {
        lostpack++;
        if ((bind_data.flags & FAILSAFE_FAST) && (lostpack > 6)) {
          lostpack=10; // go to failsafe faster
        }
      } else if (lostpack == 10) {
        lostpack = 11;
        // Serious trouble, apply failsafe
        load_failsafe_values();
        if (bind_data.flags & FAILSAFE_NOPPM) {
	  enablePPMout(0);
        }
        fs_time = time;
      } else if (bind_data.beacon_interval && bind_data.beacon_deadtime &&
                 bind_data.beacon_frequency) {
        if (lostpack == 11) {   // failsafes set....
          if ((time - fs_time) > (bind_data.beacon_deadtime * 1000000UL)) {
            lostpack = 12;
            last_beacon = time;
          }
        } else if (lostpack == 12) {   // beacon mode active
          if ((time - last_beacon) > (bind_data.beacon_interval * 1000000UL)) {
            last_beacon = time;
            beacon_send(1);
            init_rfm(1,0);   // go back to normal RX
            rx_reset(1);
          }
        }
      }

      willhop = 1;
    }
  }

  if (willhop == 1) {
    RF_channel++;

    if (RF_channel >= bind_data.hopcount) {
      RF_channel = 0;
    }

    rfmSetChannel(1,bind_data.hopchannel[RF_channel]);
    rfmSetChannel(2,bind_data.hopchannel[RF_channel]);
    willhop = 0;
  }

}

int main(void)
{
  uint8_t i;
  systemInit();
  delay(10);
  init_printf(NULL, _putc);
  uartInit(115200);
  delay(100);
  checkReflash();

  for (i=0; i<16; i++) {
    setPWM(i,1000);
  }
  configureSPI();
  rfmPreInit();

  if (bindReadEeprom()) {
    if (bindReceive(1000)) {
      bindWriteEeprom();
    }
  } else {
    while (!bindReceive(1000)) {
      if (uartAvailable() && ('R' == uartRead())) {
	systemReset(true);      // reboot to bootloader
      }
    }
    bindWriteEeprom();
  }
  
  printf("Entering normal mode\r\n");

  ppmChannels = getChannelCount(&bind_data);
  printf("Entering normal mode with PPM=%d CHs=%d\r\n", PPM_output, ppmChannels);
  init_rfm(1,0);   // Configure the RFM22B's registers for normal operation
  init_rfm(2,0);   // Configure the RFM22B's registers for normal operation
  RF_channel = 0;
  rfmSetChannel(1,bind_data.hopchannel[RF_channel]);
  rfmSetChannel(2,bind_data.hopchannel[RF_channel]);

  to_rx_mode(1);
  to_rx_mode(2);
  firstpack = 0;

  while(1){
    loop();
    if (uartAvailable() && ('R' == uartRead())) {
      systemReset(true);      // reboot to bootloader
    }
  }

}

