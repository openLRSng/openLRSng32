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
  uint32_t startFreq = MIN_RFM_FREQUENCY, endFreq = MAX_RFM_FREQUENCY, nrSamples = 500, stepSize = 50000;
  uint32_t currentFrequency = startFreq;
  uint32_t currentSamples = 0;
  uint8_t nextIndex = 0;
  uint8_t rssiMin = 0, rssiMax = 0;
  uint32_t rssiSum = 0;
  LEDR_OFF;
  LEDG_OFF;
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
          startFreq = nextConfig[0] * 1000UL; // MHz
          endFreq   = nextConfig[1] * 1000UL; // MHz
          nrSamples = nextConfig[2]; // count
          stepSize  = nextConfig[3] * 1000UL;   // 10kHz
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
      printf("%d,%d,%d,%d,\r\n",
	     currentFrequency / 1000UL, rssiMax, rssiSum / currentSamples, rssiMin);
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

#define PPM_CHANNELS 16
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 ,512,512,512,512,512,512,512,512};

uint8_t RF_channel = 0;

uint32_t time;
uint32_t last_pack_time = 0;
uint32_t last_rssi_time = 0;
uint32_t fs_time; // time when failsafe activated

uint32_t last_beacon;

uint8_t  RSSI_count = 0;
uint16_t RSSI_sum = 0;
uint8_t  last_rssi_value = 0;
uint8_t  smoothRSSI = 0;

uint16_t ppmSync = 40000;
uint8_t  ppmChannels = 16;

volatile uint8_t disablePWM = 0;
volatile uint8_t disablePPM = 0;

uint8_t firstpack = 0;
uint8_t lostpack = 0;

bool willhop = 0, fs_saved = 0;

void save_failsafe_values(void)
{
  FLASH_Status status;
  uint32_t i;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  if (FLASH_ErasePage(FLASH_FSWRITE_ADDR) == FLASH_COMPLETE)
  {
	status = FLASH_ProgramWord(FLASH_FSWRITE_ADDR, 0xDEADC0DE );
		if (status != FLASH_COMPLETE)
		{
			//freak out!
		}
    for (i = 0; i < sizeof(PPM); i += 4)
    {
      status = FLASH_ProgramWord(FLASH_FSWRITE_ADDR + 4 + i, *(uint32_t *) ((char *)&PPM + i));
      if (status != FLASH_COMPLETE)
      {
    	  break;
      }
    }
  }
  FLASH_Lock();
}

void load_failsafe_values(void)
{
	uint32_t * tempb = (uint32_t *)FLASH_FSWRITE_ADDR;

  if(*tempb == 0xDEADC0DE)
  {
	uint8_t i;
	memcpy(&PPM, (char *)(FLASH_FSWRITE_ADDR + 4), sizeof(PPM));
	for (i=0; i<PPM_CHANNELS; i++)
	{
		if(rx_config.RSSIpwm != i)
		{
			setPWM(i,servoBits2Us(PPM[i]));
		}
	}
  }
}

uint8_t bindReceive(uint32_t timeout)
{
  uint32_t start = millis();
  uint8_t  rxb;
  init_rfm(1,1);
  to_rx_mode(1);
  LEDR_ON;
  printf("Waiting bind\r\n");

  while ((!timeout) || ((millis() - start) < timeout))
  {
    if (rfmCheckInt(1))
    {   // RFM22B int16_t pin Enabled by received Data
      printf("Got pkt\r\n");
      rfmReceive(1, &rxb, 1);
      if (rxb=='b')
      {
    	rfmReceive(1, (uint8_t *)&bind_data, sizeof(bind_data));

        if (bind_data.version == BINDING_VERSION)
        {
          printf("data good\r\n");
          rxb='B';
          tx_packet(1,&rxb,1); // ACK that we got bound
          LEDG_ON; //signal we got bound on LED:s
//          if (timeout) {
          return 1;
//          }
        }
      }
      else if ((rxb=='p') || (rxb=='i'))
      {
        uint8_t rxc_buf[sizeof(rx_config)+1];
        if (rxb=='p')
        {
          printf("Sending RX config\r\n");
          rxc_buf[0]='P';
          timeout=0;
        }
        else
        {
          printf("Reinit RX config\r\n");
          rxInitDefaults();
          rxWriteEeprom();
          rxc_buf[0]='I';
        }
        memcpy(rxc_buf+1, &rx_config, sizeof(rx_config));
        tx_packet(1,rxc_buf,sizeof(rx_config)+1);
      }
      else if (rxb=='u')
      {
        rfmReceive(1, (uint8_t *)&rx_config, sizeof(rx_config));
        rxWriteEeprom();
        printRXconf();
        rxb='U';
        tx_packet(1,&rxb,1); // ACK that we updated settings
      }
      //RF_Mode = Receive;
      rx_reset(1);
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
  
  if (rfmReadRegister(1, 0x0C) == 0) // detect the locked module and reboot
  {
    printf("RX hang\r\n");
    init_rfm(1,0);
    to_rx_mode(1);
  }
  
  //time = micros();
  rfm1=rfmCheckInt(1);
  rfm2=rfmCheckInt(2);
  if (rfm1 || rfm2)
  {
    uint8_t rfmRXed = rfm1?1:2;

    last_pack_time = micros(); // record last package time
    lostpack = 0;
    
    LEDR_OFF;
    LEDG_ON;
    
    rfmReceive(rfmRXed, rx_buf, getPacketSize());
    
    if ((rx_buf[0]&0x3e) == 0x00) //servo data
    {
      uint8_t i;
      unpackChannels(bind_data.flags & 7, PPM, rx_buf + 1);

      for (i=0; i<PPM_CHANNELS; i++)
      {

    	if (rx_config.RSSIpwm == i)
    		setPWM(rx_config.RSSIpwm,servoBits2Us(smoothRSSI<<2));
    	else
    		setPWM(i,servoBits2Us(PPM[rx_config.pinMapping[i]]));
      }
    }
    
    if (firstpack == 0)
    {
      firstpack = 1;
      LEDB_ON;
      uint8_t i;
      for (i=0; i<MAX_OUTPUTS; i++)
      {
    	  if((rx_config.pinMapping[i] == PINMAP_PPM) & ( i == PPM_PIN ))
    	  {
    		  printf("config ppm chan %d\r\n",i);
    		  configurePPM(ppmChannels);
    	  }
    	  else if((rx_config.pinMapping[i] == PINMAP_RSSI) & ( i == RSSI_PIN ))
    	  {
    		  printf("config rssi chan %d\r\n",i);
    	  	  configureRssiPWM();
    	  }
    	  else
    	  {
    		  //config regular pwm/servo-rssi
    		  printf("config pwm chan %d\r\n",i);
    		  configureServoPWM(i);
    	  }
      }
    }
    else
    {
    	disablePWM = 0;
    	disablePPM = 0;
    }

    if (rx_buf[0] & 0x01)
    {
      if (!fs_saved)
      {
    	  save_failsafe_values();
    	  fs_saved = 1;
      }
    }
    else if (fs_saved)
    {
      fs_saved = 0;
    }

    rfm1=rfmCheckInt(1);
    rfm2=rfmCheckInt(2);
    if (bind_data.flags & TELEMETRY_ENABLED)
    {
      // reply with telemetry
      uint8_t telemetry_packet[4];
      telemetry_packet[0] = last_rssi_value;
      rx_ready((rfmRXed==1)?2:1); // shutdown other module
      tx_packet(rfmRXed,telemetry_packet, 4);
    }

    rx_reset(1);
    rx_reset(2);

    willhop = 1;

    //LEDG_OFF; //i prefer this bright :)
  }
if (rx_config.pinMapping[PPM_PIN] == PINMAP_PPM)
{
  if (disablePPM == 1)
  {
	enablePPMout(0);
  }
  else
  {
	enablePPMout(1);
  }
}
  if(disablePWM)
  {
	uint8_t i;
	for (i=0; i<PPM_CHANNELS; i++)
	{
	  setPWM(i,0);
	}
  }

  time = micros();

  // sample RSSI when packet is in the 'air'
  if ((lostpack < 2) & (last_rssi_time != last_pack_time) & ((time - last_pack_time) > (getInterval() - 1500)))
  {
    last_rssi_time = last_pack_time;
    last_rssi_value = rfmGetRSSI(1); // Read the RSSI value
    RSSI_sum += last_rssi_value;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 20)
    {
      RSSI_sum /= RSSI_count;
      RSSI_sum = constrain(RSSI_sum, 45, 255);
      smoothRSSI = (uint8_t)(((uint16_t)smoothRSSI * 6 + (uint16_t)RSSI_sum * 2)/8);
      set_RSSI_output(smoothRSSI);
      RSSI_sum = 0;
      RSSI_count = 0;
    }
  }

  time = micros();

  if (firstpack) {
      if ((lostpack < bind_data.hopcount) && ((time - last_pack_time) > (getInterval(&bind_data) + 1000))) {
        // we lost packet, hop to next channel
        willhop = 1;
        if (lostpack==0) {
          fs_time = time;
          last_beacon = 0;
        }
        lostpack++;
        last_pack_time += getInterval(&bind_data);
        willhop = 1;
        LEDR_ON;
        LEDG_OFF;
        if (smoothRSSI>30) {
          smoothRSSI-=30;
        } else {
          smoothRSSI=0;
        }
        set_RSSI_output(smoothRSSI);
      } else if ((lostpack == bind_data.hopcount) && ((time - last_pack_time) > (getInterval(&bind_data) * bind_data.hopcount))) {
        // hop slowly to allow resync with TX
        willhop = 1;
        smoothRSSI=0;
        set_RSSI_output(smoothRSSI);
        last_pack_time = time;
      }

      if (lostpack) {
        if ((fs_time) && ((time - fs_time) > (uint32_t)(rx_config.failsafe_delay * 100000UL))) {
          load_failsafe_values();
          if (rx_config.flags & FAILSAFE_NOPWM) {
            disablePWM = 1;
          }
          if (rx_config.flags & FAILSAFE_NOPPM) {
            disablePPM = 1;
          }
          fs_time=0;
          last_beacon = (time + ((uint32_t)rx_config.beacon_deadtime * 1000000UL)) | 1; //beacon activating...
        }

        if ((rx_config.beacon_frequency) && (last_beacon)) {
          if (((time - last_beacon) < 0x80000000) && // last beacon is future during deadtime
              (time - last_beacon) > ((uint32_t)rx_config.beacon_interval * 1000000UL)) {
            beacon_send(1);
            init_rfm(1,0);   // go back to normal RX
            rx_reset(1);
            last_beacon = micros() | 1; // avoid 0 in time
          }
        }
      }
    } else {
      // Waiting for first packet, hop slowly
      if ((time - last_pack_time) > (getInterval(&bind_data) * bind_data.hopcount)) {
        last_pack_time = time;
        willhop = 1;
      }
    }

  if (willhop == 1)
  {
    RF_channel++;

    if (RF_channel >= bind_data.hopcount)
    {
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
  //checkReflash();

  rxReadEeprom();
  printRXconf();

  for (i=0; i<16; i++)
  {
    setPWM(i,1000);
  }
  configureSPI();
  rfmPreInit();

//  if (checkIfConnected(OUTPUT_PIN[2],OUTPUT_PIN[3]))
//  { // ch1 - ch2 --> force scannerMode
//    scannerMode();
//  }


  if (bindReadEeprom())
  {
	  printf("Good bind data found\r\n");
    if (bindReceive(1000))
    {
      bindWriteEeprom();
    }
  }
  else
  {
    while (!bindReceive(1000))
    {
     // if (uartAvailable() && ('R' == uartRead()))
     // {
     //	  systemReset(true);      // reboot to bootloader
     // }
    }
    bindWriteEeprom();
  }
  
  printf("Entering normal mode\r\n");
  LEDR_ON;
  ppmChannels = getChannelCount(&bind_data);
  init_rfm(1,0);   // Configure the RFM22B's registers for normal operation
  //init_rfm(2,0);   // Configure the RFM22B's registers for normal operation
  RF_channel = 0;
  rfmSetChannel(1,bind_data.hopchannel[RF_channel]);
  //rfmSetChannel(2,bind_data.hopchannel[RF_channel]);

  to_rx_mode(1);
  //to_rx_mode(2);
  firstpack = 0;

  while(1)
  {
    loop();
    //if (uartAvailable() && ('R' == uartRead()))
    //{
    //  systemReset(true);      // reboot to bootloader
    //}
  }

}

