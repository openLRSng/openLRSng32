#include "board.h"

#include "binding.h"

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

#define RFMNO 2

volatile uint8_t RF_Mode = 0;

#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

void rfmSetCarrierFrequency(uint32_t f);
uint8_t rfmGetRSSI(void);
void RF22B_init_parameter(void);
void tx_packet(uint8_t* pkt, uint8_t size);
void to_rx_mode(void);

#define PPM_CHANNELS 16
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 ,512,512,512,512,512,512,512,512};

const static uint8_t pktsizes[8] = {0, 7, 11, 12, 16, 17, 21, 0};

//const static char *chConfStr[8] = {"N/A", "4+4", "8", "8+4", "12", "12+4", "16", "N/A"};

uint8_t getPacketSize(struct bind_data *bd)
{
  return pktsizes[(bd->flags & 0x07)];
}

uint8_t getChannelCount(struct bind_data *bd)
{
  return (((bd->flags & 7)/2) + 1 + (bd->flags & 1)) * 4;
}

uint32_t getInterval(struct bind_data *bd)
{
  uint32_t ret;
  // Sending a x byte packet on bps y takes about (emperical)
  // usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes,bps) ((uint32_t)((bytes)+15) * 8200000L / (uint32_t)(bps))

  ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(bd), modem_params[bd->modem_params].bps)+2000);

  if (bd->flags & TELEMETRY_ENABLED) {
    ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE,modem_params[bd->modem_params].bps)+1000);
  }

  // round up to ms
  ret= ((ret+999) / 1000) * 1000;

  // not faster than 50Hz
  if (ret < 20000) {
    ret = 20000;
  }

  return ret;
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
  uint32_t startFreq = 430000000, endFreq = 440000000, nrSamples = 500, stepSize = 50000;
  uint32_t currentFrequency = startFreq;
  uint32_t currentSamples = 0;
  uint8_t nextIndex = 0;
  uint8_t rssiMin = 0, rssiMax = 0;
  uint32_t rssiSum = 0;
  LEDR_OFF;
  LEDR_OFF;
  printf("scanner mode\n");
  to_rx_mode();

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
          nextIndex = 0;
          startFreq = nextConfig[0] * 1000000UL; // MHz
          endFreq   = nextConfig[1] * 1000000UL; // MHz
          nrSamples = nextConfig[2]; // count
          stepSize  = nextConfig[3] * 10000UL;   // 10kHz
          currentFrequency = startFreq;
          currentSamples = 0;

          // set IF filtter BW (kha)
          if (stepSize < 20000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x32);   // 10.6kHz
          } else if (stepSize < 30000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x22);   // 21.0kHz
          } else if (stepSize < 40000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x26);   // 32.2kHz
          } else if (stepSize < 50000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x12);   // 41.7kHz
          } else if (stepSize < 60000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x15);   // 56.2kHz
          } else if (stepSize < 70000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x01);   // 75.2kHz
          } else if (stepSize < 100000) {
            rfmWriteRegister(RFMNO, 0x1c, 0x03);   // 90.0kHz
          } else {
            rfmWriteRegister(RFMNO, 0x1c, 0x05);   // 112.1kHz
          }
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
      rfmSetCarrierFrequency(currentFrequency);
      rssiMax = 0;
      rssiMin = 255;
      rssiSum = 0;
      delay(1);
    }

    if (currentSamples < nrSamples) {
      uint8_t val = rfmGetRSSI();
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


void RFM22B_Int()
{
  if (RF_Mode == Transmit) {
    RF_Mode = Transmitted;
  }

  if (RF_Mode == Receive) {
    RF_Mode = Received;
  }
}

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY        0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX           0x05
#define RF22B_PWRSTATE_TX           0x09

#define RF22B_Rx_packet_received_interrupt   0x02

uint8_t ItStatus1, ItStatus2;

void to_sleep_mode(void);
void rx_reset(void);

// **** RFM22 access functions

void rfmSetChannel(uint8_t ch)
{
  rfmWriteRegister(RFMNO, 0x79, ch);
}

uint8_t rfmGetRSSI(void)
{
  return rfmReadRegister(RFMNO, 0x26);
}

void setModemRegs(struct rfm22_modem_regs* r)
{

  rfmWriteRegister(RFMNO, 0x1c, r->r_1c);
  rfmWriteRegister(RFMNO, 0x1d, r->r_1d);
  rfmWriteRegister(RFMNO, 0x1e, r->r_1e);
  rfmWriteRegister(RFMNO, 0x20, r->r_20);
  rfmWriteRegister(RFMNO, 0x21, r->r_21);
  rfmWriteRegister(RFMNO, 0x22, r->r_22);
  rfmWriteRegister(RFMNO, 0x23, r->r_23);
  rfmWriteRegister(RFMNO, 0x24, r->r_24);
  rfmWriteRegister(RFMNO, 0x25, r->r_25);
  rfmWriteRegister(RFMNO, 0x2a, r->r_2a);
  rfmWriteRegister(RFMNO, 0x6e, r->r_6e);
  rfmWriteRegister(RFMNO, 0x6f, r->r_6f);
  rfmWriteRegister(RFMNO, 0x70, r->r_70);
  rfmWriteRegister(RFMNO, 0x71, r->r_71);
  rfmWriteRegister(RFMNO, 0x72, r->r_72);
}

void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  rfmWriteRegister(RFMNO, 0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
  rfmWriteRegister(RFMNO, 0x76, (fc >> 8));
  rfmWriteRegister(RFMNO, 0x77, (fc & 0xff));
}

void init_rfm(uint8_t isbind)
{
  ItStatus1 = rfmReadRegister(RFMNO, 0x03);   // read status, clear interrupt
  ItStatus2 = rfmReadRegister(RFMNO, 0x04);
  rfmWriteRegister(RFMNO, 0x06, 0x00);    // disable interrupts
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_READY); // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  rfmWriteRegister(RFMNO, 0x09, 0x7f);   // c = 12.5p
  rfmWriteRegister(RFMNO, 0x0a, 0x05);
  rfmWriteRegister(RFMNO, 0x0b, 0x12);    // gpio0 TX State
  rfmWriteRegister(RFMNO, 0x0c, 0x15);    // gpio1 RX State
  rfmWriteRegister(RFMNO, 0x0d, 0xfd);    // gpio 2 micro-controller clk output
  rfmWriteRegister(RFMNO, 0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  if (isbind) {
    setModemRegs(&bind_params);
  } else {
    setModemRegs(&modem_params[bind_data.modem_params]);
  }

  // Packet settings
  rfmWriteRegister(RFMNO, 0x30, 0x8c);    // enable packet handler, msb first, enable crc,
  rfmWriteRegister(RFMNO, 0x32, 0x0f);    // no broadcast, check header bytes 3,2,1,0
  rfmWriteRegister(RFMNO, 0x33, 0x42);    // 4 byte header, 2 byte synch, variable pkt size
  rfmWriteRegister(RFMNO, 0x34, 0x0a);    // 10 nibbles (40 bit preamble)
  rfmWriteRegister(RFMNO, 0x35, 0x2a);    // preath = 5 (20bits), rssioff = 2
  rfmWriteRegister(RFMNO, 0x36, 0x2d);    // synchronize word 3
  rfmWriteRegister(RFMNO, 0x37, 0xd4);    // synchronize word 2
  rfmWriteRegister(RFMNO, 0x38, 0x00);    // synch word 1 (not used)
  rfmWriteRegister(RFMNO, 0x39, 0x00);    // synch word 0 (not used)

  uint32_t magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  uint8_t i;
  for (i=0; i<4; i++) {
    rfmWriteRegister(RFMNO, 0x3a + i, (magic >> 24) & 0xff);   // tx header
    rfmWriteRegister(RFMNO, 0x3f + i, (magic >> 24) & 0xff);   // rx header
    magic = magic << 8; // advance to next byte
  }

  rfmWriteRegister(RFMNO, 0x43, 0xff);    // all the bit to be checked
  rfmWriteRegister(RFMNO, 0x44, 0xff);    // all the bit to be checked
  rfmWriteRegister(RFMNO, 0x45, 0xff);    // all the bit to be checked
  rfmWriteRegister(RFMNO, 0x46, 0xff);    // all the bit to be checked

  if (isbind) {
    rfmWriteRegister(RFMNO, 0x6d, BINDING_POWER);
  } else {
    rfmWriteRegister(RFMNO, 0x6d, bind_data.rf_power);
  }

  rfmWriteRegister(RFMNO, 0x79, 0);

  rfmWriteRegister(RFMNO, 0x7a, bind_data.rf_channel_spacing);   // channel spacing

  rfmWriteRegister(RFMNO, 0x73, 0x00);
  rfmWriteRegister(RFMNO, 0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(isbind ? BINDING_FREQUENCY : bind_data.rf_frequency);

}

void to_rx_mode(void)
{
  ItStatus1 = rfmReadRegister(RFMNO, 0x03);
  ItStatus2 = rfmReadRegister(RFMNO, 0x04);
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_READY);
  delay(10);
  rx_reset();
}

void rx_reset(void)
{
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_READY);
  rfmWriteRegister(RFMNO, 0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received
  rfmWriteRegister(RFMNO, 0x08, 0x03);    //clear fifo disable multi packet
  rfmWriteRegister(RFMNO, 0x08, 0x00);    // clear fifo, disable multi packet
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_RX);   // to rx mode
  rfmWriteRegister(RFMNO, 0x05, RF22B_Rx_packet_received_interrupt);
  ItStatus1 = rfmReadRegister(RFMNO, 0x03);   //read the Interrupt Status1 register
  ItStatus2 = rfmReadRegister(RFMNO, 0x04);
}

void tx_packet(uint8_t* pkt, uint8_t size)
{
  uint8_t i;
  rfmWriteRegister(RFMNO, 0x3e, size);   // total tx size

  for (i = 0; i < size; i++) {
    rfmWriteRegister(RFMNO, 0x7f, pkt[i]);
  }

  rfmWriteRegister(RFMNO, 0x05, RF22B_PACKET_SENT_INTERRUPT);
  ItStatus1 = rfmReadRegister(RFMNO, 0x03);      //read the Interrupt Status1 register
  ItStatus2 = rfmReadRegister(RFMNO, 0x04);
#ifdef TX_TIMING
  uint32_t tx_start = micros();
#endif
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_TX);    // to tx mode

  //  while (nIRQ_1);

#ifdef TX_TIMING
  Serial.print("TX took:");
  Serial.println(micros() - tx_start);
#endif
}

void beacon_tone(int16_t hz, int16_t len)
{
  int16_t d = 500 / hz; // somewhat limited resolution ;)

  if (d < 1) {
    d = 1;
  }

  int16_t cycles = (len * 1000 / d);

  while (cycles--) {
    //  SDI_on;
    delay(d);
    // SDI_off;
    delay(d);
  }
}

void beacon_send(void)
{
  LEDG_ON
  ItStatus1 = rfmReadRegister(RFMNO, 0x03);   // read status, clear interrupt
  ItStatus2 = rfmReadRegister(RFMNO, 0x04);
  rfmWriteRegister(RFMNO, 0x06, 0x00);    // no wakeup up, lbd,
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  rfmWriteRegister(RFMNO, 0x09, 0x7f);  // (default) c = 12.5p
  rfmWriteRegister(RFMNO, 0x0a, 0x05);
  rfmWriteRegister(RFMNO, 0x0b, 0x12);    // gpio0 TX State
  rfmWriteRegister(RFMNO, 0x0c, 0x15);    // gpio1 RX State
  rfmWriteRegister(RFMNO, 0x0d, 0xfd);    // gpio 2 micro-controller clk output
  rfmWriteRegister(RFMNO, 0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  rfmWriteRegister(RFMNO, 0x70, 0x2C);    // disable manchest

  rfmWriteRegister(RFMNO, 0x30, 0x00);    //disable packet handling

  rfmWriteRegister(RFMNO, 0x79, 0);    // start channel

  rfmWriteRegister(RFMNO, 0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

  rfmWriteRegister(RFMNO, 0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  rfmWriteRegister(RFMNO, 0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  rfmWriteRegister(RFMNO, 0x73, 0x00);
  rfmWriteRegister(RFMNO, 0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(bind_data.beacon_frequency);

  rfmWriteRegister(RFMNO, 0x6d, 0x07);   // 7 set max power 100mW

  delay(10);
  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);
  beacon_tone(500, 1);

  rfmWriteRegister(RFMNO, 0x6d, 0x04);   // 4 set mid power 15mW
  delay(10);
  beacon_tone(250, 1);

  rfmWriteRegister(RFMNO, 0x6d, 0x00);   // 0 set min power 1mW
  delay(10);
  beacon_tone(160, 1);

  rfmWriteRegister(RFMNO, 0x07, RF22B_PWRSTATE_READY);
  LEDG_OFF
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

#if 0
#define FAILSAFE_OFFSET 0x80

void save_failsafe_values(void)
{
  uint8_t ee_buf[20];

  packChannels(6, PPM, ee_buf);
  for (int16_t i = 0; i < 20; i++) {
    EEPROM.write(FAILSAFE_OFFSET + 4 +i, ee_buf[i]);
  }

  ee_buf[0]=0xFA;
  ee_buf[1]=0x11;
  ee_buf[2]=0x5A;
  ee_buf[3]=0xFE;
  for (int16_t i = 0; i < 4; i++) {
    EEPROM.write(FAILSAFE_OFFSET + i, ee_buf[i]);
  }
}

void load_failsafe_values(void)
{
  uint8_t ee_buf[20];

  for (int16_t i = 0; i < 4; i++) {
    ee_buf[i] = EEPROM.read(FAILSAFE_OFFSET + i);
  }

  if ((ee_buf[0]==0xFA) && (ee_buf[1]==0x11) && (ee_buf[2]==0x5A) && (ee_buf[3]==0xFE)) {
    for (int16_t i = 0; i < 20; i++) {
      ee_buf[i] = EEPROM.read(FAILSAFE_OFFSET + 4 +i);
    }
    cli();
    unpackChannels(6, PPM, ee_buf);
    sei();
  }
}

#endif

uint8_t bindReceive(uint32_t timeout)
{
  uint32_t start = millis();
  uint8_t i;
  init_rfm(1);
  RF_Mode = Receive;
  to_rx_mode();
  printf("Waiting bind\n");

  while ((!timeout) || ((millis() - start) < timeout)) {
    //    if (RF_Mode == Received) {   // RFM22B int16_t pin Enabled by received Data
    if (uartAvailable()) {   // RFM22B int16_t pin Enabled by received Data
      uartRead();
      printf("Got pkt\n");
      RF_Mode = Receive;

      for (i = 0; i < sizeof(bind_data); i++) {
        *(((uint8_t*)&bind_data) + i) = rfmReadRegister(RFMNO, 0x7f);
      }

      if (bind_data.version == BINDING_VERSION) {
        printf("data good\n");
        return 1;
      } else {
        rx_reset();
      }
    }
  }
  return 0;
}

#if 0

uint8_t checkIfGrounded(uint8_t pin)
{
  int8_t ret = 0;

  pinMode(pin, INPUT);
  digitalWrite(pin, 1);
  delay(10);

  if (!digitalRead(pin)) {
    ret = 1;
  }

  digitalWrite(pin, 0);

  return ret;
}

int8_t checkIfConnected(uint8_t pin1, uint8_t pin2)
{
  int8_t ret = 0;
  pinMode(pin1, OUTPUT);
  digitalWrite(pin1, 1);
  digitalWrite(pin2, 1);
  delay(10);

  if (digitalRead(pin2)) {
    digitalWrite(pin1, 0);
    delay(10);

    if (!digitalRead(pin2)) {
      ret = 1;
    }
  }

  pinMode(pin1, INPUT);
  digitalWrite(pin1, 0);
  digitalWrite(pin2, 0);
  return ret;
}

void setup()
{
  LEDR_ON;

  if (checkIfConnected(PWM_3,PWM_4)) { // ch1 - ch2 --> force scannerMode
    scannerMode();
  }

  if (checkIfConnected(PWM_1,PWM_2) || (!bindReadEeprom())) {
    Serial.print("EEPROM data not valid or bind jumpper set, forcing bind\n");

    if (bindReceive(0)) {
      bindWriteEeprom();
      Serial.println("Saved bind data to EEPROM\n");
      Green_LED_ON;
    }
  } else {
#ifdef RX_ALWAYS_BIND
    if (bindReceive(500)) {
      bindWriteEeprom();
      Serial.println("Saved bind data to EEPROM\n");
      Green_LED_ON;
    }
#endif
  }

  // Check for bind plug on ch8 (PPM enable).
  if (checkIfConnected(PWM_7,PWM_8)) {
    PPM_output = 1;
  } else {
    PPM_output = 0;
  }

#ifdef FORCED_PPM_OUTPUT
  PPM_output = 1;
#endif

  ppmChannels = getChannelCount(&bind_data);

  printf("Entering normal mode with PPM=%d CHs=%d\n", PPM_output, ppmChannels);
  init_rfm(0);   // Configure the RFM22B's registers for normal operation
  RF_channel = 0;
  rfmSetChannel(bind_data.hopchannel[RF_channel]);

  //################### RX SYNC AT STARTUP #################
  RF_Mode = Receive;
  to_rx_mode();

  firstpack = 0;

}

uint8_t rx_buf[21]; // RX buffer

//############ MAIN LOOP ##############
void loop()
{
  uint32_t time;

  if (rfmReadRegister(RFMNO, 0x0C) == 0) {     // detect the locked module and reboot
    Serial.println("RX hang");
    init_rfm(0);
    to_rx_mode();
  }

  time = micros();

  if (RF_Mode == Received) {   // RFM22B int16_t pin Enabled by received Data

    last_pack_time = micros(); // record last package time
    lostpack = 0;

    Red_LED_OFF;
    Green_LED_ON;

    spiSendAddress(0x7f);   // Send the package read command

    for (int16_t i = 0; i < getPacketSize(&bind_data); i++) {
      rx_buf[i] = spiReadData();
    }

    if ((rx_buf[0] == 0x5E) || (rx_buf[0] == 0xF5)) {
      cli();
      unpackChannels(bind_data.flags & 7, PPM, rx_buf + 1);
      sei();
    }

    if (firstpack == 0) {
      firstpack = 1;
      setupPPMout();
    } else {
      if ((PPM_output) && (bind_data.flags & FAILSAFE_NOPPM)) {
        TCCR1A |= ((1 << COM1A1) | (1 << COM1A0));
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

    if (bind_data.flags & TELEMETRY_ENABLED) {
      // reply with telemetry
      uint8_t telemetry_packet[4];
      telemetry_packet[0] = last_rssi_value;
      tx_packet(telemetry_packet, 4);
    }

    RF_Mode = Receive;
    rx_reset();

    willhop = 1;

    Green_LED_OFF;
  }

  time = micros();

  // sample RSSI when packet is in the 'air'
  if ((lostpack < 2) && (last_rssi_time != last_pack_time) &&
      (time - last_pack_time) > (getInterval(&bind_data) - 1500)) {
    last_rssi_time = last_pack_time;
    last_rssi_value = rfmGetRSSI(); // Read the RSSI value
    RSSI_sum += last_rssi_value;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 20) {
      RSSI_sum /= RSSI_count;
      set_RSSI_output(map(constrain(RSSI_sum, 45, 200), 40, 200, 0, 255));
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
      Red_LED_ON;
      set_RSSI_output(0);
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
        if ((PPM_output) && (bind_data.flags & FAILSAFE_NOPPM)) {
          TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
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
            beacon_send();
            init_rfm(0);   // go back to normal RX
            rx_reset();
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

    rfmSetChannel(bind_data.hopchannel[RF_channel]);
    willhop = 0;
  }

}
#endif

void setupRFMints()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
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
    setPWM(i,1000+i*50);
  }
  configurePWMs();
  configureSPI();
  setupRFMints();

  // loopy
  while (1) {
    bindReceive(5000);
    //    scannerMode();
  }
}

