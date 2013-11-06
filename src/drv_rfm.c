#include "board.h"

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY        0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX           0x05
#define RF22B_PWRSTATE_TX           0x09

#define RF22B_Rx_packet_received_interrupt   0x02

uint8_t ItStatus1, ItStatus2;

void selectRFM(uint8_t which)
{
  if (which == 1) {
    digitalLo(GPIOB,GPIO_Pin_12);
  } else {
    digitalHi(GPIOB,GPIO_Pin_12);
  }
  if (which == 2) {
    digitalLo(GPIOC,GPIO_Pin_15);
  } else {
    digitalHi(GPIOC,GPIO_Pin_15);
  }
}

uint8_t rfmReadRegister(uint8_t which, uint8_t reg)
{
  selectRFM(which);
  SPI_I2S_SendData(SPI2, ((uint16_t)(0x00 + (reg & 0x7f))<<8));
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  selectRFM(0);
  return (uint8_t) SPI_I2S_ReceiveData(SPI2);
}

void rfmWriteRegister(uint8_t which, uint8_t reg, uint8_t data)
{
  //printf("RFM%d(%x)=%x\r\n",which,reg,data);
  selectRFM(which);
  SPI_I2S_SendData(SPI2, ((uint16_t)(0x80 + (reg & 0x7f))<<8) + data);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI2);
  selectRFM(0);
}

// **** RFM22 access functions

void rfmSetChannel(uint8_t unit, uint8_t ch)
{
  rfmWriteRegister(unit, 0x79, ch);
}

uint8_t rfmGetRSSI(uint8_t unit)
{
  return rfmReadRegister(unit, 0x26);
}

uint16_t rfmGetAFCC(uint8_t unit)
{
  return (((uint16_t)rfmReadRegister(unit, 0x2B)<<2) | ((uint16_t)rfmReadRegister(unit, 0x2C)>>6));
}

static void setModemRegs(uint8_t unit, struct rfm22_modem_regs* r)
{

  rfmWriteRegister(unit, 0x1c, r->r_1c);
  rfmWriteRegister(unit, 0x1d, r->r_1d);
  rfmWriteRegister(unit, 0x1e, r->r_1e);
  rfmWriteRegister(unit, 0x20, r->r_20);
  rfmWriteRegister(unit, 0x21, r->r_21);
  rfmWriteRegister(unit, 0x22, r->r_22);
  rfmWriteRegister(unit, 0x23, r->r_23);
  rfmWriteRegister(unit, 0x24, r->r_24);
  rfmWriteRegister(unit, 0x25, r->r_25);
  rfmWriteRegister(unit, 0x2a, r->r_2a);
  rfmWriteRegister(unit, 0x6e, r->r_6e);
  rfmWriteRegister(unit, 0x6f, r->r_6f);
  rfmWriteRegister(unit, 0x70, r->r_70);
  rfmWriteRegister(unit, 0x71, r->r_71);
  rfmWriteRegister(unit, 0x72, r->r_72);
}

void rfmSetCarrierFrequency(uint8_t unit, uint32_t f)
{
  //printf("RFM%d(CF)=%d\r\n",unit,f);
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
  rfmWriteRegister(unit, 0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
  rfmWriteRegister(unit, 0x76, (fc >> 8));
  rfmWriteRegister(unit, 0x77, (fc & 0xff));
}

void init_rfm(uint8_t unit, uint8_t isbind)
{
  ItStatus1 = rfmReadRegister(unit, 0x03);   // read status, clear interrupt
  ItStatus2 = rfmReadRegister(unit, 0x04);
  rfmWriteRegister(unit, 0x06, 0x00);    // disable interrupts
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY); // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  rfmWriteRegister(unit, 0x09, 0x7f);   // c = 12.5p
  rfmWriteRegister(unit, 0x0a, 0x05);
  rfmWriteRegister(unit, 0x0b, 0x12);    // gpio0 TX State
  rfmWriteRegister(unit, 0x0c, 0x15);    // gpio1 RX State
  rfmWriteRegister(unit, 0x0d, 0xfd);    // gpio 2 micro-controller clk output
  rfmWriteRegister(unit, 0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  setModemRegs(unit, isbind?&bind_params:&modem_params[bind_data.modem_params]);

  // Packet settings
#ifdef USE_ECC
  rfmWriteRegister(unit, 0x30, 0x88);    // enable packet handler, msb first, disable crc,
#else
  rfmWriteRegister(unit, 0x30, 0x8c);    // enable packet handler, msb first, enable crc,
#endif

  rfmWriteRegister(unit, 0x32, 0x0f);    // no broadcast, check header bytes 3,2,1,0
  rfmWriteRegister(unit, 0x33, 0x42);    // 4 byte header, 2 byte synch, variable pkt size
  rfmWriteRegister(unit, 0x34, 0x0a);    // 10 nibbles (40 bit preamble)
  rfmWriteRegister(unit, 0x35, 0x2a);    // preath = 5 (20bits), rssioff = 2
  rfmWriteRegister(unit, 0x36, 0x2d);    // synchronize word 3
  rfmWriteRegister(unit, 0x37, 0xd4);    // synchronize word 2
  rfmWriteRegister(unit, 0x38, 0x00);    // synch word 1 (not used)
  rfmWriteRegister(unit, 0x39, 0x00);    // synch word 0 (not used)

  uint32_t magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  uint8_t i;
  for (i=0; i<4; i++) {
    rfmWriteRegister(unit, 0x3a + i, (magic >> 24) & 0xff);   // tx header
    rfmWriteRegister(unit, 0x3f + i, (magic >> 24) & 0xff);   // rx header
    magic = magic << 8; // advance to next byte
  }

  rfmWriteRegister(unit, 0x43, 0xff);    // all the bit to be checked
  rfmWriteRegister(unit, 0x44, 0xff);    // all the bit to be checked
  rfmWriteRegister(unit, 0x45, 0xff);    // all the bit to be checked
  rfmWriteRegister(unit, 0x46, 0xff);    // all the bit to be checked

  if (isbind) {
    rfmWriteRegister(unit, 0x6d, BINDING_POWER);
  } else {
    rfmWriteRegister(unit, 0x6d, bind_data.rf_power);
  }

  rfmWriteRegister(unit, 0x79, 0);

  rfmWriteRegister(unit, 0x7a, bind_data.rf_channel_spacing);   // channel spacing

  rfmWriteRegister(unit, 0x73, 0x00);
  rfmWriteRegister(unit, 0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(unit, isbind ? BINDING_FREQUENCY : bind_data.rf_frequency);

}

void to_rx_mode(uint8_t unit)
{
  ItStatus1 = rfmReadRegister(unit, 0x03);
  ItStatus2 = rfmReadRegister(unit, 0x04);
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY);
  delay(10);
  rx_reset(unit);
}

void rx_ready(uint8_t unit)
{
  ItStatus1 = rfmReadRegister(unit, 0x03);
  ItStatus2 = rfmReadRegister(unit, 0x04);
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY);
  rfmWriteRegister(unit, 0x05, 0);
  ItStatus1 = rfmReadRegister(unit, 0x03);
  ItStatus2 = rfmReadRegister(unit, 0x04);
}

void rx_reset(uint8_t unit)
{
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY);
  rfmWriteRegister(unit, 0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received
  rfmWriteRegister(unit, 0x08, 0x03);    //clear fifo disable multi packet
  rfmWriteRegister(unit, 0x08, 0x00);    // clear fifo, disable multi packet
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_RX);   // to rx mode
  rfmWriteRegister(unit, 0x05, RF22B_Rx_packet_received_interrupt);
  ItStatus1 = rfmReadRegister(unit, 0x03);   //read the Interrupt Status1 register
  ItStatus2 = rfmReadRegister(unit, 0x04);
}

void rfmReceive(uint8_t unit, uint8_t *data, uint8_t size)
{
  while (size--) {
    *(data++) = rfmReadRegister(unit, 0x7f);
  }
}

void tx_packet(uint8_t unit, uint8_t* pkt, uint8_t size)
{
  uint8_t i;
  rfmWriteRegister(unit, 0x3e, size);   // total tx size

  for (i = 0; i < size; i++) {
    rfmWriteRegister(unit, 0x7f, pkt[i]);
  }

  rfmWriteRegister(unit, 0x05, RF22B_PACKET_SENT_INTERRUPT);
  ItStatus1 = rfmReadRegister(unit, 0x03);      //read the Interrupt Status1 register
  ItStatus2 = rfmReadRegister(unit, 0x04);
#ifdef TX_TIMING
  uint32_t tx_start = micros();
#endif
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_TX);    // to tx mode

  while (!rfmCheckInt(unit));
  ItStatus1 = rfmReadRegister(unit, 0x03);      //read the Interrupt Status1 register
  ItStatus2 = rfmReadRegister(unit, 0x04);


#ifdef TX_TIMING
  printf("TX took: %d\r\n",(micros() - tx_start));
#endif
}

void beacon_tone(int32_t hz, int32_t len, uint8_t which) //duration is in seconds.
{
  int32_t d = 500000 / hz;

  if (d < 100) {
    d = 100;
  }

  selectRFM(which);

  configureSPIBitBang();

  len *= 1000;
  uint32_t start = millis();
  while ((millis()-start) < len) {
    GPIO_SetBits(GPIOB, PIN_MOSI);
    delayMicroseconds(d);
    GPIO_ResetBits(GPIOB, PIN_MOSI);
    delayMicroseconds(d);
  }

  configureSPI();

  selectRFM(0);

}

void beacon_send(uint8_t unit)
{
  LEDG_ON
  ItStatus1 = rfmReadRegister(unit, 0x03);   // read status, clear interrupt
  ItStatus2 = rfmReadRegister(unit, 0x04);
  rfmWriteRegister(unit, 0x06, 0x00);    // no wakeup up, lbd,
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  rfmWriteRegister(unit, 0x09, 0x7f);  // (default) c = 12.5p
  rfmWriteRegister(unit, 0x0a, 0x05);
  rfmWriteRegister(unit, 0x0b, 0x12);    // gpio0 TX State
  rfmWriteRegister(unit, 0x0c, 0x15);    // gpio1 RX State
  rfmWriteRegister(unit, 0x0d, 0xfd);    // gpio 2 micro-controller clk output
  rfmWriteRegister(unit, 0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  rfmWriteRegister(unit, 0x70, 0x2C);    // disable manchest

  rfmWriteRegister(unit, 0x30, 0x00);    //disable packet handling

  rfmWriteRegister(unit, 0x79, 0);    // start channel

  rfmWriteRegister(unit, 0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

  rfmWriteRegister(unit, 0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  rfmWriteRegister(unit, 0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  rfmWriteRegister(unit, 0x73, 0x00);
  rfmWriteRegister(unit, 0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(unit, rx_config.beacon_frequency);

  rfmWriteRegister(unit, 0x6d, 0x07);   // 7 set max power 100mW

  delay(10);
  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);

  //close encounters tune
  //  G, A, F, F(lower octave), C
  //octave 3:  392  440  349  175   261

  beacon_tone(392, 1, 1);

  rfmWriteRegister(unit, 0x6d, 0x05);   // 5 set mid power 25mW
  delay(10);
  beacon_tone(440,1, 1);

  rfmWriteRegister(unit, 0x6d, 0x04);   // 4 set mid power 13mW
  delay(10);
  beacon_tone(349, 1, 1);

  rfmWriteRegister(unit, 0x6d, 0x02);   // 2 set min power 3mW
  delay(10);
  beacon_tone(175,1, 1);

  rfmWriteRegister(unit, 0x6d, 0x00);   // 0 set min power 1.3mW
  delay(10);
  beacon_tone(261, 2, 1);

  rfmWriteRegister(unit, 0x07, RF22B_PWRSTATE_READY);
  LEDG_OFF
}

static volatile uint8_t rfmIntFired[2]={0,0};

void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line13) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line13);
    rfmIntFired[0]=1;
  }
  if (EXTI_GetITStatus(EXTI_Line14) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line14);
    rfmIntFired[1]=1;
  }
}

int8_t rfmCheckInt(uint8_t unit)
{
  if ((unit==1)||(unit==2)) {
    if (rfmIntFired[unit-1]) {
      rfmIntFired[unit-1]=0;
      //printf("%d\r\n",unit);
      return 1;
    }
  }
  return 0;
}

static void setupRFMints()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  // Enable and set EXTI10-15 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/* CS pins */
//  PB12 GPIO (SPI2NSS) RFM1CS
//  PC15 GPIO RFM2CS

// Setup communication, detect modules and setup interrupts
void rfmPreInit() {
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CS1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // CS2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  selectRFM(0);
  setupRFMints();

}
