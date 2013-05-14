#include "board.h"

/* RX32 SPI layout
  PB13 SPI2 SCK
  PB14 SPI2 MISO
  PB15 SPI2 MOSI
  PB12 GPIO (SPI2NSS) RFM1CS
  PC15 GPIO RFM2CS
*/

void selectRFM(uint8_t which)
{
  digitalHi(GPIOB,GPIO_Pin_12);
  digitalHi(GPIOC,GPIO_Pin_15);

  switch (which) {
  case 1: digitalLo(GPIOB,GPIO_Pin_12);
    break;
  case 2: digitalLo(GPIOC,GPIO_Pin_15);
    break;
  default:
    break;
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
  selectRFM(which);
  SPI_I2S_SendData(SPI2, ((uint16_t)(0x80 + (reg & 0x7f))<<8) + data);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI2);
  selectRFM(0);
}


void configureSPI()
{
  //  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStruct;
  SPI_I2S_DeInit(SPI2);

  // Init pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15; // SCK,MOSI
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; // MISO
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CS1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // CS2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  selectRFM(0);

  SPI_StructInit(&SPI_InitStruct);
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_Init(SPI2, &SPI_InitStruct);
  SPI_Cmd(SPI2, ENABLE);

}



