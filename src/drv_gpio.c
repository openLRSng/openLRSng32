#pragma once
#include "board.h"


uint8_t checkIfConnected(uint8_t pin1, uint8_t pin2)
{
  bool ret = 0;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = pinDefine[pin1].pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(pinDefine[pin1].gpio, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pinDefine[pin2].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(pinDefine[pin2].gpio, &GPIO_InitStructure);

  GPIO_SetBits(pinDefine[pin1].gpio,pinDefine[pin1].pin);
  delay(10);

  if (GPIO_ReadInputDataBit(pinDefine[pin2].gpio,pinDefine[pin2].pin)) {
    GPIO_ResetBits(pinDefine[pin1].gpio,pinDefine[pin1].pin);
    delay(10);

    if (!GPIO_ReadInputDataBit(pinDefine[pin2].gpio,pinDefine[pin2].pin)) {
      ret = 1;
    }
  }

  //Return pins to default state
  GPIO_InitStructure.GPIO_Pin = pinDefine[pin1].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pinDefine[pin1].gpio, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pinDefine[pin2].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pinDefine[pin2].gpio, &GPIO_InitStructure);
  return ret;
}
