#include "board.h"
#include "hw_defines.h"

#if BOARD_TYPE == DTFUHF10CH

//special pins def
const uint8_t ANALOG0_PIN = CH7_PIN;
const uint8_t ANALOG1_PIN = CH8_PIN;
const uint8_t RSSI_PIN = CH9_PIN;
const uint8_t PPM_PIN = CH10_PIN;

const struct rxSpecialPinMap rxSpecialPins[] = {
  {6, PINMAP_ANALOG},
  {7, PINMAP_ANALOG},
  {8, PINMAP_RSSI},
  {8, PINMAP_LBEEP},
  {9, PINMAP_PPM},
};

const uint8_t rxSpecialPinsSize = sizeof(rxSpecialPins);
  
const pinDefine_t pinDefine[] =
  {
    { TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM1 ADC12_IN0
    { TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM2 ADC12_IN1
    { TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM3 ADC12_IN2
    { TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM4 ADC12_IN3
    { TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM5 ADC12_IN6
    { TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM6 ADC12_IN7
    { TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM7 ADC12_IN8
    { TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM8 ADC12_IN9
    { TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM9 RSSI
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PPM
};

const adcDefine_t adcDefine[] =
{
    { ADC_Channel_8, },
    { ADC_Channel_9, },
};

#endif
