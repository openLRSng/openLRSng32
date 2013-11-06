#pragma once
#include "board.h"

#define RX_FLYTRON8CH 0x01
#define RX_OLRSNG4CH  0x02
#define RX_OLRSNG12CH 0x03
#define RX_DTFUHF10CH 0x04


//*******************************
//  SELECT HARDWARE HERE
//set this to the hardware type you are compiling for
#define BOARD_TYPE DTFUHF10CH //DTF UHF 10ch
//*******************************

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
} pinDefine_t;

#if BOARD_TYPE == DTFUHF10CH
//PPM_PIN is the only pin connected to timer1. timerHardware[12]
//RSSI_PIN must be the only pin connected to its timer. timerHardware[8]
enum BOARD_PINMAP {
	CH1_PIN = 0,
	CH2_PIN,
	CH3_PIN,
	CH4_PIN,
	CH5_PIN,
	CH6_PIN,
	CH7_PIN,
	CH8_PIN,
	RSSI_PIN,
	PPM_PIN,
	MAX_OUTPUTS
};

static pinDefine_t pinDefine[] = {
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

#endif
