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

typedef struct
{
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
} pinDefine_t;

typedef struct
{
    uint8_t channel;
} adcDefine_t;

#if BOARD_TYPE == DTFUHF10CH
//PPM_PIN is the only pin connected to timer1. timerHardware[12]
//RSSI_PIN must be the only pin connected to its timer. timerHardware[8]
enum BOARD_PINMAP
{
    CH1_PIN = 0,
    CH2_PIN,
    CH3_PIN,
    CH4_PIN,
    CH5_PIN,
    CH6_PIN,
    CH7_PIN,
    CH8_PIN,
    CH9_PIN,
    CH10_PIN,
    MAX_OUTPUTS
};

//special pins def
extern const uint8_t ANALOG0_PIN;
extern const uint8_t ANALOG1_PIN;
extern const uint8_t RSSI_PIN;
extern const uint8_t PPM_PIN;

extern const pinDefine_t pinDefine[];

extern const adcDefine_t adcDefine[];

#endif
