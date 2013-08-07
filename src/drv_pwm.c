#include "board.h"


/* RX32 PWM lauyout
    PWM1  PA0 TIM2_CH1
    PWM2  PA1 TIM2_CH2
    PWM3  PA2 TIM2_CH3
    PWM4  PA3 TIM2_CH4
    PWM5  PA6 TIM3_CH1
    PWM6  PA7 TIM3_CH2
    PWM7  PB0 TIM3_CH3
    PWM8  PB1 TIM3_CH4
    PWM9  PB9 TIM4_CH4
    PWM10 PB8 TIM4_CH3
    PWM11 PB7 TIM4_CH2
    PWM12 PB6 TIM4_CH1
    PPM   PA8 TIM1_CH1
*/

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
  PWM1 =0,
  PWM2,
  PWM3,
  PWM4,
  PWM5,
  PWM6,
  PWM7,
  PWM8,
  PWM9,
  PWM10,
  PWM11,
  PWM12,
  PPM,
  MAX_PORTS
};

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
} pwmHardware_t;

volatile uint16_t PPMout[16]; // 12PWM upto 16 PPM
volatile uint16_t PPMsync;
volatile uint8_t  PPMch=0;
static uint16_t activeChannels = 16;

/*
 * this doesn't seem to match the schematic. it's backwards!
static pwmHardware_t timerHardware[] = {
    { TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn, 0, },          // PWM1
    { TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn, 0, },          // PWM2
    { TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn, 0, },          // PWM3
    { TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM4
    { TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM5
    { TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM6
    { TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM7
    { TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM8
    { TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM9
    { TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM10
    { TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM11
    { TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM12
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PPM
};
*/
static pwmHardware_t timerHardware[] = {
	{ TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM1
	{ TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM2
	{ TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM3
	{ TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM4
	{ TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM5
	{ TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM6
	{ TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM7
	{ TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM8
	{ TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM9
	{ TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn, 0, },          // PWM10
	{ TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn, 0, },          // PWM11
    { TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn, 0, },          // PWM12
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PPM
};


static void pwmTimeBase(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // all timers run at 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void rssiTimeBase(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 255; //31kHz
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 8000000) - 1; // 8MHz timer
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, bool inverted)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = inverted?TIM_OCPolarity_High:TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);
}

void enablePPMout(bool enable)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = timerHardware[PPM].pin;
  if (enable)
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  }
  else
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_SetBits(timerHardware[PPM].gpio, timerHardware[PPM].pin);
  }
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(timerHardware[PPM].gpio, &GPIO_InitStructure);
}

void setPWM(uint8_t ch, uint16_t value)
{
  if (ch>15) return;
  PPMout[ch] = value;
  if ((ch<12) & (rx_config.pinMapping[ch] == ch)) {
    switch (timerHardware[ch].channel) {
    case TIM_Channel_1:
      timerHardware[ch].tim->CCR1 = value;
      break;
    case TIM_Channel_2:
      timerHardware[ch].tim->CCR2 = value;
      break;
    case TIM_Channel_3:
      timerHardware[ch].tim->CCR3 = value;
      break;
    case TIM_Channel_4:
      timerHardware[ch].tim->CCR4 = value;
      break;
    }
  }
}

static void pwmNVICConfig(uint8_t irq)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = irq;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM1_CC_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    if (PPMch<activeChannels)
    {
      uint16_t out = PPMout[PPMch++];
      timerHardware[PPM].tim->ARR = out;
      PPMsync -= out;
      if (PPMsync < rx_config.minsync)
      {
		PPMsync = rx_config.minsync;
      }
    }
    else
    {
      timerHardware[PPM].tim->ARR = PPMsync;
      PPMch=0;
    }
  }
}

void configureServoPWM(uint8_t port)
{
	// PWM outputs
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	pwmTimeBase(timerHardware[port].tim, 20000);
	pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin);
	pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, 0, 0);
	TIM_Cmd(timerHardware[port].tim, ENABLE);
 }

void configurePPM(uint8_t ppmChannels)
{
	activeChannels = ppmChannels;
	if(rx_config.flags & PPM_MAX_8CH)
		activeChannels = 8;
	PPMch = activeChannels;
	PPMsync = rx_config.minsync + 300;
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	pwmTimeBase(timerHardware[PPM].tim, 1000);
	pwmGPIOConfig(timerHardware[PPM].gpio, timerHardware[PPM].pin);
	pwmOCConfig(timerHardware[PPM].tim, timerHardware[PPM].channel, 300, 1);
	pwmNVICConfig(timerHardware[PPM].irq);
	TIM_ITConfig(timerHardware[PPM].tim, TIM_IT_CC1, ENABLE);
	// Needed only on TIM1
	TIM_CtrlPWMOutputs(timerHardware[PPM].tim, ENABLE);
	TIM_Cmd(timerHardware[PPM].tim, ENABLE);
}

void configureRssiPWM(void)
{
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	rssiTimeBase(timerHardware[RSSI_PIN].tim);
	pwmGPIOConfig(timerHardware[RSSI_PIN].gpio, timerHardware[RSSI_PIN].pin);
	pwmOCConfig(timerHardware[RSSI_PIN].tim, timerHardware[RSSI_PIN].channel, 0, 0);
	TIM_Cmd(timerHardware[RSSI_PIN].tim, ENABLE);
}

void set_RSSI_output(uint8_t rssi)
{
	switch (timerHardware[RSSI_PIN].channel) {
	    case TIM_Channel_1:
	      timerHardware[RSSI_PIN].tim->CCR1 = rssi;
	      break;
	    case TIM_Channel_2:
	      timerHardware[RSSI_PIN].tim->CCR2 = rssi;
	      break;
	    case TIM_Channel_3:
	      timerHardware[RSSI_PIN].tim->CCR3 = rssi;
	      break;
	    case TIM_Channel_4:
	      timerHardware[RSSI_PIN].tim->CCR4 = rssi;
	      break;
	    }
}

