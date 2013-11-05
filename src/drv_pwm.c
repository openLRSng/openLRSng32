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
//enum {
//  PWM1 =0,
//  PWM2,
//  PWM3,
//  PWM4,
//  PWM5,
//  PWM6,
//  PWM7,
//  PWM8,
//  PWM9,
//  PWM10,
//  PWM11,
//  PWM12,
//  PPM,
//  MAX_PORTS
//};

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

#if BOARD_TYPE == DTFUHF10CH
static pwmHardware_t timerHardware[] = {
	{ TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM1
	{ TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM2
	{ TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM3
	{ TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM4
	{ TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM5
	{ TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM6
	{ TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM7
	{ TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM8
	{ TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM9 RSSI
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PPM
};
#endif

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

static void lbeepTimeBase(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 8000; //1kHz
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

void enablePWMout(uint8_t ch, bool enable, bool polarity)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = timerHardware[ch].pin;
  if (enable)
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  }
  else
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    if(polarity)
      GPIO_SetBits(timerHardware[ch].gpio, timerHardware[ch].pin); //write the port high
    else
      GPIO_ResetBits(timerHardware[ch].gpio, timerHardware[ch].pin); //write the port low
  }
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(timerHardware[ch].gpio, &GPIO_InitStructure);
}

void setPWM(uint8_t ch, uint16_t value)
{
  if (ch>15) return;
  PPMout[ch] = value;
  uint8_t i;
  for(i=0;i<MAX_OUTPUTS;i++)
  {
	  if (rx_config.pinMapping[i] == ch)
	  {
		  switch (timerHardware[i].channel)
		  {
		  case TIM_Channel_1:
			  timerHardware[i].tim->CCR1 = value;
			  break;
		  case TIM_Channel_2:
			  timerHardware[i].tim->CCR2 = value;
			  break;
		  case TIM_Channel_3:
			  timerHardware[i].tim->CCR3 = value;
			  break;
		  case TIM_Channel_4:
			  timerHardware[i].tim->CCR4 = value;
  	      break;
		  }
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
      timerHardware[PPM_PIN].tim->ARR = out;
      PPMsync -= out;
      if (PPMsync < rx_config.minsync)
      {
		PPMsync = rx_config.minsync;
      }
    }
    else
    {
      timerHardware[PPM_PIN].tim->ARR = PPMsync;
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
	if(port == PPM_PIN)
	{
		TIM_CtrlPWMOutputs(timerHardware[PPM_PIN].tim, ENABLE); //needed only for timer1
	}
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
	pwmTimeBase(timerHardware[PPM_PIN].tim, 1000);
	pwmGPIOConfig(timerHardware[PPM_PIN].gpio, timerHardware[PPM_PIN].pin);
	pwmOCConfig(timerHardware[PPM_PIN].tim, timerHardware[PPM_PIN].channel, 300, 1);
	pwmNVICConfig(timerHardware[PPM_PIN].irq);
	TIM_ITConfig(timerHardware[PPM_PIN].tim, TIM_IT_CC1, ENABLE);
	// Needed only on TIM1
	TIM_CtrlPWMOutputs(timerHardware[PPM_PIN].tim, ENABLE);
	TIM_Cmd(timerHardware[PPM_PIN].tim, ENABLE);
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

void configureLbeepPWM(void)
{
        //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
        //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
        lbeepTimeBase(timerHardware[RSSI_PIN].tim);
        pwmGPIOConfig(timerHardware[RSSI_PIN].gpio, timerHardware[RSSI_PIN].pin);
        pwmOCConfig(timerHardware[RSSI_PIN].tim, timerHardware[RSSI_PIN].channel, 0, 0);
        switch (timerHardware[RSSI_PIN].channel) {
                  case TIM_Channel_1:
                      timerHardware[RSSI_PIN].tim->CCR1 = 4000;
                      break;
                    case TIM_Channel_2:
                      timerHardware[RSSI_PIN].tim->CCR2 = 4000;
                      break;
                    case TIM_Channel_3:
                      timerHardware[RSSI_PIN].tim->CCR3 = 4000;
                      break;
                    case TIM_Channel_4:
                      timerHardware[RSSI_PIN].tim->CCR4 = 4000;
                      break;
        }
}

void enableLbeep(bool enable)
{
  if(enable) {
      TIM_Cmd(timerHardware[RSSI_PIN].tim, ENABLE);
  } else {
      TIM_Cmd(timerHardware[RSSI_PIN].tim, DISABLE);
  }
}

void set_RSSI_output(uint8_t rssi)
{
	if(rx_config.pinMapping[RSSI_PIN] == PINMAP_RSSI)
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
}

