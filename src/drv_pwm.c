#include "board.h"

volatile uint16_t PPMout[16]; // 12PWM upto 16 PPM
volatile uint16_t PPMsync;
volatile uint8_t  PPMch=0;
static uint16_t activeChannels = 16;

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
  GPIO_InitStructure.GPIO_Pin = pinDefine[ch].pin;
  if (enable)
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  }
  else
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    if(polarity)
      GPIO_SetBits(pinDefine[ch].gpio, pinDefine[ch].pin); //write the port high
    else
      GPIO_ResetBits(pinDefine[ch].gpio, pinDefine[ch].pin); //write the port low
  }
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(pinDefine[ch].gpio, &GPIO_InitStructure);
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
		  switch (pinDefine[i].channel)
		  {
		  case TIM_Channel_1:
		    pinDefine[i].tim->CCR1 = value;
			  break;
		  case TIM_Channel_2:
		    pinDefine[i].tim->CCR2 = value;
			  break;
		  case TIM_Channel_3:
		    pinDefine[i].tim->CCR3 = value;
			  break;
		  case TIM_Channel_4:
		    pinDefine[i].tim->CCR4 = value;
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
      pinDefine[PPM_PIN].tim->ARR = out;
      PPMsync -= out;
      if (PPMsync < rx_config.minsync)
      {
		PPMsync = rx_config.minsync;
      }
    }
    else
    {
        pinDefine[PPM_PIN].tim->ARR = PPMsync;
      PPMch=0;
    }
  }
}

void configureServoPWM(uint8_t port)
{
	// PWM outputs
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	pwmTimeBase(pinDefine[port].tim, 20000);
	pwmGPIOConfig(pinDefine[port].gpio, pinDefine[port].pin);
	pwmOCConfig(pinDefine[port].tim, pinDefine[port].channel, 0, 0);
	if(port == PPM_PIN)
	{
		TIM_CtrlPWMOutputs(pinDefine[PPM_PIN].tim, ENABLE); //needed only for timer1
	}
	TIM_Cmd(pinDefine[port].tim, ENABLE);
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
	pwmTimeBase(pinDefine[PPM_PIN].tim, 1000);
	pwmGPIOConfig(pinDefine[PPM_PIN].gpio, pinDefine[PPM_PIN].pin);
	pwmOCConfig(pinDefine[PPM_PIN].tim, pinDefine[PPM_PIN].channel, 300, 1);
	pwmNVICConfig(pinDefine[PPM_PIN].irq);
	TIM_ITConfig(pinDefine[PPM_PIN].tim, TIM_IT_CC1, ENABLE);
	// Needed only on TIM1
	TIM_CtrlPWMOutputs(pinDefine[PPM_PIN].tim, ENABLE);
	TIM_Cmd(pinDefine[PPM_PIN].tim, ENABLE);
}

void configureRssiPWM(void)
{
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	rssiTimeBase(pinDefine[RSSI_PIN].tim);
	pwmGPIOConfig(pinDefine[RSSI_PIN].gpio, pinDefine[RSSI_PIN].pin);
	pwmOCConfig(pinDefine[RSSI_PIN].tim, pinDefine[RSSI_PIN].channel, 0, 0);
	TIM_Cmd(pinDefine[RSSI_PIN].tim, ENABLE);
}

void configureLbeepPWM(void)
{
        //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
        //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
        lbeepTimeBase(pinDefine[RSSI_PIN].tim);
        pwmGPIOConfig(pinDefine[RSSI_PIN].gpio, pinDefine[RSSI_PIN].pin);
        pwmOCConfig(pinDefine[RSSI_PIN].tim, pinDefine[RSSI_PIN].channel, 0, 0);
        switch (pinDefine[RSSI_PIN].channel) {
                  case TIM_Channel_1:
                    pinDefine[RSSI_PIN].tim->CCR1 = 4000;
                      break;
                    case TIM_Channel_2:
                      pinDefine[RSSI_PIN].tim->CCR2 = 4000;
                      break;
                    case TIM_Channel_3:
                      pinDefine[RSSI_PIN].tim->CCR3 = 4000;
                      break;
                    case TIM_Channel_4:
                      pinDefine[RSSI_PIN].tim->CCR4 = 4000;
                      break;
        }
}

void enableLbeep(bool enable)
{
  if(enable) {
      TIM_Cmd(pinDefine[RSSI_PIN].tim, ENABLE);
  } else {
      TIM_Cmd(pinDefine[RSSI_PIN].tim, DISABLE);
  }
}

void set_RSSI_output(uint8_t rssi)
{
	if(rx_config.pinMapping[RSSI_PIN] == PINMAP_RSSI)
	{
	switch (pinDefine[RSSI_PIN].channel) {
	    case TIM_Channel_1:
	      pinDefine[RSSI_PIN].tim->CCR1 = rssi;
	      break;
	    case TIM_Channel_2:
	      pinDefine[RSSI_PIN].tim->CCR2 = rssi;
	      break;
	    case TIM_Channel_3:
	      pinDefine[RSSI_PIN].tim->CCR3 = rssi;
	      break;
	    case TIM_Channel_4:
	      pinDefine[RSSI_PIN].tim->CCR4 = rssi;
	      break;
	    }
	}
}

