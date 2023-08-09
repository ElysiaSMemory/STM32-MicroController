#include "stm32f10x.h"                  // Device header

/**
 * @brief Initialize Encoder Path with No inversion, count both AB edges
 * @param None
 * @retval None
 */
void Encoder_Init(void)
{
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// Set UP GPIOA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	// DO Internal CLK // NO NEED INTERNAL CLK
//	TIM_InternalClockConfig(TIM3);
	
	// SetUP TIM BASE
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // USELESS
	// TIM Frequency is 1Hz, 1Hz = 72,000,000 / 7,200 / 10,000 = 1
	// -1 because formula is x + 1
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;	// ARR Reloader, MAX
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // PSC Frequency divider: ENCODER TIM runs CNT directly 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	// Initialize IC
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	// CH 1
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);	
	// CH 2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	// Encoder Config
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); // RISING is not Invert
	
	// ENABLE TIM
	TIM_Cmd(TIM3, ENABLE);
}


int16_t Encoder_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;
}
