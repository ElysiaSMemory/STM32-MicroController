#include "stm32f10x.h"                  // Device header

/**
 * @brief Initialize IC with TIM3, CH1
 * @param None
 * @retval None
 */
void IC_Init(void)
{
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// Set UP GPIOA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// DO Internal CLK
	TIM_InternalClockConfig(TIM3);
	
	// SetUP TIM BASE
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// TIM Frequency is 1Hz, 1Hz = 72,000,000 / 7,200 / 10,000 = 1
	// -1 because formula is x + 1
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;	// ARR Reloader
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; // PSC Frequency divider Freq = 72M / 72 = 1MHz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	// Initialize IC
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 		
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	// Dom Sub Settings
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	
	// Enable Clock
	TIM_Cmd(TIM3, ENABLE);
}

uint32_t IC_GetFreq(void)
{
	// FREQ = 72M / (PSC + 1) / (ARR100 + 1) = 1M
	return 1000000 / (TIM_GetCapture1(TIM3) + 1); // +1 is optional
}
