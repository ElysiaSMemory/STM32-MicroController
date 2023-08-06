#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Set UP GPIOA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		// Play with Pin reMap
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); // Change PA0 to PA15
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // Change this to 15
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// DO Internal CLK
	TIM_InternalClockConfig(TIM2);
	
	// SetUP TIM BASE
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// TIM Frequency is 1Hz, 1Hz = 72,000,000 / 7,200 / 10,000 = 1
	// -1 because formula is x + 1
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;	// ARR Reloader
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1; // PSC Frequency divider
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	// Enable OC
	TIM_OCInitTypeDef TIM_OCInitStructure;
		// Give Default Value because not all members are used
	TIM_OCStructInit(&TIM_OCInitStructure);
		// Members
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50; // Set CCR
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	// Enable Timer
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Set the CCR of PWM
 * @param Compare： New value for CCR
 * @retval None
 */
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);
}
