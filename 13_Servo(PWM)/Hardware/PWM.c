#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Set UP GPIOA1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // Change this to 15
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
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;	// ARR Reloader
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; // PSC Frequency divider
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
	TIM_OCInitStructure.TIM_Pulse = 0; // Set CCR 500 - 2500， 0.5 to 2.5 ms
	
	// TIM2_CH2
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	// Enable Timer
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Set the CCR of PWM
 * @param Compare： New value for CCR
 * @retval None
 */
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);
}
