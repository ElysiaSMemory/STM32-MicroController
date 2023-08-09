#include "stm32f10x.h"                  // Device header

// extern uint16_t Num;

/**
 * @brief Initializing TIM2 to send interrupt every 1 second
 * @param None
 * @retval None
 */
void Timer_Init(void)
{
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// DO Internal CLK
	TIM_InternalClockConfig(TIM2);
	
	// SetUP TIM BASE
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// TIM Frequency is 1Hz, 1Hz = 72,000,000 / 7,200 / 10,000 = 1
	// -1 because formula is x + 1
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;	// ARR Reloader
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1; // PSC Frequency divider
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
		// Add this to make sure flag is not set immediately
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	
	// Enable interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	// NVIC setup
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	// Enable Timer
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief TIM2 interrupt function
 * @param None
 * @retval None
 */
/*

// Interrupt Code
void TIM2_IRQHandler(void)
{
	// Check Legit
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		// Code here
		Num ++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

*/
