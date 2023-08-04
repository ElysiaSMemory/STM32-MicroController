#include "stm32f10x.h"                  // Device header

// extern uint16_t Num;

/**
 * @brief Initializing TIM2 to send interrupt based on external CLK input
 * @param None
 * @retval None
 */
void Timer_Init(void)
{
	// TIM2 is a part of APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Enable GPIOA PIN0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	// DO EXTRN CLK
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x0F);
	
	
	// SetUP TIM BASE
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// TIM Frequency is 1Hz, 1Hz = 72,000,000 / 7,200 / 10,000 = 1
	// -1 because formula is x + 1
	TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;	// ARR Reloader
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // PSC Frequency divider
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
 * @brief Get the current CNT value in TIM2
 * @param None
 * @retval uint16_t CNT value
 */
uint16_t Timer_GetCounter(void)
{
	return TIM_GetCounter(TIM2);
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
