#include "stm32f10x.h"    // Device header

int16_t Encoder_Count;

void Encoder_Init(void)
{
	// RCC enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	// Set up GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Setup AFIO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
	
	// Setup EXTI
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Falling Edge Trigger - Remove Block Between
	EXTI_Init(&EXTI_InitStructure);

	// Setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 2 bits for pre-emption priority 2 bits for subpriority
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// PBB0, A
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 1~3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	// PB1, B
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 1~3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief Get the total Amount Changed after last call of this function
 * @param None
 * @retval Temp Temporary variable that stores the amount changed
 */
int16_t Encoder_Get(void)
{
	int16_t Temp;
	Temp = Encoder_Count;
	Encoder_Count = 0;
	return Temp;
}


void EXTI0_IRQHandler(void)
{
	// Check Flag is right?
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		
		// Code Here
		// Turn BackWard: A is Going Down, and B is Low
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
		{
			Encoder_Count --;
		}
		
		// Clear Flag
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

}


void EXTI1_IRQHandler(void)
{
	// Check Flag is right?
	if (EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		
		// Code Here
		// Turn Foward: B is Going Down, and A is Low
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
		{
			Encoder_Count ++;
		}
		
		// Clear Flag
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
