#include "stm32f10x.h"                  // Device header
#include "PWM.h"


/**
 * @brief Initialize Motor, PA4,5 Control Direction; TIM2Channel 3 output PWM
 * @param None
 * @retval None
 */
void Motor_Init(void)
{
	// GPIO for direction
	// Set UP GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	PWM_Init();
}

/**
 * @brief Set the speed of motor
 * @param Speed Range -100 ~ 100, speed to motor
 * @retval None
 */
void Motor_SetSpeed(int8_t Speed)
{
	// + Direction
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3(Speed);
	}
	// - Direction
	else
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		PWM_SetCompare3(- Speed);
	}
}
