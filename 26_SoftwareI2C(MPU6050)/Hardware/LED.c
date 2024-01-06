#include "stm32f10x.h"                  // Device header

/**
 * @brief Initialize GPIO for two LEDs
 * @param None
 * @retval None
 */
void LED_Init(void)
{
	// Set UP GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
}


/**
 * @brief Turn ON LED 1
 * @param None
 * @retval None
 */
void LED1_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}


/**
 * @brief Turn OFF LED 1
 * @param None
 * @retval None
 */
void LED1_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
}


/**
 * @brief Reverse state of LED1
 * @param None
 * @retval None
 */
void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0)
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	else 
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}


/**
 * @brief Turn ON LED 2
 * @param None
 * @retval None
 */
void LED2_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}


/**
 * @brief Turn OFF LED 2
 * @param None
 * @retval None
 */
void LED2_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}


/**
 * @brief Reverse state of LED1
 * @param None
 * @retval None
 */
void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) == 0)
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	else 
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}
