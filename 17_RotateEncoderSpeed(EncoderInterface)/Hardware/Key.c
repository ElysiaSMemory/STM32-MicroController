#include "stm32f10x.h" 
#include "Delay.h "

/**
 * @brief Initialize GPIO IPU Mode for two Switches
 * @param None
 * @retval None
 */
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input Pull UP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief Get the key Number when releasing the Bottom
 * @param None
 * @retval KeyNum Key Number ID
 */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0; // None Pressed
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
	{
		Delay_ms(20); // Wait for stable
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0){}
		Delay_ms(20); // Wait for stable
		KeyNum = 1;
	}

	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
	{
		Delay_ms(20); // Wait for stable
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0){}
		Delay_ms(20); // Wait for stable
		KeyNum = 2;
	}	
	
	return KeyNum;
}
