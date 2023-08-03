#include "stm32f10x.h"  
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"

uint16_t Num;

int main()
{
	OLED_Init();
	Timer_Init();
	OLED_ShowString(1, 1, "NUM: ");

	while(1)
	{
		OLED_ShowNum(1, 5, Num, 5);
		OLED_ShowNum(2, 5, TIM_GetCounter(TIM2), 5);
	}
}


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
