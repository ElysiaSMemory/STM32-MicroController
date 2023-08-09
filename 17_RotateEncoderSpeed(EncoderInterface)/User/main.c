#include "stm32f10x.h"  
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"

int16_t Speed;

int main()
{
	OLED_Init();
	Encoder_Init();
	Timer_Init();
	OLED_ShowString(1, 1, "Speed: ");

	while(1)
	{
		OLED_ShowSignedNum(1, 7, Speed, 5);
	}
}


void TIM2_IRQHandler(void)
{
	// Check Legit
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		// Code here
		Speed = Encoder_Get(); // Get CNT every second
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
