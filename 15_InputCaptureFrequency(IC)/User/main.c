#include "stm32f10x.h"  
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "IC.h"

uint8_t i;

int main()
{
	OLED_Init();
	PWM_Init();
	IC_Init();
	
	OLED_ShowString(1, 1, "Freq:00000Hz");
	
	PWM_SetPrescaler(720 - 1); // FREQ = 72M / (PSC + 1) / (ARR100 + 1) = 1M
	PWM_SetCompare1(50); // Duty = CCR / (ARR100 + 1)
	while(1)
	{
		OLED_ShowNum(1, 6, IC_GetFreq(), 5);
	}
}
