#include "stm32f10x.h"  
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"

uint8_t RxData;

int main()
{
	OLED_Init();
	Serial_Init();
	
	
	OLED_ShowString(1, 1, "RxData: ");
	
	while (1)
	{
		if (Serial_GetRxFlag() == 1)
		{
			RxData = Serial_GetRxData();
			Serial_SendByte(RxData);
			OLED_ShowHexNum(1, 8, RxData, 2);
			// Read Operation will Clear the Flag Automatically
		}		
	}
}


// EASY Way to Recieve
/*
// If recieved data
if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
{
	RxData = USART_ReceiveData(USART1);
	OLED_ShowHexNum(1, 1, RxData, 2);
	// Read Operation will Clear the Flag Automatically
}
*/
