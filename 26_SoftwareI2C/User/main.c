#include "stm32f10x.h"  
#include "Delay.h"
#include "OLED.h"
#include "MyI2C.h"

uint8_t KeyNum;

int main()
{
	OLED_Init();
	MyI2C_Init();
	
	MyI2C_Start();
	MyI2C_SendByte(0xD0); // 1101 000 0
	uint8_t Ack = MyI2C_RecieveAck();
	MyI2C_Stop();
	OLED_ShowNum(1, 1, Ack, 3);
	
	while(1)
	{

	}
}
