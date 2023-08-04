#include "stm32f10x.h"  
#include "Delay.h"

int main()
{
	// Turn On the CLK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// Set up GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; // All 16 are set PP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
	while(1)
	{
		GPIO_Write(GPIOA, ~(0x0001 << 0)); // 0000 0000 0000 0001
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 1)); // 0000 0000 0000 0010
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 2)); // 0000 0000 0000 0100
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 3)); // 0000 0000 0000 1000
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 4)); // 0000 0000 0001 0000
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 5)); // 0000 0000 0010 0000
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 6)); // 0000 0000 0100 0000
		Delay_ms(500);
		GPIO_Write(GPIOA, ~(0x0001 << 7)); // 0000 0000 1000 0000
		Delay_ms(500);
	}
}
