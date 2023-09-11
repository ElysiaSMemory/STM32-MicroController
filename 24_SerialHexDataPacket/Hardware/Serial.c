#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];
uint8_t Serial_RxFlag;


/**
 * @brief  Initialize Serial COM
 * 9600 Baud, 8 bits, no parity, no flow, only send, 1 stop bit
 * @param None
 * @retval None
 */
void Serial_Init(void)
{
	// ENABLE USART1 CLK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	// GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// Initialize GOIOA9 to AF_PP, USART out to COMPUTER
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Initialize GOIOA10 to IPU, COMPUTER to USART
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Initialize USART1
	// 9600 Baud, 8 bits, no parity, no flow, only send, 1 stop bit
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // Enable Both
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	// Enable Interrupt upon recieving
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		// Setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	
	// ENABLE USART
	USART_Cmd(USART1, ENABLE);
}

/**
 * @brief Send One byte out from USART
 * @param Byte: Data to be sent
 * @retval None
 */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	// Check When Finished
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	// No need to clear flag, next call flag is unset
}


/**
 * @brief Sent an arra out USART
 * @param Array: Pointer to the head of array
 * @param Length: Length of the array
 * @retval None
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}


/**
 * @brief Send A string 
 * @param String: String To be sent C-String
 * @retval None
 */
void Serial_SendString(char *String)
{
	uint16_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

/**
 * @brief Get X ^ Y
 * @param X: Base
 * @param Y: Power
 * @retval result of mathematical expression
 */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	if (Y == 0) return 1;
	return X * Serial_Pow(X, Y - 1);
}

/**
 * @brief Send a number in text mode
 * @param Number: Num to be sent
 * @param Length: Length of Num
 * @retval None
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	while (Length --)
	{
		Serial_SendByte((Number / Serial_Pow(10, Length) % 10) + '0');
	}
}

/**
 * @brief Align Prinf to serial com, this is the basis of printf
 * printf now outputnto com
 * @param Unknown
 * @retval Unknown
 */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}


/**
 * @brief Custom Printf
 * @param Unknown
 * @retval Unknown
 */
void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg; 
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	
	Serial_SendString(String);
}

/**
 * @brief Send A Packet with beginning FF, and ending with FE
 * @param None
 * @retval None
 */
void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 4);
	Serial_SendByte(0xFE);
}

/**
 * @brief Return Rx Flag
 * @param void
 * @retval State of Flag
 */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}


/**
 * @brief Interrupt Functon called when USART recieved data
 * @details State machine implementation, 3 states
 * @param None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0; // Current index of filling array
	// Check Flag
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		// Using State Machine
			// Waiting For Head
		if (RxState == 0)
		{
			if (RxData == 0xFF)
			{
				RxState = 1;
				pRxPacket = 0; // Clear array index
			}
		}
			// Recieving Data
		else if (RxState == 1)
		{
			Serial_RxPacket[pRxPacket] = RxData; // Fill one element in array
			pRxPacket ++;
			if (pRxPacket >= 4) // When array is full
			{
				RxState = 2;
			}
		}
			// Waiting For End
		else if (RxState == 2)
		{
			if (RxData == 0xFE)
			{
				RxState = 0;
				Serial_RxFlag = 1;
			}
		}
		
		// No need to clear flag if read
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}


