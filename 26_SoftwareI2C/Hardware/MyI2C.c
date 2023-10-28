#include "stm32f10x.h"                  // Device header
#include "Delay.h"

// Lets do this without macro
/**
 * @brief Write to SCL port
 * @param BitVaue: Either 0 or 1
 * @retval None
 */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
	Delay_us(10); // Delay to make MPU keep up the speed
}

/**
 * @brief Write to SDA port
 * @param BitVaue: Either 0 or 1
 * @retval None
 */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
	Delay_us(10); // Delay to make MPU keep up the speed
}

/**
 * @brief Read the bit value at SDA
 * @param None
 * @retval Value at SDA
 */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
	Delay_us(10); // Delay to make MPU keep up the speed
	return BitValue;
}


/**
 * @brief Initialize I2C with B10 and B11, release I2C bus
 * @param None
 * @retval None
 */
void MyI2C_Init(void)
{
	// Set SDA SCL to be Open Drain
		// Set UP GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // OD can also input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //Pin B10 and B11
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Set SDA SCL to be HIGH - Beginning of I2C sentence
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11); // I2C is in idle
}

/**
 * @brief Start Section of I2C COM / Compatible with ReStart
 * @param None
 * @retval None
 */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);	// This go first: Consider the case we need restart
	// In case of restart, SCL is low. SDA is uncertain
	MyI2C_W_SCL(1);

	
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);

}

/**
 * @brief Stop part of I2C COM
 * @param None
 * @retval None
 */
void MyI2C_Stop(void)
{
	// SDA is uncertain, hence we make it low to ensure rising edge
	MyI2C_W_SDA(0);
	
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

/**
 * @brief Send a byte to Sub
 * When SCL low, put data on line; Sub read data when SCL is high
 * @param Byte: Data to be sent
 * @retval None
 */
void MyI2C_SendByte(uint8_t Byte)
{
	for (uint8_t i = 0; i < 8; ++ i)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}


/**
 * @brief Recieve a byte from sub
 * @param None
 * @retval recieved byte
 */
uint8_t MyI2C_RecieveByte(void)
{
	uint8_t Byte = 0x00;
	// Coming in SCL is low
	MyI2C_W_SDA(1); // Release SDA -> Sub put data on SDA

	for (uint8_t i = 0; i < 7; ++ i)
	{
		MyI2C_W_SCL(1); // Super read data
		if (MyI2C_R_SDA() == 1) {Byte |= (0x80 >> i);}
		// if SDA == 0; since Byte is initialized to be all 0. This dosent matter
		MyI2C_W_SCL(0); // "I'm finished"
	}
	return Byte;
}

// ACKS
/**
 * @brief Send Acknowledgement to Sub
 * @param AckBit: 0/1 to be sent
 * @retval None
 */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}


/**
 * @brief Recieve a byte from sub
 * @param None
 * @retval recieved byte
 */
uint8_t MyI2C_RecieveAck(void)
{
	uint8_t AckBit = 0x00;
	// Coming in SCL is low
	MyI2C_W_SDA(1); // Release SDA -> Sub put data on SDA

	MyI2C_W_SCL(1); // Super read data
	AckBit = MyI2C_R_SDA();
	// if SDA == 0; since Byte is initialized to be all 0. This dosent matter
	MyI2C_W_SCL(0); // "I'm finished"

	return AckBit;
}
