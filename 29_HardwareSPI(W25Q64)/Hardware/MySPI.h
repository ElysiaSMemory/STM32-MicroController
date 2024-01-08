#ifndef __MYSPI_H__
#define __MYSPI_H__
/**
 * @brief Set status of SS - Start Communication or not
 * @param BitValue: 0/1
 * @retval None
 */
void MySPI_W_SS(uint8_t BitValue);

/**
 * @brief Initialize SPI COM
 * @param None
 * @retval None
 */
void MySPI_Init(void);

/**
 * @brief Send Start Segment
 * @param None
 * @retval None
 */
void MySPI_Start(void);

/**
 * @brief Send End Segment
 * @param None
 * @retval None
 */
void MySPI_Stop(void);

/**
 * @brief Swap 1 byte of information with the SPI device
 * @param ByteSend: Byte going out from Master
 * @retval Byte Comming from Sub
 */
uint8_t MySPI_SwapByte(uint8_t ByteSend);
#endif
