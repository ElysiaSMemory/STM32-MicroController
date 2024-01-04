#ifndef __MPU6050_H__
#define __MPU6050_H__

/**
  * @brief  Custom Event Observer that has timeout control
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_EVENT: Event Type
  * @retval None
  */
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

/**
  * @brief  Write one byte to the registor on 6050
  * @param  RegAddress: Register address on 6050
  * @param  Data: Data to be written
  * @retval None
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/**
  * @brief  Read one byte from 6050
  * @param  RegAddress: Address to be read from
  * @retval Data in the desired register
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/**
  * @brief  Get 6050 Device ID
  * @retval Address of this 6050
  */
uint8_t MPU6050_GetID(void);

/**
  * @brief  Initialize 6050
  * @retval None
  */
void MPU6050_Init(void);

/**
  * @brief  Get all 6 data from 6050
  * @param  Pointers to different storages...
  * @retval Data in the desired register
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
	int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
