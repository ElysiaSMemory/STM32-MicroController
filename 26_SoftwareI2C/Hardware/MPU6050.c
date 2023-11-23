#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();
	
	// Sub-Address + Write -> ACK
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_RecieveAck();
	
	MyI2C_SendByte(RegAddress);
	MyI2C_RecieveAck();
	
	MyI2C_SendByte(Data);
	MyI2C_RecieveAck();
	
	MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	// Tell Sub the desired adress to read from
	MyI2C_Start();
	
	// Sub-Address + Read -> ACK
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_RecieveAck();
	
	MyI2C_SendByte(RegAddress);
	MyI2C_RecieveAck();
	
	// Read the address
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_RecieveAck();
	
	Data = MyI2C_RecieveByte();
	MyI2C_SendAck(1); // No ACK: This stop MPU sending data
	MyI2C_Stop();
	
	return Data;
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


void MPU6050_Init(void)
{
	MyI2C_Init();
	// Initialize regisers
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); // Gyro CLK
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); // Sampling rate, smaller the faster
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06); // Low Pass filter 000001110 (Most flat filter)
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 00011000 (Max Measurement Range)
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
	int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	// Read ACC X
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	// Read ACC Y
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	// Read ACC Z
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	// Read Gyro X
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	// Read Gyro Y
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	// Read Gyro Z
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
	
	// More efficent if  real multiple registers at once...
	
}
