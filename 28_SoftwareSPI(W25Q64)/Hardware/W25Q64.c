#include "stm32f10x.h"                  // Device header
#include "MySPI.h"
#include "W25Q64_Ins.h"

void W25Q64_Init(void)
{
	MySPI_Init();
	
}

void W25Q64_ReadID(uint8_t *MID, uint16_t *DID)
{
	MySPI_Start();
	MySPI_SwapByte(W25Q64_JEDEC_ID); // Command for Device information
	*MID = MySPI_SwapByte(W25Q64_DUMMY_BYTE); // Send useless, used for recieve
	*DID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);
	*DID <<=8; // Move High 8 to left
	*DID |= MySPI_SwapByte(W25Q64_DUMMY_BYTE);
	MySPI_Stop();
}

void W25Q64_WriteEnable(void)
{
	MySPI_Start();
	MySPI_SwapByte(W25Q64_WRITE_ENABLE);
	MySPI_Stop();
}

void W25Q64_WaitBusy(void)
{
	uint32_t Timeout;
	
	MySPI_Start();
	MySPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);
	Timeout = 100000;
	// Wait until LSB of Status Register BUSY goes 0
	while((MySPI_SwapByte(W25Q64_DUMMY_BYTE) & 0x01) == 0x01)
	{
		Timeout --;
		if (Timeout == 0)
			break;
	}
	MySPI_Stop();
}

void W25Q64_PageProgram(uint32_t Address, uint8_t *DataArray, uint16_t Count)
{
	uint16_t i;
	W25Q64_WriteEnable();
	MySPI_Start();
	MySPI_SwapByte(W25Q64_PAGE_PROGRAM);
	// Send Address
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8); // Higher 8 bits are automatically truncated
	MySPI_SwapByte(Address); // Higher 8 bits are automatically truncated
	// Send Data
	for (i = 0; i < Count; ++i)
	{
		MySPI_SwapByte(DataArray[i]);
	}
	
	MySPI_Stop();
	W25Q64_WaitBusy();
}

void W25Q64_SectorErase(uint32_t Address)
{
	W25Q64_WriteEnable();
	MySPI_Start();
	MySPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);
	// Send Address
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8); // Higher 8 bits are automatically truncated
	MySPI_SwapByte(Address); // Higher 8 bits are automatically truncated
	MySPI_Stop();
	W25Q64_WaitBusy();
}

void W25Q64_ReadData(uint32_t Address, uint8_t *DataArray, uint32_t Count)
{
	uint32_t i;
	MySPI_Start();
	MySPI_SwapByte(W25Q64_READ_DATA);
	// Send Address
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8); // Higher 8 bits are automatically truncated
	MySPI_SwapByte(Address); // Higher 8 bits are automatically truncated
	// Start Reading
	for (i = 0; i < Count; ++ i)
	{
		DataArray[i] = MySPI_SwapByte(W25Q64_DUMMY_BYTE);
	}

	MySPI_Stop();
}
