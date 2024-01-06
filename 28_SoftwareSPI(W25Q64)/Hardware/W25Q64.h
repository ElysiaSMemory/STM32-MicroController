#ifndef __W25Q64_H__
#define __W25Q64_H__
/**
 * @brief Initialize W25Q64
 * @param None
 * @retval None
 */
void W25Q64_Init(void);

/**
 * @brief Get Device Manufacture and ID/Capacity through pointer
 * @param MID: Pointer storing Manufacture
 * @param DID: Device ID and Capacity
 * @retval None
 */
void W25Q64_ReadID(uint8_t *MID, uint16_t *DID);

/**
 * @brief Enable Write
 * @param None
 * @retval None
 */
void W25Q64_WriteEnable(void);

/**
 * @brief Wait until not busy
 * @param None
 * @retval None
 */
void W25Q64_WaitBusy(void);

/**
 * @brief Program a page
 * @param Address: Address of target
 * @param DataArray: Data in an array to be overried
 * @param Count: How many item in array
 * @retval None
 */
void W25Q64_PageProgram(uint32_t Address, uint8_t *DataArray, uint16_t Count);


/**
 * @brief Erase a sector
 * @param Address: Address of target
 * @retval None
 */
void W25Q64_SectorErase(uint32_t Address);

/**
 * @brief Read Data starting from a location
 * @param Address: Address of target
 * @param DataArray: Data to be stored
 * @param Count: How many item to read
 * @retval None
 */
void W25Q64_ReadData(uint32_t Address, uint8_t *DataArray, uint32_t Count);

#endif
