#include "stm32f10x.h"                  // Device header

uint16_t AD_Value[4];

/**
 * @brief Initialize ADC1 to Not continuous and Not scan, adn then calibrate it
 * @param None
 * @retval None 
 */
void AD_Init(void)
{
	// Enable GPIO CLK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// Enable ADC1 CLK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		// ADCCLK DIV6, 73M to 13MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	// DMA CLK Enable 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	
	// Set GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 4ADC Channel (Menu)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);


	// Initialize ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Scan
	ADC_InitStructure.ADC_NbrOfChannel = 4;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// Interrupt and WatchDog
		// Code Here
		
	// Initialize DMA (Waiter)
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1 -> DR; // 0x4001 244C
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16bits
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		// Counter
	DMA_InitStructure.DMA_BufferSize = 4;
		// Auto refill
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	// Turn On DMA and Path
	ADC_DMACmd(ADC1, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);


	// Turn On ADC
	ADC_Cmd(ADC1, ENABLE);
	
	// ADC Calibration
	ADC_ResetCalibration(ADC1);
		// When hardware clear the flag we are good to go
	while (ADC_GetResetCalibrationStatus(ADC1) == SET); // Until FInished
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET); // Until FInished	
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
}


///**
// * @brief Get the AD value
// * @param None
//* @retval uint16_t: ADC value
// */
//void AD_GetValue(void)
//{	
//	// Refill counter in DMA
//	DMA_Cmd(DMA1_Channel1, DISABLE);
//	DMA_SetCurrDataCounter(DMA1_Channel1, 4);
//	DMA_Cmd(DMA1_Channel1, ENABLE);	

//	
//	// Wait for completion DMA
//	while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
//		// Clear Flag
//	DMA_ClearFlag(DMA1_FLAG_TC1);

//}
