#include "stm32f10x.h"                  // Device header


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
	
	// Set GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	// Initialize ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// Interrupt and WatchDog
		// Code Here
		
	// Turn On ADC
	ADC_Cmd(ADC1, ENABLE);
	
	// ADC Calibration
	ADC_ResetCalibration(ADC1);
		// When hardware clear the flag we are good to go
	while (ADC_GetResetCalibrationStatus(ADC1) == SET); // Until FInished
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET); // Until FInished	
	
	
	// ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Continuous
}


/**
 * @brief Get the AD value
 * @param None
* @retval uint16_t: ADC value
 */
uint16_t AD_GetValue(uint8_t ADC_Channel)
{
	// Congif ADC to Regular Mode， CHOOSE CHANNEL
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);

	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	// Check Flag and Clear Flag
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET); // Until FInished, 55.5 + 12.5 = 68T, (1/12M)(68) = 5.6us to finish	
	// ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	
	return ADC_GetConversionValue(ADC1);
}
