/******************************************************************************
 * @file    ADC.c
 * @brief   ADC hardware interface implementation
 * @details This file implements the ADC hardware interface for reading analog
 *          values from various sensors in the system
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "ADC.h"

/******************************************************************************
 * @brief   Initialize ADC hardware
 * @details Configure GPIO pins and ADC peripheral for analog input
 * @param   None
 * @return  None
 *****************************************************************************/
void adc_hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(PT100_RCC | WATER_RCC | SPEED_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  

    GPIO_InitStructure.GPIO_Pin = PT100_PIN;
    GPIO_Init(PT100_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = WATER_PIN;
    GPIO_Init(WATER_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPEED_PIN;
    GPIO_Init(SPEED_PORT, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    /* Changed to software trigger */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    /* No need to enable external trigger */
}

/******************************************************************************
 * @brief   Read ADC value
 * @details Read analog value from specified ADC channel
 * @param   channel ADC channel to read
 * @return  uint16_t ADC conversion result
 *****************************************************************************/
uint16_t adc_hw_read(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    return ADC_GetConversionValue(ADC1);
}
