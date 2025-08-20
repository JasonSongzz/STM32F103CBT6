/**
 * @file ADC.h
 * @brief ADC hardware interface definition
 * @details This header file defines the ADC hardware interface for reading 
 *          analog values from various sensors in the system
 */

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include <stdint.h>

/* ADC channel definitions */
#define ADC_CHANNEL_PT100    ADC_Channel_9   /* PB1 - PT100                  */
#define ADC_CHANNEL_WATER    ADC_Channel_8   /* PB0 - Water sensor           */
#define ADC_CHANNEL_TACHO    ADC_Channel_7   /* PA7 - Tachometer sensor      */

/* ADC pin definitions */
#define PT100_RCC            RCC_APB2Periph_GPIOB  /* PT100 GPIO clock      */
#define PT100_PORT           GPIOB                 /* PT100 GPIO port       */
#define PT100_PIN            GPIO_Pin_1            /* PT100 GPIO pin        */

#define WATER_RCC            RCC_APB2Periph_GPIOB  /* Water sensor GPIO clock */
#define WATER_PORT           GPIOB                 /* Water sensor GPIO port  */
#define WATER_PIN            GPIO_Pin_0            /* Water sensor GPIO pin   */

#define SPEED_RCC            RCC_APB2Periph_GPIOA  /* Speed sensor GPIO clock */
#define SPEED_PORT           GPIOA                 /* Speed sensor GPIO port  */
#define SPEED_PIN            GPIO_Pin_7            /* Speed sensor GPIO pin   */

/* Function declarations */

/******************************************************************************
 * @brief   Initialize ADC hardware
 * @details Configure GPIO pins and ADC peripheral for analog input
 * @param   None
 * @return  None
 *****************************************************************************/
void adc_hw_init(void);

/******************************************************************************
 * @brief   Read ADC value
 * @details Read analog value from specified ADC channel
 * @param   channel ADC channel to read
 * @return  uint16_t ADC conversion result
 *****************************************************************************/
uint16_t adc_hw_read(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
