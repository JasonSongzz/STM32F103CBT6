/**
 * @file TIM3.h
 * @brief TIM3 hardware interface definition
 * @details This header file defines the TIM3 hardware interface for 
 *          generating periodic interrupts and triggering ADC conversions
 */

#ifndef __TIM3_H
#define __TIM3_H

#include "stm32f10x.h"                  /* Device header                 */
#include <stdbool.h>

/* Function pointer type definition */
typedef void (*tim3_callback_t)(void);  /* Timer callback function ptr    */

/* Function declarations */

/******************************************************************************
 * @brief   Initialize TIM3
 * @details Configure TIM3 for 1 second periodic interrupt
 * @param   None
 * @return  None
 *****************************************************************************/
void TIM3Init(void);

/******************************************************************************
 * @brief   Set update callback function
 * @details Set the callback function to be called on TIM3 update interrupt
 * @param   callback Callback function pointer
 * @return  None
 *****************************************************************************/
void TIM3_SetUpdateCallback(tim3_callback_t callback);

#endif /* __TIM3_H */
