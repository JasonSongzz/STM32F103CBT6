/******************************************************************************
 * @file    TIM3.c
 * @brief   TIM3 hardware interface implementation
 * @details This file implements the TIM3 hardware interface for generating
 *          periodic interrupts and triggering ADC conversions
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "TIM3.h"
#include <stdio.h>

static tim3_callback_t update_callback = NULL;

/******************************************************************************
 * @brief   Initialize TIM3
 * @details Configure TIM3 for 1 second periodic interrupt
 * @param   None
 * @return  None
 *****************************************************************************/
void TIM3Init(void)
{
    /* Enable TIM3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* Calculate timer parameters (1 second timing) */
    uint32_t prescaler = 3600 - 1;
    uint32_t period = 20000 - 1;
    
    /* Timer basic configuration */
    TIM_TimeBaseInitTypeDef TIM_Initstructure;
    TIM_Initstructure.TIM_Period = period;
    TIM_Initstructure.TIM_Prescaler = prescaler;
    TIM_Initstructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_Initstructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_Initstructure);
    
    /* Configure TIM3 TRGO event for ADC triggering */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    
    /* Configure update interrupt (for other processing, such as data storage) */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Start timer */
    TIM_Cmd(TIM3, ENABLE);
}

/******************************************************************************
 * @brief   Set update callback function
 * @details Set the callback function to be called on TIM3 update interrupt
 * @param   callback Callback function pointer
 * @return  None
 *****************************************************************************/
void TIM3_SetUpdateCallback(tim3_callback_t callback)
{
    update_callback = callback;
}

/******************************************************************************
 * @brief   Timer interrupt handler function
 * @details Handle TIM3 update interrupt and call registered callback function
 * @param   None
 * @return  None
 *****************************************************************************/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        if (update_callback != NULL) {
            update_callback();
        }
    }
}
