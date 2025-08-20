/**
 * @file LED.c
 * @brief LED hardware interface implementation
 * @details This file implements the LED hardware interface for controlling
 *          various LEDs in the system
 */

#include "LED.h"

/**
 * @brief   Initialize LED hardware
 * @details Configure GPIO pins for LED control
 * @param   led_id LED identifier
 * @return  None
 */
void led_hw_init(led_id_t led_id)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    switch(led_id) {
        case LED_ID_STATUS:
            RCC_APB2PeriphClockCmd(STATUS_LED_RCC, ENABLE);
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Pin = STATUS_LED_PIN;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(STATUS_LED_PORT, &GPIO_InitStructure);
            break;
           
        default:
            break;
    }
}

/**
 * @brief   Set LED state
 * @details Turn LED on or off
 * @param   led_id LED identifier
 * @param   state  LED state (true = on, false = off)
 * @return  None
 */
void led_hw_set(led_id_t led_id, bool state)
{
    switch(led_id) {
        case LED_ID_STATUS:
            if(state) {
                GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN);
            } else {
                GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN);
            }
            break;
            
        default:
            break;
    }
}
