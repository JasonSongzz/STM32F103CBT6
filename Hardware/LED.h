/**
 * @file LED.h
 * @brief LED hardware interface definition
 * @details This header file defines the LED hardware interface for 
 *          controlling various LEDs in the system
 */

#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"                  /* Device header                 */
#include "bsp_led_driver.h"
#include <stdbool.h>

/* LED pin definitions */
#define STATUS_LED_RCC   RCC_APB2Periph_GPIOB  /* Status LED GPIO clock    */
#define STATUS_LED_PORT  GPIOB                 /* Status LED GPIO port     */
#define STATUS_LED_PIN   GPIO_Pin_12           /* Status LED GPIO pin      */

/* Function declarations */

/******************************************************************************
 * @brief   Initialize LED hardware
 * @details Configure GPIO pins for LED control
 * @param   led_id LED identifier
 * @return  None
 *****************************************************************************/
void led_hw_init(led_id_t led_id);

/******************************************************************************
 * @brief   Set LED state
 * @details Turn LED on or off
 * @param   led_id LED identifier
 * @param   state  LED state (true = on, false = off)
 * @return  None
 *****************************************************************************/
void led_hw_set(led_id_t led_id, bool state);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
