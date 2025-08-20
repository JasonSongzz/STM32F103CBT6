/**
 * @file bsp_led_driver.h
 * @brief LED driver interface
 */

#ifndef BSP_LED_DRIVER_H
#define BSP_LED_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* LED ID enumeration */
typedef enum {
    LED_ID_STATUS = 0,                /* Status LED                        */
    LED_ID_MAX                        /* Maximum LED ID value              */
} led_id_t;

/* LED mode enumeration */
typedef enum {
    LED_MODE_OFF = 0,                 /* LED off                           */
    LED_MODE_ON,                      /* LED on                            */
    LED_MODE_BLINK_SLOW,              /* Slow blinking                     */
    LED_MODE_BLINK_FAST,              /* Fast blinking                     */
    LED_MODE_BLINK_ERROR,             /* Error indication blinking         */
    LED_MODE_BREATHE,                 /* Breathing effect                  */
    LED_MODE_MAX                      /* Maximum mode value                */
} led_mode_t;

/* LED instance structure */
typedef struct {
    led_id_t id;                      /* LED identifier                    */
    led_mode_t mode;                  /* LED operation mode                */
    bool state;                       /* Current LED state                 */
    uint32_t last_tick;               /* Last state change timestamp       */
    uint32_t blink_period;            /* Blink period in milliseconds      */
    uint32_t on_time;                 /* LED on duration in milliseconds   */
    uint32_t off_time;                /* LED off duration in milliseconds  */
} led_instance_t;

/* LED driver structure (using function pointers for decoupling) */
typedef struct {
    /* Hardware operation function pointers */
    void (*hw_init)(led_id_t led_id);          /* Hardware init function   */
    void (*hw_set)(led_id_t led_id, bool state); /* Hardware control func. */
    
    /* LED instance array */
    led_instance_t leds[LED_ID_MAX];           /* LED instances            */
} led_driver_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize LED driver
 * @details Initialize the LED driver with default hardware functions
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup
 *****************************************************************************/
void led_init(void); 

/******************************************************************************
 * @brief   Update LED states
 * @details Process LED state changes based on current mode and timing 
 *          parameters. This function should be called periodically (e.g., in 
 *          main loop or timer interrupt)
 * @param   None
 * @return  None
 * @note    This function handles blinking and breathing LED modes
 *****************************************************************************/
void led_driver_update(void);

/******************************************************************************
 * @brief   Set LED mode
 * @details Configure LED operation mode (ON, OFF, blinking, etc.)
 * @param   led_id Identifier of the LED to configure
 * @param   mode   LED mode to set
 * @return  None
 * @note    Predefined modes include blinking, error indication and breathing
 *          effect
 *****************************************************************************/
void led_driver_set_mode(led_id_t led_id, led_mode_t mode);

/******************************************************************************
 * @brief   Set custom LED blinking pattern
 * @details Configure LED with custom on/off timing for blinking
 * @param   led_id   Identifier of the LED to configure
 * @param   on_time  LED on duration in milliseconds
 * @param   off_time LED off duration in milliseconds
 * @return  None
 * @note    This function sets the LED mode to LED_MODE_BLINK_SLOW internally
 *****************************************************************************/
void led_driver_set_custom_blink(led_id_t led_id, uint32_t on_time, 
                                 uint32_t off_time);

/******************************************************************************
 * @brief   Get current LED mode
 * @details Retrieve the current operation mode of specified LED
 * @param   led_id Identifier of the LED to query
 * @return  led_mode_t Current LED mode
 * @retval  LED_MODE_OFF if LED is off or invalid LED ID
 * @retval  Other LED modes as currently configured
 *****************************************************************************/
led_mode_t led_driver_get_mode(led_id_t led_id);

/******************************************************************************
 * @brief   Get current LED state
 * @details Retrieve the current physical state (on/off) of specified LED
 * @param   led_id Identifier of the LED to query
 * @return  bool Current LED state
 * @retval  true  if LED is currently on
 * @retval  false if LED is currently off or invalid LED ID
 *****************************************************************************/
bool led_driver_get_state(led_id_t led_id);

#endif /* BSP_LED_DRIVER_H */
