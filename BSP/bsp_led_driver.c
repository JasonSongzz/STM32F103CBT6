/******************************************************************************
 * @file    bsp_led_driver.c
 * @brief   LED driver interface implementation
 * @details This file implements the LED driver functionality for controlling 
 *          multiple LEDs with various modes including on/off, blinking and 
 *          breathing
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "bsp_led_driver.h"
#include "sysTick.h"
#include "LED.h"

/******************************************************************************
 * @brief   Global LED driver instance
 * @details Contains all LED instances and hardware interface functions
 *****************************************************************************/
static led_driver_t g_led_driver = {0};

/******************************************************************************
 * @brief   Initialize a single LED instance
 * @details Set default values for LED instance parameters
 * @param   led Pointer to LED instance to initialize [in,out]
 * @param   id  LED identifier for this instance
 * @return  None
 * @note    This is an internal function and should not be called directly by 
 *          user
 *****************************************************************************/
static void led_init_single(led_instance_t* led, led_id_t id)
{
    led->id = id;
    led->mode = LED_MODE_OFF;
    led->state = false;
    led->last_tick = 0;
    led->blink_period = 0;
    led->on_time = 0;
    led->off_time = 0;
}

/******************************************************************************
 * @brief   Initialize LED driver with custom hardware functions
 * @details Set up LED driver with custom hardware initialization and control 
 *          functions
 * @param   hw_init_func Pointer to hardware initialization function
 * @param   hw_set_func  Pointer to hardware LED control function
 * @return  None
 * @note    This function initializes all LED instances and sets them to OFF 
 *          state
 * @see     led_hw_init
 * @see     led_hw_set
 *****************************************************************************/
static void led_driver_init(void (*hw_init_func)(led_id_t), 
                            void (*hw_set_func)(led_id_t, bool))
{
    g_led_driver.hw_init = hw_init_func;
    g_led_driver.hw_set = hw_set_func;
    
    for (int i = 0; i < LED_ID_MAX; i++) {
        if (g_led_driver.hw_init) {
            g_led_driver.hw_init((led_id_t)i);
        }
        led_init_single(&g_led_driver.leds[i], (led_id_t)i);
        if (g_led_driver.hw_set) {
            g_led_driver.hw_set((led_id_t)i, false); 
        }
    }
}

/******************************************************************************
 * @brief   Initialize LED driver
 * @details Initialize the LED driver with default hardware functions
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup
 * @see     led_driver_init
 *****************************************************************************/
void led_init(void)
{
    led_driver_init(led_hw_init, led_hw_set);
}

/******************************************************************************
 * @brief   Update LED states
 * @details Process LED state changes based on current mode and timing 
 *          parameters. This function should be called periodically (e.g., in 
 *          main loop or timer interrupt)
 * @param   None
 * @return  None
 * @note    This function handles blinking and breathing LED modes
 * @see     led_instance_t
 *****************************************************************************/
void led_driver_update(void)
{
    for (int i = 0; i < LED_ID_MAX; i++) {
        led_instance_t* led = &g_led_driver.leds[i];
        bool new_state = led->state;
        
        uint32_t current_tick = get_Tick();
        
        switch (led->mode) {
            case LED_MODE_OFF:
                new_state = false;
                break;
                
            case LED_MODE_ON:
                new_state = true;
                break;
                
            case LED_MODE_BLINK_SLOW:
            case LED_MODE_BLINK_FAST:
            case LED_MODE_BLINK_ERROR:
            case LED_MODE_BREATHE:
                if (led->blink_period > 0) {
                    uint32_t elapsed = current_tick - led->last_tick;
                    if (led->state && elapsed >= led->on_time) {
                        new_state = false;
                        led->last_tick = current_tick;
                    } else if (!led->state && elapsed >= led->off_time) {
                        new_state = true;
                        led->last_tick = current_tick;
                    }
                }
                break;
                
            default:
                new_state = false;
                break;
        }
        
        if (new_state != led->state) {
            led->state = new_state;
            if (g_led_driver.hw_set) {
                g_led_driver.hw_set(led->id, new_state);
            }
        }
    }
}

/******************************************************************************
 * @brief   Set LED mode
 * @details Configure LED operation mode (ON, OFF, blinking, etc.)
 * @param   led_id Identifier of the LED to configure
 * @param   mode   LED mode to set
 * @return  None
 * @note    Predefined modes include blinking, error indication and breathing
 *          effect
 * @see     led_mode_t
 *****************************************************************************/
void led_driver_set_mode(led_id_t led_id, led_mode_t mode)
{
    if (led_id >= LED_ID_MAX || mode >= LED_MODE_MAX) {
        return;
    }
    
    led_instance_t* led = &g_led_driver.leds[led_id];
    led->mode = mode;
    led->last_tick = 0; 
    
    switch (mode) {
        case LED_MODE_OFF:
            led->blink_period = 0;
            led->state = false;
            if (g_led_driver.hw_set) {
                g_led_driver.hw_set(led_id, false);
            }
            break;
            
        case LED_MODE_ON:
            led->blink_period = 0;
            led->state = true;
            if (g_led_driver.hw_set) {
                g_led_driver.hw_set(led_id, true);
            }
            break;
            
        case LED_MODE_BLINK_SLOW:
            led->blink_period = 1000;   /* 1000ms */
            led->on_time = 500;
            led->off_time = 500;
            break;
            
        case LED_MODE_BLINK_FAST:
            led->blink_period = 200;    /* 200ms */
            led->on_time = 100;
            led->off_time = 100;
            break;
            
        case LED_MODE_BLINK_ERROR:
            led->blink_period = 100;    /* 100ms */
            led->on_time = 50;
            led->off_time = 50;
            break;
            
        case LED_MODE_BREATHE:
            led->blink_period = 2000;   /* 2000ms */
            led->on_time = 1000;
            led->off_time = 1000;
            break;
            
        default:
            break;
    }
}

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
                                 uint32_t off_time)
{
    if (led_id >= LED_ID_MAX) {
        return;
    }
    
    led_instance_t* led = &g_led_driver.leds[led_id];
    led->mode = LED_MODE_BLINK_SLOW; 
    led->on_time = on_time;
    led->off_time = off_time;
    led->blink_period = on_time + off_time;
    led->last_tick = 0;
}

/******************************************************************************
 * @brief   Get current LED mode
 * @details Retrieve the current operation mode of specified LED
 * @param   led_id Identifier of the LED to query
 * @return  led_mode_t Current LED mode
 * @retval  LED_MODE_OFF if LED is off or invalid LED ID
 * @retval  Other LED modes as currently configured
 * @see     led_mode_t
 *****************************************************************************/
led_mode_t led_driver_get_mode(led_id_t led_id)
{
    if (led_id >= LED_ID_MAX) {
        return LED_MODE_OFF;
    }
    return g_led_driver.leds[led_id].mode;
}

/******************************************************************************
 * @brief   Get current LED state
 * @details Retrieve the current physical state (on/off) of specified LED
 * @param   led_id Identifier of the LED to query
 * @return  bool Current LED state
 * @retval  true  if LED is currently on
 * @retval  false if LED is currently off or invalid LED ID
 *****************************************************************************/
bool led_driver_get_state(led_id_t led_id)
{
    if (led_id >= LED_ID_MAX) {
        return false;
    }
    return g_led_driver.leds[led_id].state;
}
