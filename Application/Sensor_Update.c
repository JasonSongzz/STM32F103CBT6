#include "Sensor_Update.h"
#include "bsp_led_driver.h"
#include "bsp_pt100_driver.h"
#include "bsp_water_driver.h"
#include "bsp_tsh54_driver.h"
#include "bsp_adxl345_driver.h"
#include "bsp_a22_driver.h"
#include "sysTick.h"

#define SENSOR_TRIGGER_INTERVAL 5000  /* 5 seconds */

/* Sensor task related variables */
static uint32_t s_last_trigger_time = 0;

/******************************************************************************
 * @brief   Initialize all sensors
 * @details Initialize all connected sensors in the system
 * @param   None
 * @return  None
 *****************************************************************************/
void sensor_driver_init(void)
{
    /* Initialize all sensors */
    led_init();
    pt100_init();
    water_init();
    tsh54_init();
    adxl345_init();
    a22_init();
    
    led_driver_set_mode(LED_ID_STATUS, LED_MODE_BLINK_SLOW);  
}

/******************************************************************************
 * @brief   Trigger measurements for all sensors
 * @details Start measurement process for all sensors
 * @param   None
 * @return  None
 *****************************************************************************/
static void sensor_driver_trigger(void)
{
    /* Trigger all sensors */
    pt100_driver_trigger();
    water_driver_trigger();
    tsh54_driver_trigger();
    adxl345_driver_trigger();
    a22_driver_trigger();
}

/******************************************************************************
 * @brief   Update all sensors state machines
 * @details Process state machines for all sensors
 * @param   None
 * @return  None
 *****************************************************************************/
static void sensor_driver_update(void)
{
    /* Update all sensors state machines */
    led_driver_update();
    pt100_driver_update();
    water_driver_update();
    tsh54_driver_update();
    adxl345_driver_update();
    a22_driver_update();
}

/******************************************************************************
 * @brief   Main sensor task function
 * @details Periodically trigger sensors and update their state machines
 * @param   None
 * @return  None
 * @note    This function should be called regularly in the main loop
 *****************************************************************************/
void Sensor_Task(void)
{
    /* Non-blocking periodic sensor triggering */
    if ((get_Tick() - s_last_trigger_time) >= SENSOR_TRIGGER_INTERVAL) {
        sensor_driver_trigger();
        s_last_trigger_time = get_Tick();
    }
        
    /* Continuously update sensor state machines */
    sensor_driver_update();
}
