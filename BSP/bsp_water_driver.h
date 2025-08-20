/******************************************************************************
 * @file bsp_water_driver.h
 * @brief Water immersion rope sensor driver interface
 * @details This header file defines the interface and data structures for 
 *          the water immersion rope sensor driver
 *****************************************************************************/

#ifndef BSP_WATER_DRIVER_H
#define BSP_WATER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "mutex.h"

/* Water immersion status enumeration */
typedef enum {
    WATER_OK = 0,              /* Operation completed successfully          */
    WATER_ERROR,               /* General error occurred                    */
    WATER_ERRORTIMEOUT,        /* Operation timed out                       */
    WATER_ERRORRESOURCE,       /* Resource allocation error                 */
    WATER_ERRORPARAMETER,      /* Invalid parameter error                   */
    WATER_ERRORNOMOMORY,       /* Memory allocation error                   */
    WATER_ERRORISR,            /* Interrupt service routine error           */
    WATER_MAX                  /* Maximum status value (used for bounds     */
                               /* checking)                                 */
} water_status_t;

/* Water immersion state enumeration */
typedef enum {
    WATER_STATE_IDLE,          /* Idle state                                */
    WATER_STATE_READ_ADC,      /* Reading ADC state                         */
    WATER_STATE_CALCULATE,     /* Calculating resistance state              */
    WATER_STATE_DETERMINE,     /* Determining water status state            */
    WATER_STATE_INIT           /* Initialization state                      */
} water_state_t;

/* Water immersion sensor data structure */
typedef struct {
    uint16_t adc_value;        /* ADC raw value                             */
    float voltage;             /* Voltage value (V)                         */
    float resistance;          /* Resistance value (Ohms)                   */
    water_status_t status;     /* Water immersion status                    */
    uint32_t timestamp;        /* Timestamp of last measurement             */
} water_data_t;

/* Water immersion sensor configuration structure */
typedef struct {
    uint8_t adc_channel;       /* ADC channel                               */
    float reference_voltage;   /* Reference voltage (V)                     */
    float series_resistor;     /* Series resistor value (Ohms)              */
    float threshold_dry;       /* Dry threshold (V)                         */
    float threshold_wet;       /* Wet threshold (V)                         */
    float threshold_submerged; /* Submerged threshold (V)                   */
    mutex_t water_mutex;       /* Mutex for thread-safe access              */
} water_config_t;

/* Water interface structure */
typedef struct {
    void (*init)(void);
    water_status_t (*update)(void);
    water_status_t (*trigger)(void);
    void (*get_data)(water_data_t* data);
} water_interface_t;

/* Water immersion sensor driver structure */
typedef struct {
    water_config_t config;     /* Configuration parameters for the sensor   */
    water_interface_t interface;    /* Interface functions for the sensor   */
    water_data_t data;         /* Latest sensor measurement data            */
    water_state_t state;       /* Current state of the state machine        */
    uint32_t state_timestamp;  /* Timestamp for state transitions           */
} water_driver_t;

/* Water immersion status types */
typedef enum {
    WATER_STATUS_DRY = 0,      /* Dry state                                 */
    WATER_STATUS_WET,          /* Wet state                                 */
    WATER_STATUS_SUBMERGED,    /* Submerged state                           */
    WATER_STATUS_ERROR         /* Error state                               */
} water_immersion_status_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize the water immersion sensor
 * @details Configure the ADC interface and initialize the sensor parameters
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void water_init(void);

/******************************************************************************
 * @brief   Update the water sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  water_status_t Operation status
 * @retval  WATER_OK       State machine processed successfully
 * @retval  WATER_ERROR    Error occurred during processing
 *****************************************************************************/
water_status_t water_driver_update(void);

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  water_status_t Operation status
 * @retval  WATER_OK       Measurement started successfully
 * @retval  WATER_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
water_status_t water_driver_trigger(void);

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
water_driver_t water_get_driver(void);

/******************************************************************************
 * @brief   Get current water immersion status
 * @details Determine water immersion status based on voltage thresholds
 * @param   voltage Voltage value from sensor
 * @return  water_immersion_status_t Water immersion status
 *****************************************************************************/
water_immersion_status_t water_get_immersion_status(float voltage);

#endif /* BSP_WATER_DRIVER_H */
