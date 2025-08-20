/******************************************************************************
 * @file bsp_pt100_driver.h
 * @brief PT100 platinum resistance temperature sensor driver interface
 * @details This header file defines the interface and data structures for 
 *          the PT100 resistance temperature detector (RTD) sensor driver
 *****************************************************************************/

#ifndef BSP_PT100_DRIVER_H
#define BSP_PT100_DRIVER_H

#include <stdint.h>
#include "mutex.h"

/* PT100 sensor status enumeration */
typedef enum {
    PT100_OK = 0,                /* Operation completed successfully         */
    PT100_ERROR,                 /* General error occurred                   */
    PT100_ERRORTIMEOUT,          /* Operation timed out                      */
    PT100_ERRORRESOURCE,         /* Resource allocation error                */
    PT100_ERRORPARAMETER,        /* Invalid parameter error                  */
    PT100_ERRORNOMOMORY,         /* Memory allocation error                  */
    PT100_ERRORISR,              /* Interrupt service routine error          */
    PT100_MAX                    /* Maximum status value (used for bounds    */
                                 /* checking)                                */
} pt100_status_t;

/* PT100 state enumeration */
typedef enum {
    PT100_STATE_IDLE,            /* Idle state                               */
    PT100_STATE_READ_ADC,        /* Reading ADC state                        */
    PT100_STATE_CALCULATE,       /* Calculating temperature state            */
    PT100_STATE_INIT             /* Initialization state                     */
} pt100_state_t;

/* PT100 calculation method enumeration */
typedef enum {
    PT100_CALC_LOOKUP_TABLE = 0, /* Use lookup table method for temp. calc.  */
    PT100_CALC_LINEAR,           /* Use linear approximation method for temp */
    PT100_CALC_MAX               /* Maximum calculation method value         */
} pt100_calc_method_t;

/* PT100 data structure */
typedef struct {
    uint16_t adc_value;          /* Raw ADC value                            */
    float voltage;               /* Voltage value (V)                        */
    float resistance;            /* Resistance value (ohms)                  */
    float temperature;           /* Temperature value (degrees Celsius)      */
    uint32_t timestamp;          /* Timestamp of last measurement            */
} pt100_data_t;

/* PT100 configuration structure */
typedef struct {
    uint8_t adc_channel;         /* ADC channel                              */
    float reference_voltage;     /* Reference voltage (V)                    */
    float series_resistor;       /* Series resistor value (ohms)             */
    pt100_calc_method_t calc_method;   /* Temperature calculation method     */
    mutex_t pt100_mutex;         /* Mutex for thread-safe access             */
} pt100_config_t;

/* PT100 interface structure */
typedef struct {
    void (*init)(void);
    pt100_status_t (*update)(void);
    pt100_status_t (*trigger)(void);
    void (*get_data)(pt100_data_t* data);
} pt100_interface_t;

/* PT100 driver structure */
typedef struct {
    pt100_config_t config;       /* Configuration parameters for the sensor  */
    pt100_interface_t interface; /* Interface functions for the sensor       */
    pt100_data_t data;           /* Latest sensor measurement data           */
    pt100_state_t state;         /* Current state of the state machine       */
    uint32_t state_timestamp;    /* Timestamp for state transitions          */
} pt100_driver_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize the PT100 sensor
 * @details Configure the PT100 sensor with default parameters
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void pt100_init(void);

/******************************************************************************
 * @brief   Update the PT100 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK       State machine processed successfully
 * @retval  PT100_ERROR    Error occurred during processing
 *****************************************************************************/
pt100_status_t pt100_driver_update(void);

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK       Measurement started successfully
 * @retval  PT100_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
pt100_status_t pt100_driver_trigger(void);

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
pt100_driver_t pt100_get_driver(void);

/******************************************************************************
 * @brief   Set PT100 calculation method
 * @details Configure which method to use for temperature calculation
 * @param   method Calculation method to use
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK    Successfully set calculation method
 * @retval  PT100_ERROR Invalid method parameter
 *****************************************************************************/
pt100_status_t pt100_set_calc_method(pt100_calc_method_t method);

#endif /* BSP_PT100_DRIVER_H */
