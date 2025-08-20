/******************************************************************************
 * @file bsp_tsh54_driver.h
 * @brief TSH54 tachometer sensor driver interface
 * @details This header file defines the interface and data structures for 
 *          the TSH54 tachometer sensor driver
 *****************************************************************************/

#ifndef BSP_TSH54_DRIVER_H
#define BSP_TSH54_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "mutex.h"

/* TSH54 tachometer sensor status enumeration */
typedef enum {
    TSH54_OK = 0,              /* Operation completed successfully          */
    TSH54_ERROR,               /* General error occurred                    */
    TSH54_ERRORTIMEOUT,        /* Operation timed out                       */
    TSH54_ERRORRESOURCE,       /* Resource allocation error                 */
    TSH54_ERRORPARAMETER,      /* Invalid parameter error                   */
    TSH54_ERRORNOMEMORY,       /* Memory allocation error                   */
    TSH54_ERRORISR,            /* Interrupt service routine error           */
    TSH54_MAX                  /* Maximum status value (used for bounds     */
                               /* checking)                                 */
} tsh54_status_t;

/* TSH54 tachometer state enumeration */
typedef enum {
    TSH54_STATE_IDLE,          /* Idle state                                */
    TSH54_STATE_SAMPLING,      /* Sampling state                            */
    TSH54_STATE_PROCESS_DATA,  /* Process data state                        */
    TSH54_STATE_INIT           /* Initialization state                      */
} tsh54_state_t;

/* TSH54 tachometer data structure */
typedef struct {
    uint16_t adc_value;        /* Raw ADC value                             */
    float voltage;             /* Voltage value (V)                         */
    float rpm;                 /* Rotations per minute (RPM)                */
    uint32_t pulse_count;      /* Pulse count                               */
    uint32_t timestamp;        /* Timestamp                                 */
    bool signal_detected;      /* Whether signal is detected                */
} tsh54_data_t;

/* TSH54 tachometer sensor configuration structure */
typedef struct {
    uint8_t adc_channel;       /* ADC channel                               */
    float reference_voltage;   /* Reference voltage (V)                     */
    float sensitivity;         /* Sensor sensitivity (RPM/V)                */
    float threshold;           /* Signal detection threshold (V)            */
    uint32_t sampling_window;  /* Sampling window time (ms)                 */
    mutex_t tsh54_mutex;       /* Mutex for thread-safe access              */
} tsh54_config_t;

/* TSH54 interface structure */
typedef struct {
    void (*init)(void);
    tsh54_status_t (*update)(void);
    tsh54_status_t (*trigger)(void);
    void (*get_data)(tsh54_data_t* data);
} tsh54_interface_t;

/* TSH54 tachometer sensor driver structure */
typedef struct {
    tsh54_config_t config;     /* Configuration parameters for the sensor   */
    tsh54_interface_t interface;    /* Interface functions for the sensor   */
    tsh54_data_t data;         /* Latest sensor measurement data            */
    tsh54_state_t state;       /* Current state of the state machine        */
    uint32_t state_timestamp;  /* Timestamp for state transitions           */
    /* Sampling state variables */
    uint32_t sampling_start_time;     /* Sampling start time                */
    uint32_t last_adc_read_time;      /* Last ADC read time                 */
    bool signal_detected_during_sampling; /* Signal detected flag           */
    bool last_signal_state;           /* Last signal state                  */
} tsh54_driver_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize the TSH54 tachometer sensor
 * @details Configure the TSH54 tachometer sensor with default parameters
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void tsh54_init(void);

/******************************************************************************
 * @brief   Update the TSH54 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  tsh54_status_t Operation status
 * @retval  TSH54_OK       State machine processed successfully
 * @retval  TSH54_ERROR    Error occurred during processing
 *****************************************************************************/
tsh54_status_t tsh54_driver_update(void);

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  tsh54_status_t Operation status
 * @retval  TSH54_OK       Measurement started successfully
 * @retval  TSH54_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
tsh54_status_t tsh54_driver_trigger(void);

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
tsh54_driver_t tsh54_get_driver(void);

#endif /* BSP_TSH54_DRIVER_H */
