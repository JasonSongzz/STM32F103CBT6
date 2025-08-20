/******************************************************************************
 * @file bsp_a22_driver.h
 * @brief DYP-A22YYCW ultrasonic sensor driver interface
 * @details This header file defines the interface and data structures for 
 *          the DYP-A22YYCW ultrasonic distance and temperature sensor driver
 *****************************************************************************/

#ifndef BSP_A22_DRIVER_H
#define BSP_A22_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "mutex.h"

/* DYP-A22YYCW I2C address */
#define A22_I2C_ADDR            0x74

/* DYP-A22YYCW register addresses */
#define A22_REG_DISTANCE_DATA   0x02  /* Distance measurement data register    */
#define A22_REG_TEMP_DATA       0x0A  /* Temperature measurement data register */
#define A22_REG_ANGLE           0x07  /* Measurement angle configuration reg.  */
#define A22_REG_DISTANCE_CMD    0x10  /* Distance measurement command register */
#define A22_REG_TEMP_CMD        0x10  /* Temperature measurement command reg.  */

/* DYP-A22 status enumeration */
typedef enum {
    A22_OK = 0,                       /* Operation completed successfully      */
    A22_ERROR,                        /* General error occurred                */
    A22_ERRORTIMEOUT,                 /* Operation timed out                   */
    A22_ERRORRESOURCE,                /* Resource allocation error             */
    A22_ERRORPARAMETER,               /* Invalid parameter error               */
    A22_ERRORNOMOMORY,                /* Memory allocation error               */
    A22_ERRORISR,                     /* Interrupt service routine error       */
    A22_MAX                           /* Maximum status value (used for bounds */
                                      /* checking)                             */
} a22_status_t;

/* DYP-A22 state enumeration */
typedef enum {
    A22_STATE_IDLE,           /* Idle state                               */
    A22_STATE_START_MEASURE,  /* Start measurement                        */
    A22_STATE_WAITING,        /* Waiting for measurement completion       */
    A22_STATE_READ_DISTANCE,  /* Read distance data                       */
    A22_STATE_READ_TEMP,      /* Read temperature data                    */
    A22_STATE_INIT            /* Initialization state                     */
} a22_state_t;

/* DYP-A22 data structure */
typedef struct {
    uint16_t distance_mm;             /* Distance measurement in millimeters   */
    int16_t temperature;              /* Temperature measurement with 0.1°C    */
                                      /* resolution                            */
    uint32_t timestamp;               /* Timestamp of the last measurement     */
} a22_data_t;

/* DYP-A22 configuration structure */
typedef struct {
    uint8_t a22_address;              /* I2C slave address of the sensor       */
    mutex_t a22_mutex;                /* Mutex for thread-safe access to sens  */
} a22_config_t;

/* DYP-A22 interface structure */
typedef struct {
    void (*init)(void);
    a22_status_t (*update)(void);
    a22_status_t (*trigger)(void);
    void (*get_data)(a22_data_t* data);
} a22_interface_t;

/* DYP-A22 driver structure */
typedef struct {
    a22_config_t config;              /* Configuration parameters for the sens */
    a22_interface_t interface;        /* Interface functions for the sens.     */
    a22_data_t data;                  /* Latest sensor measurement data        */
    a22_state_t state;                /* Current state of the state machine    */
    uint32_t state_timestamp;         /* Timestamp for state transitions       */
    bool read_temperature;            /* Flag to indicate if temperature       */
                                      /* should be read                        */
} a22_driver_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize the DYP-A22YYCW sensor
 * @details Configure the I2C interface and initialize the sensor parameters 
 *          with default I2C address
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void a22_init(void);

/******************************************************************************
 * @brief   Update the A22 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK       State machine processed successfully
 * @retval  A22_ERROR    Error occurred during processing
 *****************************************************************************/
a22_status_t a22_driver_update(void);

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK       Measurement started successfully
 * @retval  A22_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
a22_status_t a22_driver_trigger(void);

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
a22_driver_t a22_get_driver(void);

#endif /* BSP_A22_DRIVER_H */
