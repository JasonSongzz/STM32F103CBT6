/******************************************************************************
 * @file bsp_adxl345_driver.h
 * @brief ADXL345 accelerometer sensor driver interface
 * @details This header file defines the interface and data structures for 
 *          the ADXL345 digital accelerometer sensor driver
 *****************************************************************************/

#ifndef BSP_ADXL345_DRIVER_H
#define BSP_ADXL345_DRIVER_H

#include <stdint.h>
#include "mutex.h"

/* ADXL345 I2C address */
#define ADXL345_I2C_ADDR            0x53

/* ADXL345 register addresses */
#define ADXL345_REG_DEVID           0x00  /* Device ID register                */
#define ADXL345_REG_POWER_CTL       0x2D  /* Power control register            */
#define ADXL345_REG_DATA_FORMAT     0x31  /* Data format control register      */
#define ADXL345_REG_DATAX0          0x32  /* X-axis data 0 (LSB)               */
#define ADXL345_REG_DATAX1          0x33  /* X-axis data 1 (MSB)               */
#define ADXL345_REG_DATAY0          0x34  /* Y-axis data 0 (LSB)               */
#define ADXL345_REG_DATAY1          0x35  /* Y-axis data 1 (MSB)               */
#define ADXL345_REG_DATAZ0          0x36  /* Z-axis data 0 (LSB)               */
#define ADXL345_REG_DATAZ1          0x37  /* Z-axis data 1 (MSB)               */
#define ADXL345_REG_FIFO_CTL        0x38  /* FIFO control register             */
#define ADXL345_REG_FIFO_STATUS     0x39  /* FIFO status register              */

/* ADXL345 status enumeration */
typedef enum {
    ADXL345_OK = 0,                    /* Operation completed successfully     */
    ADXL345_ERROR,                     /* General error occurred               */
    ADXL345_ERRORTIMEOUT,              /* Operation timed out                  */
    ADXL345_ERRORRESOURCE,             /* Resource allocation error            */
    ADXL345_ERRORPARAMETER,            /* Invalid parameter error              */
    ADXL345_ERRORNOMOMORY,             /* Memory allocation error              */
    ADXL345_ERRORISR,                  /* Interrupt service routine error      */
    ADXL345_MAX                        /* Maximum status value                 */
} adxl345_status_t;

/* ADXL345 state enumeration */
typedef enum {
    ADXL345_STATE_IDLE,                /* Idle state                           */
    ADXL345_STATE_INIT,                /* Initialization state                 */
    ADXL345_STATE_READ_DATA,           /* Reading data state                   */
    ADXL345_STATE_CONFIGURE            /* Configuration state                  */
} adxl345_state_t;

/* ADXL345 data structure */
typedef struct {
    int16_t x;                         /* X-axis acceleration in mg            */
    int16_t y;                         /* Y-axis acceleration in mg            */
    int16_t z;                         /* Z-axis acceleration in mg            */
    uint32_t timestamp;                /* Timestamp of the last measurement    */
} adxl345_data_t;

/* ADXL345 configuration structure */
typedef struct {
    uint8_t i2c_address;               /* I2C slave address of the sensor      */
    mutex_t adxl345_mutex;             /* Mutex for thread-safe access         */
} adxl345_config_t;

/* ADXL345 interface structure */
typedef struct adxl345_interface {
    void (*init)(void);
    adxl345_status_t (*update)(void);
    adxl345_status_t (*trigger)(void);
    void (*get_data)(adxl345_data_t* data);
} adxl345_interface_t;

/* ADXL345 driver structure */
typedef struct {
    adxl345_config_t config;           /* Configuration parameters             */
    adxl345_interface_t interface;     /* Interface functions                  */
    adxl345_data_t data;               /* Latest sensor measurement data       */
    adxl345_state_t state;             /* Current state of the state machine   */
    uint32_t state_timestamp;          /* Timestamp for state transitions      */
} adxl345_driver_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize the ADXL345 sensor
 * @details Configure the I2C interface and initialize the sensor parameters 
 *          with default I2C address
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void adxl345_init(void);

/******************************************************************************
 * @brief   Update the ADXL345 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       State machine processed successfully
 * @retval  ADXL345_ERROR    Error occurred during processing
 * @note    This function should be called regularly in the main loop
 *****************************************************************************/
adxl345_status_t adxl345_driver_update(void);

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       Measurement started successfully
 * @retval  ADXL345_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
adxl345_status_t adxl345_driver_trigger(void);

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
adxl345_driver_t adxl345_get_driver(void);

#endif /* BSP_ADXL345_DRIVER_H */
