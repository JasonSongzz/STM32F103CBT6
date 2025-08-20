/******************************************************************************
 * @file    bsp_a22_driver.c
 * @brief   DYP-A22YYCW ultrasonic sensor driver implementation
 * @details This file implements the driver functions for the DYP-A22YYCW 
 *          ultrasonic distance and temperature sensor including 
 *          initialization, data acquisition, and measurement control functions
 *****************************************************************************/
#include "bsp_a22_driver.h"
#include "I2C1.h"
#include "sysTick.h"

#define A22_MEASUREMENT_DELAY_MS 100  /* Measurement delay in milliseconds */

/******************************************************************************
 * @brief   Global DYP-A22YYCW driver instance
 * @details Contains configuration and data structures for the DYP-A22YYCW 
 *          sensor
 *****************************************************************************/
static a22_driver_t g_a22_driver = {0};

/******************************************************************************
 * @brief   Read distance measurement from DYP-A22YYCW sensor
 * @details Read 2-byte distance data from sensor registers and convert to 
 *          millimeters
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK            Successfully read distance data
 * @retval  A22_ERROR         Invalid parameter or communication failure
 * @retval  A22_ERRORTIMEOUT  No echo received (maximum distance or obstruction)
 * @note    Distance value is returned in millimeters (mm)
 * @note    Returns A22_ERRORTIMEOUT when distance reading is 0xFFFF (no echo 
 *          condition)
 *****************************************************************************/
static a22_status_t a22_read_distance(void)
{
    uint8_t data[2];
    
    I2C1_status_t result = I2C1_hw_read(g_a22_driver.config.a22_address,
                                        A22_REG_DISTANCE_DATA, data, 2);
    
    if (result != I2C1_OK) {
        return A22_ERROR;
    } 

    uint16_t distance = (data[0] << 8) | data[1];

    if (distance == 0xFFFF) {
        return A22_ERRORTIMEOUT;
    }
    
    // Lock to protect data update
    mutex_lock(&g_a22_driver.config.a22_mutex);
    g_a22_driver.data.distance_mm = distance;
    mutex_unlock(&g_a22_driver.config.a22_mutex);
    
    return A22_OK;
}

/******************************************************************************
 * @brief   Read temperature measurement from DYP-A22YYCW sensor
 * @details Read 2-byte temperature data from sensor registers and convert to
 *          signed integer with 0.1°C resolution
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK    Successfully read temperature data
 * @retval  A22_ERROR Invalid parameter or communication failure
 * @note    Temperature value is returned with 0.1°C resolution (e.g., 250 = 
 *          25.0°C)
 *****************************************************************************/
static a22_status_t a22_read_temperature(void)
{
    uint8_t data[2];
    
    I2C1_status_t result = I2C1_hw_read(g_a22_driver.config.a22_address,
                                        A22_REG_TEMP_DATA, data, 2);
    
    if (result != I2C1_OK) {
        return A22_ERROR;
    }

    int16_t temperature = (int16_t)((data[0] << 8) | data[1]);
    
    // Lock to protect data update
    mutex_lock(&g_a22_driver.config.a22_mutex);
    g_a22_driver.data.temperature = temperature;
    mutex_unlock(&g_a22_driver.config.a22_mutex);
    
    return A22_OK;
}

/******************************************************************************
 * @brief   Configure the DYP-A22YYCW sensor
 * @details Send configuration command to the sensor to set measurement 
 *          parameters
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK    Successfully configured the sensor
 * @retval  A22_ERROR Communication failure
 *****************************************************************************/
static a22_status_t a22_config(void)
{
    uint8_t cmd = 0x01;

    I2C1_status_t result = I2C1_hw_write(g_a22_driver.config.a22_address,
                                         A22_REG_ANGLE, &cmd, 1);
    
    if (result != I2C1_OK) {
        return A22_ERROR;
    }
    
    return A22_OK;
}

/******************************************************************************
 * @brief   Start distance measurement on DYP-A22YYCW sensor
 * @details Send measurement command to DYP-A22YYCW sensor to initiate a new
 *          distance measurement cycle
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK    Successfully sent measurement command
 * @retval  A22_ERROR Communication failure
 *****************************************************************************/
static a22_status_t a22_start_measurement(void)
{
    uint8_t cmd = 0xBD;

    I2C1_status_t result = I2C1_hw_write(g_a22_driver.config.a22_address,
                                         A22_REG_DISTANCE_CMD, &cmd, 1);
    
    if (result != I2C1_OK) {
        return A22_ERROR;
    }
    
    return A22_OK;
}

/******************************************************************************
 * @brief   Initialize the DYP-A22YYCW sensor
 * @details Configure the I2C interface and initialize the sensor parameters 
 *          with default I2C address
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 *****************************************************************************/
void a22_init(void)
{
    I2C1_hw_init();
    g_a22_driver.config.a22_address = A22_I2C_ADDR;
    mutex_init(&g_a22_driver.config.a22_mutex);
    
    // Initialize state machine
    g_a22_driver.state = A22_STATE_INIT;
    g_a22_driver.state_timestamp = 0;
    g_a22_driver.read_temperature = false; // Default enable temperature read
    
}

/******************************************************************************
 * @brief   Process distance reading
 * @details Read distance data from sensor
 * @param   None
 * @return  a22_status_t Operation status
 *****************************************************************************/
static a22_status_t a22_process_read_distance(void)
{
    a22_status_t status = a22_read_distance();
    
    if (status == A22_OK) {
        // Distance read successful, update timestamp
        mutex_lock(&g_a22_driver.config.a22_mutex);
        g_a22_driver.data.timestamp = get_Tick();
        mutex_unlock(&g_a22_driver.config.a22_mutex);
    }
    
    return status;
}

/******************************************************************************
 * @brief   Process temperature reading
 * @details Read temperature data from sensor
 * @param   None
 * @return  a22_status_t Operation status
 *****************************************************************************/
static a22_status_t a22_process_read_temperature(void)
{
    return a22_read_temperature();
}

/******************************************************************************
 * @brief   Update the A22 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK       State machine processed successfully
 * @retval  A22_ERROR    Error occurred during processing
 *****************************************************************************/
a22_status_t a22_driver_update(void)
{
    a22_status_t status = A22_OK;
    
    switch (g_a22_driver.state) {
        case A22_STATE_IDLE:
            // Idle state, waiting for measurement trigger
            break;
            
        case A22_STATE_INIT:
            // Initialization state
            status = a22_config();
            if (status == A22_OK) {
                g_a22_driver.state = A22_STATE_IDLE;
            } else {
                g_a22_driver.state = A22_STATE_IDLE;
            }
            break;
            
        case A22_STATE_START_MEASURE:
            // Start measurement
            status = a22_start_measurement();
            if (status == A22_OK) {
                g_a22_driver.state_timestamp = get_Tick();
                g_a22_driver.state = A22_STATE_WAITING;
            } else {
                // If measurement start fails, return to idle state
                g_a22_driver.state = A22_STATE_IDLE;
            }
            break;
            
        case A22_STATE_WAITING:
            // Wait for measurement completion
            if ((get_Tick() - g_a22_driver.state_timestamp) >= A22_MEASUREMENT_DELAY_MS) {
                g_a22_driver.state = A22_STATE_READ_DISTANCE;
            }
            break;
            
        case A22_STATE_READ_DISTANCE:
            // Read distance data
            status = a22_process_read_distance();
            if (status != A22_ERROR) {
                // Decide whether to read temperature based on configuration
                if (g_a22_driver.read_temperature) {
                    g_a22_driver.state = A22_STATE_READ_TEMP;
                } else {
                    g_a22_driver.state = A22_STATE_IDLE;
                }
            } else {
                // Return to idle state even if read fails
                g_a22_driver.state = A22_STATE_IDLE;
            }
            break;
            
        case A22_STATE_READ_TEMP:
            // Read temperature data
            status = a22_process_read_temperature();
            if (status != A22_ERROR) {
                g_a22_driver.state = A22_STATE_IDLE;
            } else {
                // Return to idle state even if read fails
                g_a22_driver.state = A22_STATE_IDLE;
            }
            break;
            
        default:
            // Abnormal state, reset to IDLE
            g_a22_driver.state = A22_STATE_IDLE;
            break;
    }
    
    return status;
}

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  a22_status_t Operation status
 * @retval  A22_OK       Measurement started successfully
 * @retval  A22_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
a22_status_t a22_driver_trigger(void)
{
    if (g_a22_driver.state == A22_STATE_IDLE) {
        g_a22_driver.state = A22_STATE_START_MEASURE;
        return A22_OK;
    }
    return A22_ERROR; // Sensor busy, cannot start new measurement
}

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
a22_driver_t a22_get_driver(void)
{
	mutex_lock(&g_a22_driver.config.a22_mutex);
	a22_driver_t ret = g_a22_driver;
	mutex_unlock(&g_a22_driver.config.a22_mutex);
	
	return ret;
}
