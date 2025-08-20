/******************************************************************************
 * @file    bsp_adxl345_driver.c
 * @brief   ADXL345 accelerometer sensor driver implementation
 * @details This file implements the driver functions for the ADXL345 
 *          digital accelerometer sensor including initialization, data 
 *          acquisition, and measurement control functions
 *****************************************************************************/
#include "bsp_adxl345_driver.h"
#include "I2C.h"
#include "sysTick.h"

#define ADXL345_MEASUREMENT_DELAY_MS 10  /* Measurement delay in milliseconds */

/******************************************************************************
 * @brief   Global ADXL345 driver instance
 * @details Contains configuration and data structures for the ADXL345 sensor
 *****************************************************************************/
static adxl345_driver_t g_adxl345_driver = {0};

/******************************************************************************
 * @brief   Read acceleration data from ADXL345 sensor
 * @details Read 6-byte acceleration data from sensor registers and convert to 
 *          milli-g values
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       Successfully read acceleration data
 * @retval  ADXL345_ERROR    Invalid parameter or communication failure
 *****************************************************************************/
static adxl345_status_t adxl345_read_acceleration(void)
{
    uint8_t data[6];
    
    I2C_status_t result = I2C_hw_read(g_adxl345_driver.config.i2c_address,
                                        ADXL345_REG_DATAX0, data, 6);
    
    if (result != I2C_OK) {
        return ADXL345_ERROR;
    }

    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);
    
    mutex_lock(&g_adxl345_driver.config.adxl345_mutex);
    g_adxl345_driver.data.x = x_raw;  /* Assuming 1mg/LSB resolution */
    g_adxl345_driver.data.y = y_raw;
    g_adxl345_driver.data.z = z_raw;
    g_adxl345_driver.data.timestamp = get_Tick();
    mutex_unlock(&g_adxl345_driver.config.adxl345_mutex);
    
    return ADXL345_OK;
}

/******************************************************************************
 * @brief   Configure the ADXL345 sensor
 * @details Send configuration commands to the sensor to set measurement 
 *          parameters
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       Successfully configured the sensor
 * @retval  ADXL345_ERROR    Communication failure
 *****************************************************************************/
static adxl345_status_t adxl345_configure_sensor(void)
{
    uint8_t cmd;
    I2C_status_t result;
    
    /* Set data format (±2g, 10-bit mode) */
    cmd = 0x0B;
    result = I2C_hw_write(g_adxl345_driver.config.i2c_address,
                           ADXL345_REG_DATA_FORMAT, &cmd, 1);
    if (result != I2C_OK) {
        return ADXL345_ERROR;
    }
    
    /* Start measurement mode */
    cmd = 0x08;  /* Measurement mode */
    result = I2C_hw_write(g_adxl345_driver.config.i2c_address,
                           ADXL345_REG_POWER_CTL, &cmd, 1);
    if (result != I2C_OK) {
        return ADXL345_ERROR;
    }
    
    return ADXL345_OK;
}

/******************************************************************************
 * @brief   Check device ID
 * @details Read device ID to verify sensor presence
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       Device ID matches
 * @retval  ADXL345_ERROR    Device ID mismatch or communication failure
 *****************************************************************************/
static adxl345_status_t adxl345_check_device_id(void)
{
    uint8_t device_id;
    
    I2C_status_t result = I2C_hw_read(g_adxl345_driver.config.i2c_address,
                                        ADXL345_REG_DEVID, &device_id, 1);
    
    if (result != I2C_OK) {
        return ADXL345_ERROR;
    }
    
    /* ADXL345 device ID should be 0xE5 */
    if (device_id != 0xE5) {
        return ADXL345_ERROR;
    }
    
    return ADXL345_OK;
}

/******************************************************************************
 * @brief   Initialize the ADXL345 sensor
 * @details Configure the I2C interface and initialize the sensor parameters 
 *          with default I2C address
 * @param   None
 * @return  None
 *****************************************************************************/
void adxl345_init(void)
{
    I2C_hw_init();
    g_adxl345_driver.config.i2c_address = ADXL345_I2C_ADDR;
    mutex_init(&g_adxl345_driver.config.adxl345_mutex);
    
    /* Initialize state machine */
    g_adxl345_driver.state = ADXL345_STATE_INIT;
    g_adxl345_driver.state_timestamp = 0;
}

/******************************************************************************
 * @brief   Update the ADXL345 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       State machine processed successfully
 * @retval  ADXL345_ERROR    Error occurred during processing
 *****************************************************************************/
adxl345_status_t adxl345_driver_update(void)
{
    adxl345_status_t status = ADXL345_OK;
    
    switch (g_adxl345_driver.state) {
        case ADXL345_STATE_IDLE:
            /* Idle state, waiting for trigger */
            break;
            
        case ADXL345_STATE_INIT:
            /* Initialize sensor */
            status = adxl345_check_device_id();
            if (status == ADXL345_OK) {
                status = adxl345_configure_sensor();
            }
            
            if (status == ADXL345_OK) {
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            } else {
                /* Initialization failed, return to idle state */
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            }
            break;
            
        case ADXL345_STATE_READ_DATA:
            /* Read acceleration data */
            status = adxl345_read_acceleration();
            if (status == ADXL345_OK) {
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            } else {
                /* Return to idle state even if read fails */
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            }
            break;
            
        case ADXL345_STATE_CONFIGURE:
            /* Configure sensor */
            status = adxl345_configure_sensor();
            if (status == ADXL345_OK) {
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            } else {
                /* Return to idle state if configuration fails */
                g_adxl345_driver.state = ADXL345_STATE_IDLE;
            }
            break;
            
        default:
            /* Abnormal state, reset to IDLE */
            g_adxl345_driver.state = ADXL345_STATE_IDLE;
            break;
    }
    
    return status;
}

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  adxl345_status_t Operation status
 * @retval  ADXL345_OK       Measurement started successfully
 * @retval  ADXL345_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
adxl345_status_t adxl345_driver_trigger(void)
{
    if (g_adxl345_driver.state == ADXL345_STATE_IDLE) {
        g_adxl345_driver.state = ADXL345_STATE_READ_DATA;
        return ADXL345_OK;
    }
    return ADXL345_ERROR; /* Sensor busy, cannot start new measurement */
}

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
adxl345_driver_t adxl345_get_driver(void)
{
    mutex_lock(&g_adxl345_driver.config.adxl345_mutex);
    adxl345_driver_t ret = g_adxl345_driver;
    mutex_unlock(&g_adxl345_driver.config.adxl345_mutex);
        
    return ret;
}
