/******************************************************************************
 * @file    bsp_water_driver.c
 * @brief   Water detection sensor driver interface implementation
 * @details This file implements the driver functions for water detection 
 *          sensor including initialization, data acquisition, and status 
 *          determination
 *****************************************************************************/
#include "bsp_water_driver.h"
#include "ADC.h"
#include "sysTick.h"

/******************************************************************************
 * @brief   Global water detection driver instance
 * @details Contains configuration and data structures for the water detection
 *          sensor
 *****************************************************************************/
static water_driver_t g_water_driver = {0};

/******************************************************************************
 * @brief   Determine water immersion status based on voltage
 * @details Determine water immersion status based on predefined threshold values
 * @param   voltage Voltage value from sensor
 * @return  water_immersion_status_t Water immersion status
 * @note    Threshold values:
 *          - < 0.5V: Dry state
 *          - 0.5V-1.5V: Wet state
 *          - 1.5V-2.5V: Submerged state
 *          - > 2.5V: Error state
 *****************************************************************************/
water_immersion_status_t water_get_immersion_status(float voltage)
{
    if (voltage < g_water_driver.config.threshold_dry) {
        return WATER_STATUS_DRY;
    } else if (voltage < g_water_driver.config.threshold_wet) {
        return WATER_STATUS_WET;
    } else if (voltage < g_water_driver.config.threshold_submerged) {
        return WATER_STATUS_SUBMERGED;
    } else {
        return WATER_STATUS_ERROR;
    }
}

/******************************************************************************
 * @brief   Initialize the water detection sensor
 * @details Configure the water detection sensor with default parameters 
 *          including ADC channel, reference voltage, and threshold values for
 *          different states
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 * @note    Threshold values:
 *          - < 0.5V: Dry state
 *          - 0.5V-1.5V: Wet state
 *          - 1.5V-2.5V: Submerged state
 *          - > 2.5V: Error state
 * @see     water_config_t
 *****************************************************************************/
void water_init(void)
{
    g_water_driver.config.adc_channel = ADC_CHANNEL_WATER;
    g_water_driver.config.reference_voltage = 3.3f;
    g_water_driver.config.series_resistor = 100000.0f; 
    g_water_driver.config.threshold_dry = 0.5f;       /* < 0.5V is dry */
    g_water_driver.config.threshold_wet = 1.5f;       /* 0.5V-1.5V is wet */
    g_water_driver.config.threshold_submerged = 2.5f; /* 1.5V-2.5V is submerged */
    mutex_init(&g_water_driver.config.water_mutex);
    
    /* Initialize state machine */
    g_water_driver.state = WATER_STATE_IDLE;
    g_water_driver.state_timestamp = 0;
}

/******************************************************************************
 * @brief   Process ADC reading
 * @details Read ADC value from water detection sensor
 * @param   None
 * @return  water_status_t Operation status
 *****************************************************************************/
static water_status_t water_process_adc_read(void)
{
    g_water_driver.data.adc_value = adc_hw_read(
        g_water_driver.config.adc_channel);
    
    return WATER_OK;
}

/******************************************************************************
 * @brief   Calculate resistance and voltage
 * @details Calculate resistance and voltage from ADC value
 * @param   None
 * @return  water_status_t Operation status
 *****************************************************************************/
static water_status_t water_process_calculation(void)
{
    /* R_water = R_series * (ADC_value / (4095 - ADC_value)) */
    if (g_water_driver.data.adc_value < 4095) {
        g_water_driver.data.resistance = g_water_driver.config.series_resistor *
            ((float)g_water_driver.data.adc_value / 
                (4095.0f - (float)g_water_driver.data.adc_value));
        
        /* Calculate voltage */
        g_water_driver.data.voltage = ((float)g_water_driver.data.adc_value / 4095.0f) *
                                     g_water_driver.config.reference_voltage;
        
        return WATER_OK;
    } else {
        g_water_driver.data.resistance = 0.0f;
        g_water_driver.data.voltage = 0.0f;
        return WATER_ERROR;
    }
}

/******************************************************************************
 * @brief   Determine water status
 * @details Determine water immersion status based on voltage thresholds
 * @param   None
 * @return  water_status_t Operation status
 *****************************************************************************/
static water_status_t water_process_determination(void)
{
    water_immersion_status_t immersion_status = water_get_immersion_status(
        g_water_driver.data.voltage);
    
    /* Convert to driver status */
    switch (immersion_status) {
        case WATER_STATUS_DRY:
        case WATER_STATUS_WET:
        case WATER_STATUS_SUBMERGED:
            g_water_driver.data.status = WATER_OK;
            break;
        case WATER_STATUS_ERROR:
        default:
            g_water_driver.data.status = WATER_ERROR;
            break;
    }
    
    g_water_driver.data.timestamp = get_Tick();
    
    return WATER_OK;
}

/******************************************************************************
 * @brief   Update the water sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  water_status_t Operation status
 * @retval  WATER_OK       State machine processed successfully
 * @retval  WATER_ERROR    Error occurred during processing
 *****************************************************************************/
water_status_t water_driver_update(void)
{
    water_status_t status = WATER_OK;
    
    switch (g_water_driver.state) {
        case WATER_STATE_IDLE:
            /* Idle state, waiting for trigger */
            break;
            
        case WATER_STATE_INIT:
            /* Initialization state (if additional initialization needed) */
            g_water_driver.state = WATER_STATE_IDLE;
            break;
            
        case WATER_STATE_READ_ADC:
            /* ADC reading state */
            status = water_process_adc_read();
            if (status == WATER_OK) {
                /* ADC reading complete, enter calculation state */
                g_water_driver.state = WATER_STATE_CALCULATE;
            } else {
                /* Reading failed, return to idle state */
                g_water_driver.state = WATER_STATE_IDLE;
            }
            break;
            
        case WATER_STATE_CALCULATE:
            /* Calculation state */
            status = water_process_calculation();
            if (status == WATER_OK) {
                /* Calculation complete, enter determination state */
                g_water_driver.state = WATER_STATE_DETERMINE;
            } else {
                /* Calculation failed, return to idle state */
                g_water_driver.state = WATER_STATE_IDLE;
            }
            break;
            
        case WATER_STATE_DETERMINE:
            /* Status determination state */
            status = water_process_determination();
            if (status == WATER_OK) {
                /* Determination complete, return to idle state */
                g_water_driver.state = WATER_STATE_IDLE;
            } else {
                /* Determination failed, return to idle state */
                g_water_driver.state = WATER_STATE_IDLE;
            }
            break;
            
        default:
            /* Abnormal state, reset to IDLE */
            g_water_driver.state = WATER_STATE_IDLE;
            break;
    }
    
    return status;
}

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  water_status_t Operation status
 * @retval  WATER_OK       Measurement started successfully
 * @retval  WATER_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
water_status_t water_driver_trigger(void)
{
    if (g_water_driver.state == WATER_STATE_IDLE) {
        /* Enter ADC reading state */
        g_water_driver.state = WATER_STATE_READ_ADC;
        return WATER_OK;
    }
    return WATER_ERROR; /* Sensor busy, cannot start new measurement */
}

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
water_driver_t water_get_driver(void)
{
    mutex_lock(&g_water_driver.config.water_mutex);
    water_driver_t ret = g_water_driver;
    mutex_unlock(&g_water_driver.config.water_mutex);
    
    return ret;
}
