/******************************************************************************
 * @file    bsp_pt100_driver.c
 * @brief   PT100 temperature sensor driver interface implementation
 * @details This file implements the driver functions for the PT100 resistance 
 *          temperature detector (RTD) sensor including initialization, data 
 *          acquisition, and temperature calculation functions using both 
 *          linear approximation and lookup table methods
 *****************************************************************************/
#include "bsp_pt100_driver.h"
#include "pt100_lookup_table.h"
#include "ADC.h"
#include "sysTick.h"
#include <math.h>

/******************************************************************************
 * @brief   Global PT100 driver instance
 * @details Contains configuration and data structures for the PT100 sensor
 *****************************************************************************/
static pt100_driver_t g_pt100_driver = {0};

/******************************************************************************
 * @brief   Size of the PT100 lookup table
 *****************************************************************************/
#define LOOKUP_TABLE_SIZE (sizeof(pt100_lookup_table) / \
                           sizeof(pt100_lookup_table[0]))

/******************************************************************************
 * @brief   Calculate temperature from PT100 resistance using linear approx.
 * @details Convert PT100 resistance value to temperature using linear 
 *          approximation formula: T = (R - R0) / (ALPHA * R0)
 * @param   resistance PT100 sensor resistance value in ohms
 * @return  float Temperature value in degrees Celsius
 * @note    This function uses linear approximation. For higher accuracy, 
 *          consider using the lookup table method or Callendar-Van Dusen eq.
 * @see     pt100_calculate_temperature_lookup
 *****************************************************************************/
static float pt100_calculate_temperature_linear(float resistance)
{
    const float R0 = 100.0f;      
    const float ALPHA = 0.00385f; 

    if (resistance > 0) {
        return (resistance - R0) / (ALPHA * R0);
    }
    
    return -273.15f; 
}

/******************************************************************************
 * @brief   Calculate temperature from PT100 resistance using table with binary
 *          search
 * @details Convert PT100 resistance value to temperature by finding the 
 *          closest match in the pre-calculated lookup table using binary 
 *          search for improved performance
 * @param   resistance PT100 sensor resistance value in ohms
 * @return  float Temperature value in degrees Celsius
 * @note    The function uses linear interpolation between table entries for
 *          better accuracy and binary search for improved performance
 * @note    Temperature range: -100°C to 250°C based on the lookup table
 * @see     pt100_lookup_table
 * @see     pt100_calculate_temperature_linear
 *****************************************************************************/
static float pt100_calculate_temperature_lookup(float resistance)
{
    /* Boundary check - below minimum temperature */
    if (resistance <= pt100_lookup_table[0].resist) {
        return pt100_lookup_table[0].temp;
    }
    
    /* Boundary check - above maximum temperature */
    if (resistance >= pt100_lookup_table[LOOKUP_TABLE_SIZE - 1].resist) {
        return pt100_lookup_table[LOOKUP_TABLE_SIZE - 1].temp;
    }
    
    /* Use binary search to quickly locate the interval containing resistance */
    uint16_t low = 0;
    uint16_t high = LOOKUP_TABLE_SIZE - 1;
    uint16_t mid;
    
    /* Binary search to find the interval containing the target resistance */
    /* Continue searching while high - low > 1 */
    while ((high - low) > 1) {
        mid = (low + high) >> 1; /* Equivalent to (low + high) / 2 */
        
        if (resistance > pt100_lookup_table[mid].resist) {
            low = mid;  /* Target is in the right half */
        } else {
            high = mid; /* Target is in the left half or exactly at mid */
        }
    }
    
    /* Ensure interval is valid and perform linear interpolation */
    if (pt100_lookup_table[high].resist != pt100_lookup_table[low].resist) {
        /* Calculate interpolation ratio */
        float ratio = (resistance - pt100_lookup_table[low].resist) / 
                     (pt100_lookup_table[high].resist - 
                      pt100_lookup_table[low].resist);
        
        /* Linear interpolation to calculate temperature value */
        return pt100_lookup_table[low].temp + 
               ratio * (pt100_lookup_table[high].temp - 
                        pt100_lookup_table[low].temp);
    } else {
        /* Prevent division by zero, return the lower temperature value */
        return pt100_lookup_table[low].temp;
    }
}

/******************************************************************************
 * @brief   Calculate temperature from PT100 resistance value
 * @details Convert PT100 resistance value to temperature using the currently
 *          selected calculation method (linear approximation or lookup table)
 * @param   resistance PT100 sensor resistance value in ohms
 * @return  float Temperature value in degrees Celsius
 * @note    The calculation method can be changed using 
 *          pt100_set_calculation_method()
 * @see     pt100_set_calculation_method
 * @see     pt100_calculate_temperature_linear
 * @see     pt100_calculate_temperature_lookup
 *****************************************************************************/
static float pt100_calculate_temperature(float resistance)
{
    switch (g_pt100_driver.config.calc_method) {
        case PT100_CALC_LOOKUP_TABLE:
            return pt100_calculate_temperature_lookup(resistance);
            
        case PT100_CALC_LINEAR:
        default:
            return pt100_calculate_temperature_linear(resistance);
    }
}

/******************************************************************************
 * @brief   Initialize the PT100 sensor
 * @details Configure the PT100 sensor with default parameters including ADC
 *          channel, reference voltage, and series resistor values
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 * @see     pt100_config_t
 *****************************************************************************/
void pt100_init(void)
{
    g_pt100_driver.config.adc_channel = ADC_CHANNEL_PT100;
    g_pt100_driver.config.reference_voltage = 3.3f;  
    g_pt100_driver.config.series_resistor = 1000.0f; 
    g_pt100_driver.config.calc_method = PT100_CALC_LOOKUP_TABLE;
    mutex_init(&g_pt100_driver.config.pt100_mutex);
    
    /* Initialize state machine */
    g_pt100_driver.state = PT100_STATE_IDLE;
    g_pt100_driver.state_timestamp = 0;
}

/******************************************************************************
 * @brief   Process ADC reading
 * @details Read ADC value from PT100 sensor
 * @param   None
 * @return  pt100_status_t Operation status
 *****************************************************************************/
static pt100_status_t pt100_process_adc_read(void)
{
    g_pt100_driver.data.adc_value = adc_hw_read(
        g_pt100_driver.config.adc_channel);
    
    return PT100_OK;
}

/******************************************************************************
 * @brief   Calculate resistance and temperature
 * @details Calculate resistance from ADC value and convert to temperature
 * @param   None
 * @return  pt100_status_t Operation status
 *****************************************************************************/
static pt100_status_t pt100_process_calculation(void)
{
    /* R_pt100 = R_series * (ADC_value / (4095 - ADC_value)) */
    if (g_pt100_driver.data.adc_value < 4095) {
        g_pt100_driver.data.resistance = g_pt100_driver.config.series_resistor * 
            ((float)g_pt100_driver.data.adc_value / 
                (4095.0f - (float)g_pt100_driver.data.adc_value));
    } else {
        g_pt100_driver.data.resistance = 0.0f;
    }
    
    g_pt100_driver.data.temperature = 
        pt100_calculate_temperature(g_pt100_driver.data.resistance);
    
    g_pt100_driver.data.timestamp = get_Tick();
    
    return PT100_OK;
}

/******************************************************************************
 * @brief   Update the PT100 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK       State machine processed successfully
 * @retval  PT100_ERROR    Error occurred during processing
 *****************************************************************************/
pt100_status_t pt100_driver_update(void)
{
    pt100_status_t status = PT100_OK;
    
    switch (g_pt100_driver.state) {
        case PT100_STATE_IDLE:
            /* Idle state, waiting for trigger */
            break;
            
        case PT100_STATE_INIT:
            /* Initialization state (if additional initialization needed) */
            g_pt100_driver.state = PT100_STATE_IDLE;
            break;
            
        case PT100_STATE_READ_ADC:
            /* ADC reading state */
            status = pt100_process_adc_read();
            if (status == PT100_OK) {
                /* ADC reading complete, enter calculation state */
                g_pt100_driver.state = PT100_STATE_CALCULATE;
            } else {
                /* Reading failed, return to idle state */
                g_pt100_driver.state = PT100_STATE_IDLE;
            }
            break;
            
        case PT100_STATE_CALCULATE:
            /* Calculation state */
            status = pt100_process_calculation();
            if (status == PT100_OK) {
                /* Calculation complete, return to idle state */
                g_pt100_driver.state = PT100_STATE_IDLE;
            } else {
                /* Calculation failed, return to idle state */
                g_pt100_driver.state = PT100_STATE_IDLE;
            }
            break;
            
        default:
            /* Abnormal state, reset to IDLE */
            g_pt100_driver.state = PT100_STATE_IDLE;
            break;
    }
    
    return status;
}

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK       Measurement started successfully
 * @retval  PT100_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
pt100_status_t pt100_driver_trigger(void)
{
    if (g_pt100_driver.state == PT100_STATE_IDLE) {
        /* Enter ADC reading state */
        g_pt100_driver.state = PT100_STATE_READ_ADC;
        return PT100_OK;
    }
    return PT100_ERROR; /* Sensor busy, cannot start new measurement */
}

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
pt100_driver_t pt100_get_driver(void)
{
    mutex_lock(&g_pt100_driver.config.pt100_mutex);
    pt100_driver_t ret = g_pt100_driver;
    mutex_unlock(&g_pt100_driver.config.pt100_mutex);
    
    return ret;
}

/******************************************************************************
 * @brief   Set PT100 calculation method
 * @details Configure which method to use for temperature calculation
 * @param   method Calculation method to use
 * @return  pt100_status_t Operation status
 * @retval  PT100_OK    Successfully set calculation method
 * @retval  PT100_ERROR Invalid method parameter
 *****************************************************************************/
pt100_status_t pt100_set_calc_method(pt100_calc_method_t method)
{
    if (method >= PT100_CALC_MAX) {
        return PT100_ERROR;
    }
    
    g_pt100_driver.config.calc_method = method;
    return PT100_OK;
}
