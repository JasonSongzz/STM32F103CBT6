/******************************************************************************
 * @file    bsp_tsh54_driver.c
 * @brief   TSH54 tachometer sensor driver interface implementation
 * @details This file implements the driver functions for the TSH54 tachometer
 *          sensor including initialization, RPM measurement, and rotation 
 *          detection
 *****************************************************************************/
#include "bsp_tsh54_driver.h"
#include "ADC.h"
#include "sysTick.h"

/******************************************************************************
 * @brief   Global TSH54 tachometer driver instance
 * @details Contains configuration and data structures for the TSH54 tachometer
 *          sensor
 *****************************************************************************/
static tsh54_driver_t g_tsh54_driver = {0};

/******************************************************************************
 * @brief   Initialize the TSH54 tachometer sensor
 * @details Configure the TSH54 tachometer sensor with default parameters 
 *          including ADC channel, reference voltage, sensitivity, and signal
 *          detection threshold
 * @param   None
 * @return  None
 * @note    This function should be called once at system startup before using
 *          the sensor
 * @note    Default configuration:
 *          - Sensitivity: 1000 RPM per volt (1V = 1000 RPM)
 *          - Detection threshold: 0.1V minimum signal level
 *****************************************************************************/
void tsh54_init(void)
{
    g_tsh54_driver.config.adc_channel = ADC_CHANNEL_TACHO;
    g_tsh54_driver.config.reference_voltage = 3.3f;
    g_tsh54_driver.config.sensitivity = 1000.0f;  /* 1V corresponds to 1000RPM */
    g_tsh54_driver.config.threshold = 1.0f;       /* 1V signal detection thr.  */
    g_tsh54_driver.config.sampling_window = 1000; /* 1 second sampling window  */
    mutex_init(&g_tsh54_driver.config.tsh54_mutex);
    
    /* Initialize state machine */
    g_tsh54_driver.state = TSH54_STATE_IDLE;
    g_tsh54_driver.state_timestamp = 0;
    
    /* Initialize sampling variables */
    g_tsh54_driver.data.pulse_count = 0;
    g_tsh54_driver.signal_detected_during_sampling = false;
    g_tsh54_driver.last_signal_state = false;
    g_tsh54_driver.sampling_start_time = 0;
    g_tsh54_driver.last_adc_read_time = 0;
}

/******************************************************************************
 * @brief   Process ADC sampling and pulse counting
 * @details Sample ADC and detect pulses during sampling window
 * @param   None
 * @return  tsh54_status_t Operation status
 *****************************************************************************/
static tsh54_status_t tsh54_process_sampling(void)
{
    uint32_t current_time = get_Tick();
    
    /* Check if sampling window has ended */
    if ((current_time - g_tsh54_driver.sampling_start_time) >= 
        g_tsh54_driver.config.sampling_window) {
        return TSH54_OK; /* Sampling complete */
    }
    
    /* Sample every 5ms to avoid oversampling */
    if ((current_time - g_tsh54_driver.last_adc_read_time) >= 5) {
        uint16_t adc_value = adc_hw_read(g_tsh54_driver.config.adc_channel);
        float voltage = ((float)adc_value / 4095.0f) * 
                       g_tsh54_driver.config.reference_voltage;
        bool signal_detected = (voltage > g_tsh54_driver.config.threshold);
        
        /* Detect rising edge (pulse) */
        if (g_tsh54_driver.last_signal_state == false && signal_detected == true) {
            g_tsh54_driver.data.pulse_count++;
        }
        
        if (signal_detected) {
            g_tsh54_driver.signal_detected_during_sampling = true;
        }
        
        g_tsh54_driver.last_signal_state = signal_detected;
        g_tsh54_driver.last_adc_read_time = current_time;
    }
    
    return TSH54_ERROR; /* Still sampling */
}

/******************************************************************************
 * @brief   Process and update sensor data
 * @details Calculate RPM and update sensor data structure
 * @param   None
 * @return  tsh54_status_t Operation status
 *****************************************************************************/
static tsh54_status_t tsh54_process_data(void)
{
    uint32_t current_time = get_Tick();
    
    /* Lock to protect data update */
    mutex_lock(&g_tsh54_driver.config.tsh54_mutex);
    
    /* Read final ADC value */
    g_tsh54_driver.data.adc_value = adc_hw_read(g_tsh54_driver.config.adc_channel);
    g_tsh54_driver.data.voltage = ((float)g_tsh54_driver.data.adc_value / 4095.0f) * 
                                  g_tsh54_driver.config.reference_voltage;
    g_tsh54_driver.data.signal_detected = g_tsh54_driver.signal_detected_during_sampling;
    g_tsh54_driver.data.timestamp = current_time;
    
    /* Calculate RPM (pulse count/sampling time * 60 seconds) */
    float sampling_time_sec = (float)g_tsh54_driver.config.sampling_window / 1000.0f;
    if (sampling_time_sec > 0) {
        g_tsh54_driver.data.rpm = (float)g_tsh54_driver.data.pulse_count / 
                                  sampling_time_sec * 60.0f;
    } else {
        g_tsh54_driver.data.rpm = 0.0f;
    }
    
    mutex_unlock(&g_tsh54_driver.config.tsh54_mutex);
    
    return TSH54_OK;
}

/******************************************************************************
 * @brief   Update the TSH54 sensor state machine
 * @details Non-blocking state machine to handle sensor operations
 * @param   None
 * @return  tsh54_status_t Operation status
 * @retval  TSH54_OK       State machine processed successfully
 * @retval  TSH54_ERROR    Error occurred during processing
 *****************************************************************************/
tsh54_status_t tsh54_driver_update(void)
{
    tsh54_status_t status = TSH54_OK;
    
    switch (g_tsh54_driver.state) {
        case TSH54_STATE_IDLE:
            /* Idle state, waiting for trigger */
            break;
            
        case TSH54_STATE_INIT:
            /* Initialization state (if additional initialization needed) */
            g_tsh54_driver.state = TSH54_STATE_IDLE;
            break;
            
        case TSH54_STATE_SAMPLING:
            /* Sampling state */
            status = tsh54_process_sampling();
            if (status == TSH54_OK) {
                /* Sampling complete, enter data processing state */
                g_tsh54_driver.state = TSH54_STATE_PROCESS_DATA;
            }
            break;
            
        case TSH54_STATE_PROCESS_DATA:
            /* Data processing state */
            status = tsh54_process_data();
            if (status == TSH54_OK) {
                /* Data processing complete, return to idle state */
                g_tsh54_driver.state = TSH54_STATE_IDLE;
            }
            break;
            
        default:
            /* Abnormal state, reset to IDLE */
            g_tsh54_driver.state = TSH54_STATE_IDLE;
            break;
    }
    
    return status;
}

/******************************************************************************
 * @brief   Trigger a new measurement cycle
 * @details Start a new measurement if sensor is in idle state
 * @param   None
 * @return  tsh54_status_t Operation status
 * @retval  TSH54_OK       Measurement started successfully
 * @retval  TSH54_ERROR    Sensor is busy, cannot start new measurement
 *****************************************************************************/
tsh54_status_t tsh54_driver_trigger(void)
{
    if (g_tsh54_driver.state == TSH54_STATE_IDLE) {
        /* Initialize sampling parameters */
        g_tsh54_driver.sampling_start_time = get_Tick();
        g_tsh54_driver.last_adc_read_time = get_Tick();
        g_tsh54_driver.data.pulse_count = 0;
        g_tsh54_driver.signal_detected_during_sampling = false;
        g_tsh54_driver.last_signal_state = false;
        
        /* Enter sampling state */
        g_tsh54_driver.state = TSH54_STATE_SAMPLING;
        return TSH54_OK;
    }
    return TSH54_ERROR; /* Sensor busy, cannot start new measurement */
}

/******************************************************************************
 * @brief   Get a copy of the current sensor data
 * @details Retrieve a thread-safe copy of the latest sensor measurements
 * @param   data Pointer to store sensor data
 * @return  None
 *****************************************************************************/
tsh54_driver_t tsh54_get_driver(void)
{
    mutex_lock(&g_tsh54_driver.config.tsh54_mutex);
    tsh54_driver_t ret = g_tsh54_driver;
    mutex_unlock(&g_tsh54_driver.config.tsh54_mutex);
    
    return ret;
}
