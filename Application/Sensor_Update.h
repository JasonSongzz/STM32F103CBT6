#ifndef SENSOR_UPDATE_H
#define SENSOR_UPDATE_H

/* Function declarations */

/******************************************************************************
 * @brief   Initialize all sensors
 * @details Initialize all connected sensors in the system
 * @param   None
 * @return  None
 *****************************************************************************/
void sensor_driver_init(void);

/******************************************************************************
 * @brief   Trigger measurements for all sensors
 * @details Start measurement process for all sensors
 * @param   None
 * @return  None
 *****************************************************************************/
void sensor_driver_trigger(void);

/******************************************************************************
 * @brief   Update all sensors state machines
 * @details Process state machines for all sensors
 * @param   None
 * @return  None
 *****************************************************************************/
void sensor_driver_update(void);

/******************************************************************************
 * @brief   Main sensor task function
 * @details Periodically trigger sensors and update their state machines
 * @param   None
 * @return  None
 *****************************************************************************/
void Sensor_Task(void);

#endif /* SENSOR_UPDATE_H */
