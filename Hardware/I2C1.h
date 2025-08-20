/**
 * @file I2C1.h
 * @brief I2C1 hardware interface definition (software bit-banging on PA5, PA6)
 * @details This header file defines the I2C1 hardware interface using either
 *          software bit-banging or hardware peripheral based on compile-time
 *          configuration
 */

#ifndef __I2C1_H
#define __I2C1_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef HARDWARE_I2C1
/* Software I2C definitions (using PA5-SCL, PA6-SDA) */
#define I2C1_RCC        RCC_APB2Periph_GPIOA  /* I2C1 GPIO clock           */
#define I2C1_PORT       GPIOA                 /* I2C1 GPIO port            */
#define I2C1_SCL_PIN    GPIO_Pin_5            /* I2C1 SCL pin              */
#define I2C1_SDA_PIN    GPIO_Pin_6            /* I2C1 SDA pin              */

/* I2C pin operation macro definitions */
#define I2C1_SCL_HIGH()  GPIO_SetBits(I2C1_PORT, I2C1_SCL_PIN)  /* SCL high  */
#define I2C1_SCL_LOW()   GPIO_ResetBits(I2C1_PORT, I2C1_SCL_PIN)/* SCL low   */
#define I2C1_SDA_HIGH()  GPIO_SetBits(I2C1_PORT, I2C1_SDA_PIN)  /* SDA high  */
#define I2C1_SDA_LOW()   GPIO_ResetBits(I2C1_PORT, I2C1_SDA_PIN)/* SDA low   */
#define I2C1_SDA_READ()  GPIO_ReadInputDataBit(I2C1_PORT, I2C1_SDA_PIN)
                                      /* Read SDA pin state                */

#else
/* Hardware I2C definitions */
#define I2C1_PERIPH     RCC_APB1Periph_I2C1   /* I2C1 peripheral clock     */
#define I2C1_SCL_PORT   GPIOB                 /* I2C1 SCL GPIO port        */
#define I2C1_SDA_PORT   GPIOB                 /* I2C1 SDA GPIO port        */
#define I2C1_SCL_PIN    GPIO_Pin_6            /* I2C1 SCL pin              */
#define I2C1_SDA_PIN    GPIO_Pin_7            /* I2C1 SDA pin              */
#define I2C1_AF         GPIO_Remap_I2C1       /* I2C1 alternate function   */

#endif /* HARDWARE_I2C1 */

/* I2C status enumeration */
typedef enum {
    I2C1_OK = 0,                            /* Operation completed success.  */
    I2C1_ERROR,                             /* General error occurred        */
    I2C1_BUSY,                              /* Device is busy                */
    I2C1_TIMEOUT,                           /* Operation timed out           */
	I2C1_MAX                                /* Maximum status value(used for */
                                            /* bounds checking)              */
} I2C1_status_t;

/* Function declarations */

#define I2C_SPEED_STANDARD  100000  // 100kHz
#define I2C_SPEED_FAST      400000  // 400kHz
#define I2C_SPEED           I2C_SPEED_STANDARD

/******************************************************************************
 * @brief   Initialize I2C1 hardware
 * @details Configure GPIO pins for I2C1 communication
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C1_hw_init(void);

/******************************************************************************
 * @brief   Write data to I2C1 device
 * @details Write data to specified register of I2C1 device
 * @param   addr Device I2C address
 * @param   reg  Register address
 * @param   data Pointer to data buffer
 * @param   len  Length of data to write
 * @return  I2C1_status_t Operation status
 * @retval  I2C1_OK    Write operation successful
 * @retval  I2C1_ERROR Write operation failed
 *****************************************************************************/
I2C1_status_t I2C1_hw_write(uint8_t addr, uint8_t reg, uint8_t *data, 
                            uint16_t len);

/******************************************************************************
 * @brief   Read data from I2C1 device
 * @details Read data from specified register of I2C1 device
 * @param   addr Device I2C address
 * @param   reg  Register address
 * @param   data Pointer to data buffer
 * @param   len  Length of data to read
 * @return  I2C1_status_t Operation status
 * @retval  I2C1_OK    Read operation successful
 * @retval  I2C1_ERROR Read operation failed
 *****************************************************************************/
I2C1_status_t I2C1_hw_read(uint8_t addr, uint8_t reg, uint8_t *data, 
                           uint16_t len);

#ifndef HARDWARE_I2C1
/* Internal function declarations for software I2C */
void I2C1_start(void);
void I2C1_stop(void);
void I2C1_send_ack(void);
void I2C1_send_nack(void);
uint8_t I2C1_wait_ack(void);
void I2C1_send_byte(uint8_t byte);
uint8_t I2C1_read_byte(uint8_t ack);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __I2C1_H */
