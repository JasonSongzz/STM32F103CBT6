/**
 * @file I2C.h
 * @brief I2C hardware interface definition (software bit-banging)
 * @details This header file defines the I2C hardware interface using either
 *          software bit-banging or hardware peripheral based on compile-time
 *          configuration
 */

#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef HARDWARE_I2C
/* Software I2C definitions (using PB10-SCL, PB11-SDA) */
#define I2C2_RCC        RCC_APB2Periph_GPIOB  /* I2C2 GPIO clock           */
#define I2C2_PORT       GPIOB                 /* I2C2 GPIO port            */
#define I2C2_SCL_PIN    GPIO_Pin_10           /* I2C2 SCL pin              */
#define I2C2_SDA_PIN    GPIO_Pin_11           /* I2C2 SDA pin              */

/* I2C pin operation macro definitions */
#define I2C_SCL_HIGH()  GPIO_SetBits(I2C2_PORT, I2C2_SCL_PIN)   /* SCL high  */
#define I2C_SCL_LOW()   GPIO_ResetBits(I2C2_PORT, I2C2_SCL_PIN) /* SCL low   */
#define I2C_SDA_HIGH()  GPIO_SetBits(I2C2_PORT, I2C2_SDA_PIN)   /* SDA high  */
#define I2C_SDA_LOW()   GPIO_ResetBits(I2C2_PORT, I2C2_SDA_PIN) /* SDA low   */
#define I2C_SDA_READ()  GPIO_ReadInputDataBit(I2C2_PORT, I2C2_SDA_PIN)
                                      /* Read SDA pin state                */

#else
/* Hardware I2C definitions */
#define I2C2_PERIPH     RCC_APB1Periph_I2C2   /* I2C2 peripheral clock     */
#define I2C2_SCL_PORT   GPIOB                 /* I2C2 SCL GPIO port        */
#define I2C2_SDA_PORT   GPIOB                 /* I2C2 SDA GPIO port        */
#define I2C2_SCL_PIN    GPIO_Pin_10           /* I2C2 SCL pin              */
#define I2C2_SDA_PIN    GPIO_Pin_11           /* I2C2 SDA pin              */

#endif /* HARDWARE_I2C */

/* I2C status enumeration */
typedef enum {
    I2C_OK = 0,                             /* Operation completed success.  */
    I2C_ERROR,                              /* General error occurred        */
    I2C_BUSY,                               /* Device is busy                */
    I2C_TIMEOUT,                            /* Operation timed out           */
	I2C_MAX                                 /* Maximum status value(used for */
                                            /* bounds checking)              */
} I2C_status_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize I2C hardware
 * @details Configure GPIO pins for I2C communication
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_hw_init(void);

/******************************************************************************
 * @brief   Write data to I2C device
 * @details Write data to specified register of I2C device
 * @param   addr Device I2C address
 * @param   reg  Register address
 * @param   data Pointer to data buffer
 * @param   len  Length of data to write
 * @return  I2C_status_t Operation status
 * @retval  I2C_OK    Write operation successful
 * @retval  I2C_ERROR Write operation failed
 *****************************************************************************/
I2C_status_t I2C_hw_write(uint8_t addr, uint8_t reg, uint8_t *data, 
                          uint16_t len);

/******************************************************************************
 * @brief   Read data from I2C device
 * @details Read data from specified register of I2C device
 * @param   addr Device I2C address
 * @param   reg  Register address
 * @param   data Pointer to data buffer
 * @param   len  Length of data to read
 * @return  I2C_status_t Operation status
 * @retval  I2C_OK    Read operation successful
 * @retval  I2C_ERROR Read operation failed
 *****************************************************************************/
I2C_status_t I2C_hw_read(uint8_t addr, uint8_t reg, uint8_t *data, 
                         uint16_t len);

#ifndef HARDWARE_I2C
/* Internal function declarations (for I2C.c internal use only) */
void I2C_start(void);
void I2C_stop(void);
void I2C_send_ack(void);
void I2C_send_nack(void);
uint8_t I2C1_wait_ack(void);
void I2C_send_byte(uint8_t byte);
uint8_t I2C_read_byte(uint8_t ack);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
