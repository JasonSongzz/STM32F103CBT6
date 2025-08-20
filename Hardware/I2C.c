/******************************************************************************
 * @file    I2C.c
 * @brief   I2C hardware interface implementation (software bit-banging)
 * @details This file implements the I2C hardware interface using either 
 *          software bit-banging or hardware peripheral based on compile-time
 *          configuration
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "I2C.h"
#include "sysTick.h"

#ifndef HARDWARE_I2C

/******************************************************************************
 * @brief   Initialize I2C hardware
 * @details Configure GPIO pins for I2C communication
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_hw_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;    

    RCC_APB2PeriphClockCmd(I2C2_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C2_PORT, &GPIO_InitStructure);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
    I2C_FAST_DELAY;
}

/******************************************************************************
 * @brief   Generate I2C start condition
 * @details Generate start condition on I2C bus
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_start(void)
{
    I2C_SDA_HIGH();         /* SDA HIGH */
    I2C_SCL_HIGH();         /* SCL HIGH */
    I2C_FAST_DELAY;
    I2C_SDA_LOW();          /* SDA LOW  */
    I2C_FAST_DELAY;
    I2C_SCL_LOW();          /* SCL LOW  */
    I2C_FAST_DELAY;
}

/******************************************************************************
 * @brief   Generate I2C stop condition
 * @details Generate stop condition on I2C bus
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_stop(void)
{
    I2C_SDA_LOW();          /* SDA LOW  */
    I2C_FAST_DELAY;
    I2C_SCL_HIGH();         /* SCL HIGH */
    I2C_FAST_DELAY;
    I2C_SDA_HIGH();         /* SDA HIGH */
    I2C_FAST_DELAY;
}

/******************************************************************************
 * @brief   Send ACK signal
 * @details Send acknowledge signal on I2C bus
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_send_ack(void)
{
    I2C_SDA_LOW();          /* SDA LOW  */
    I2C_FAST_DELAY;
    I2C_SCL_HIGH();         /* SCL HIGH */
    I2C_FAST_DELAY;
    I2C_SCL_LOW();          /* SDA LOW  */
    I2C_FAST_DELAY;
}

/******************************************************************************
 * @brief   Send NACK signal
 * @details Send not-acknowledge signal on I2C bus
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_send_nack(void)
{
    I2C_SDA_HIGH();         /* SDA HIGH */
    I2C_FAST_DELAY;
    I2C_SCL_HIGH();         /* SCL HIGH */
    I2C_FAST_DELAY;
    I2C_SCL_LOW();          /* SCL LOW  */
    I2C_FAST_DELAY;
}

/******************************************************************************
 * @brief   Wait for ACK signal
 * @details Wait for acknowledge signal from I2C slave device
 * @param   None
 * @return  uint8_t Data byte read
 *****************************************************************************/
uint8_t I2C_wait_ack(void)
{
    uint8_t ack = 0;
   
    I2C_SDA_HIGH();
    I2C_FAST_DELAY
    
    I2C_SCL_HIGH();
    I2C_FAST_DELAY
    
    if(I2C_SDA_READ() == Bit_SET)
	{
		ack = 1;  
    }else{
        ack = 0;  
    }
    I2C_SCL_LOW();
    I2C_FAST_DELAY
    
    return ack;        
}

/******************************************************************************
 * @brief   Send byte on I2C bus
 * @details Send one byte of data on I2C bus
 * @param   byte Data byte to send
 * @return  None
 *****************************************************************************/
void I2C_send_byte(uint8_t byte)
{
    for(uint8_t i = 0; i < 8; i++) 
	{
        I2C_SCL_LOW();
        if(byte & 0x80) 
		{
            I2C_SDA_HIGH();
        }else{
            I2C_SDA_LOW();
        }
        I2C_FAST_DELAY;
		
        I2C_SCL_HIGH();
        I2C_FAST_DELAY;

        byte <<= 1;
    }
    I2C_SCL_LOW();
}

/******************************************************************************
 * @brief   Read byte from I2C bus
 * @details Read one byte of data from I2C bus
 * @param   ack ACK/NACK to send after reading
 * @return  uint8_t Data byte read
 *****************************************************************************/
uint8_t I2C_read_byte(uint8_t ack)
{
    uint8_t byte = 0;
    I2C_SDA_HIGH();
    for(uint8_t i = 0; i < 8; i++) 
	{
        I2C_SCL_LOW();
        I2C_FAST_DELAY;
        
        I2C_SCL_HIGH();
		I2C_FAST_DELAY;      
        
        if(I2C_SDA_READ() == Bit_SET) 
		{
            byte |= (0x80 >> i);
        }
        I2C_FAST_DELAY;
    }
    I2C_SCL_LOW();
    if(ack) 
	{
        I2C_send_ack();
    }else{
        I2C_send_nack();
    }
    return byte;
}

#else

/******************************************************************************
 * @brief   Initialize I2C hardware
 * @details Configure hardware I2C peripheral
 * @param   None
 * @return  None
 *****************************************************************************/
void I2C_hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* Enable clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(I2C2_PERIPH, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    
    /* Apply I2C configuration */
    I2C_Init(I2C2, &I2C_InitStructure);
    
    /* Enable I2C */
    I2C_Cmd(I2C2, ENABLE);
}

/******************************************************************************
 * @brief   Write data to I2C device using hardware I2C
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
                          uint16_t len)
{
    // Wait until I2C is not busy
    uint32_t timeout = 0xFFFF;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && timeout--)
        ;
    
    // Generate start condition
    I2C_GenerateSTART(I2C2, ENABLE);
    
    // Test on EV5 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
        ;
    
    // Send device address for write
    I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    
    // Test on EV6 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && 
          timeout--)
        ;
    
    // Send register address
    I2C_SendData(I2C2, reg);
    
    // Test on EV8 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--)
        ;
    
    // Send data
    for(uint16_t i = 0; i < len; i++)
    {
        I2C_SendData(I2C2, data[i]);
        
        // Test on EV8 and clear it
        timeout = 0xFFFF;
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && 
              timeout--)
            ;
    }
    
    // Send STOP condition
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    return I2C_OK;
}

/******************************************************************************
 * @brief   Read data from I2C device using hardware I2C
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
                         uint16_t len)
{
    // Wait until I2C is not busy
    uint32_t timeout = 0xFFFF;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && timeout--)
        ;
    
    // Send START condition
    I2C_GenerateSTART(I2C2, ENABLE);
    
    // Test on EV5 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
        ;
    
    // Send device address for write
    I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    
    // Test on EV6 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && 
          timeout--)
        ;
    
    // Send register address
    I2C_SendData(I2C2, reg);
    
    // Test on EV8 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--)
        ;
    
    // Send START condition again (repeated start)
    I2C_GenerateSTART(I2C2, ENABLE);
    
    // Test on EV5 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout--)
        ;
    
    // Send device address for read
    I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Receiver);
    
    // Test on EV6 and clear it
    timeout = 0xFFFF;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && 
          timeout--)
        ;
    
    // Read data
    for(uint16_t i = 0; i < len; i++)
    {
        if(i == (len - 1))
        {
            // Disable ACK for last byte
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            // Send STOP condition
            I2C_GenerateSTOP(I2C2, ENABLE);
        }
        
        // Test on EV7 and clear it
        timeout = 0xFFFF;
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) && 
              timeout--)
            ;
        
        // Read a byte
        data[i] = I2C_ReceiveData(I2C2);
    }
    
    // Re-enable ACK
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    
    return I2C_OK;
}

#endif /* HARDWARE_I2C */

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
                          uint16_t len)
{
#ifndef HARDWARE_I2C
    I2C_start();

    I2C_send_byte(addr << 1);
    if(I2C_wait_ack())
    {
        I2C_stop();
        return I2C_ERROR;
    }
    I2C_send_byte(reg);
    if(I2C1_wait_ack())
    {
        I2C_stop();
        return I2C_ERROR;
    }
    for(uint16_t i = 0; i < len; i++)
    {
        I2C_send_byte(data[i]);
        if(I2C1_wait_ack())
        {
            I2C_stop();
            return I2C_ERROR;
        }
    }
    I2C_stop();
    
    return I2C_OK;
#else
    return I2C1_hw_write(addr, reg, data, len);
#endif
}

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
                         uint16_t len)
{
#ifndef HARDWARE_I2C
	
    I2C_start();

    I2C_send_byte(addr << 1);
    if(I2C_wait_ack())
    {
        I2C_stop();
        return I2C_ERROR;
    }

    I2C_send_byte(reg);
    if(I2C_wait_ack())
    {
        I2C_stop();
        return I2C_ERROR;
    }
    I2C_start();

    I2C_send_byte((addr << 1) | 0x01);
    if(I2C_wait_ack())
    {
        I2C_stop();
        return I2C_ERROR;
    }

    for (uint16_t i = 0; i < len; i++)
            data[i] = I2C_read_byte(i == len - 1 ? 0 : 1);

    I2C_stop();
    
    return I2C_OK;
#else
    return I2C_hw_read(addr, reg, data, len);
#endif
}
