/**
 * @file CAN.h
 * @brief CAN hardware interface definition
 * @details This header file defines the CAN hardware interface for 
 *          communication with external devices using Controller Area Network
 *          protocol
 */

#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"                  /* Device header                 */
#include <stdbool.h>

#define BROADCAST_ADDRESS 0xFF              /* Broadcast address             */
#define DEVICE_ADDRESS    0x01              /* Default device address        */

/* CAN mode enumeration */
typedef enum 
{
    CAN_MODE_NORMAL = 0x00,                 /* Normal mode                   */
    CAN_MODE_LOOPBACK = 0x01,               /* Loopback mode                 */
    CAN_MODE_SILENT = 0x02,                 /* Silent mode                   */
    CAN_MODE_SILENT_LOOPBACK = 0x03         /* Silent loopback mode          */
}CAN_Mode;

/* CAN baud rate enumeration */
typedef enum 
{
    CAN_BAUDRATE_5K = 5000,                 /* 5 kbps                        */
    CAN_BAUDRATE_10K  = 10000,              /* 10 kbps                       */
    CAN_BAUDRATE_20K = 20000,               /* 20 kbps                       */
    CAN_BAUDRATE_50K = 50000,               /* 50 kbps                       */
    CAN_BAUDRATE_100K = 100000,             /* 100 kbps                      */
    CAN_BAUDRATE_125K = 125000,             /* 125 kbps                      */
    CAN_BAUDRATE_250K = 250000,             /* 250 kbps                      */
    CAN_BAUDRATE_500K = 500000,             /* 500 kbps                      */
    CAN_BAUDRATE_600K = 600000,             /* 600 kbps                      */
    CAN_BAUDRATE_750K = 750000,             /* 750 kbps                      */
    CAN_BAUDRATE_1000K = 1000000,           /* 1 Mbps                        */
}CAN_BaudRate;

/* Function declarations */

/******************************************************************************
 * @brief   Get current CAN baud rate
 * @details Return the currently configured CAN baud rate
 * @param   None
 * @return  CAN_BaudRate Current baud rate
 *****************************************************************************/
CAN_BaudRate GetCurrentBaudRate(void);

/******************************************************************************
 * @brief   Initialize CAN peripheral
 * @details Configure and initialize CAN controller with specified mode and 
 *          baud rate
 * @param   mode CAN operation mode
 * @param   baud CAN baud rate
 * @return  bool Initialization status
 * @retval  true  Initialization successful
 * @retval  false Initialization failed
 *****************************************************************************/
bool CANInit(CAN_Mode mode, CAN_BaudRate baud);

/******************************************************************************
 * @brief   Update CAN baud rate
 * @details Dynamically change CAN baud rate without reinitializing peripheral
 * @param   newBaud New CAN baud rate
 * @return  None
 *****************************************************************************/
void CAN_Update_BaudRate(CAN_BaudRate newBaud);

/******************************************************************************
 * @brief   Convert baud number to baud rate enumeration
 * @details Convert numeric baud rate identifier to CAN_BaudRate enumeration 
 *          value
 * @param   baudNum Numeric baud rate identifier
 * @return  CAN_BaudRate Corresponding baud rate enumeration value
 *****************************************************************************/
CAN_BaudRate baudNumToBaudRate(uint8_t baudNum);

#endif /* __CAN_H */
