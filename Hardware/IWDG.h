/**
 * @file IWDG.h
 * @brief IWDG hardware interface definition
 * @details This header file defines the Independent Watchdog (IWDG) hardware
 *          interface for system monitoring and reset prevention
 */

#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f10x.h"                  /* Device header                 */
#include <stdbool.h>

/* IWDG timeout enumeration */
typedef enum {
    IWDG_TIMEOUT_1S = 0,                /* 1 second timeout              */
    IWDG_TIMEOUT_2S,                    /* 2 seconds timeout             */
    IWDG_TIMEOUT_4S,                    /* 4 seconds timeout             */
    IWDG_TIMEOUT_8S,                    /* 8 seconds timeout             */
    IWDG_TIMEOUT_16S                    /* 16 seconds timeout            */
} IWDG_Timeout;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize IWDG
 * @details Configure and enable Independent Watchdog
 * @param   timeout IWDG timeout period
 * @return  bool Initialization status
 * @retval  true  Initialization successful
 * @retval  false Initialization failed
 *****************************************************************************/
bool IWDGInit(IWDG_Timeout timeout);

/******************************************************************************
 * @brief   Feed IWDG
 * @details Reload IWDG counter to prevent reset
 * @param   None
 * @return  None
 *****************************************************************************/
void IWDG_Feed(void);

/******************************************************************************
 * @brief   Check IWDG reset cause
 * @details Check if system was reset by IWDG
 * @param   None
 * @return  uint8_t Reset cause status
 * @retval  1 System was reset by IWDG
 * @retval  0 System was not reset by IWDG
 *****************************************************************************/
uint8_t IWDG_CheckResetCause(void);

#endif /* __IWDG_H */
