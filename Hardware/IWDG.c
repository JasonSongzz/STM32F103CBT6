/******************************************************************************
 * @file    IWDG.c
 * @brief   IWDG hardware interface implementation
 * @details This file implements the Independent Watchdog (IWDG) hardware 
 *          interface for system monitoring and reset prevention
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "stm32f10x_iwdg.h"
#include "IWDG.h"

/******************************************************************************
 * @brief   Initialize IWDG
 * @details Configure and enable Independent Watchdog
 * @param   timeout IWDG timeout period
 * @return  bool Initialization status
 * @retval  true  Initialization successful
 * @retval  false Initialization failed
 *****************************************************************************/
bool IWDGInit(IWDG_Timeout timeout) 
{
    /* Enable LSI (40kHz) */
    RCC_LSICmd(ENABLE);
    /* Wait for LSI ready */
    uint32_t timeout_counter = 0;
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
        timeout_counter++;
        if(timeout_counter > 0x5000)
        {  
            break;
        }
    }
    
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    uint16_t prescaler;
    uint16_t reload;
    
    switch(timeout) 
    {
        case IWDG_TIMEOUT_1S:
            prescaler = IWDG_Prescaler_32;  /* 32 prescaler -> 1.25kHz */
            reload = 1250 - 1;              /* 1.25kHz * 1s = 1250 */
            break;
        case IWDG_TIMEOUT_2S:
            prescaler = IWDG_Prescaler_64;  /* 64 prescaler -> 625Hz */
            reload = 1250 - 1;              /* 625Hz * 2s = 1250 */
            break;
        case IWDG_TIMEOUT_4S:
            prescaler = IWDG_Prescaler_128; /* 128 prescaler -> 312.5Hz */
            reload = 1250 - 1;              /* 312.5Hz * 4s = 1250 */
            break;
        case IWDG_TIMEOUT_8S:
            prescaler = IWDG_Prescaler_256; /* 256 prescaler -> 156.25Hz */
            reload = 1250 - 1;              /* 156.25Hz * 8s = 1250 */
            break;
        case IWDG_TIMEOUT_16S:
            prescaler = IWDG_Prescaler_256; /* 256 prescaler -> 156.25Hz */
            reload = 2500 - 1;              /* 156.25Hz * 16s = 2500 */
            break;
        default:
            prescaler = IWDG_Prescaler_32;
            reload = 1250 - 1;
    }
    
    IWDG_SetPrescaler(prescaler);

    IWDG_SetReload(reload);

    IWDG_ReloadCounter();

    IWDG_Enable();
    
    return true;
}

/******************************************************************************
 * @brief   Feed IWDG
 * @details Reload IWDG counter to prevent reset
 * @param   None
 * @return  None
 *****************************************************************************/
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}

/******************************************************************************
 * @brief   Check IWDG reset cause
 * @details Check if system was reset by IWDG
 * @param   None
 * @return  uint8_t Reset cause status
 * @retval  1 System was reset by IWDG
 * @retval  0 System was not reset by IWDG
 *****************************************************************************/
uint8_t IWDG_CheckResetCause(void) 
{
    if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        RCC_ClearFlag();  
        return 1;         
    }
    return 0;             
}
