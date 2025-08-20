/******************************************************************************
 * @file    main.c
 * @brief   Main application entry point
 * @details This file contains the main function which initializes all system
 *          peripherals and runs the main application loop
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "stm32f10x.h"                      /* Device header                 */
#include "IWDG.h"
#include "TIM3.h"
#include "ADC.h"
#include "bsp_led_driver.h" 
#include "bsp_pt100_driver.h"
#include "bsp_Water_driver.h"
#include "bsp_tsh54_driver.h"
#include "bsp_adxl345_driver.h"
#include "bsp_a22_driver.h"
#include "can.h"
#include "CanCmd.h"
#include "Sensor_Update.h"

/******************************************************************************
 * @brief   Configure system clock
 * @details Configure system clock to use PLL with HSI/2 as source and 9x 
 *          multiplier
 * @param   None
 * @return  None
 *****************************************************************************/
static void SystemClockConfig(void)
{
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
}

/******************************************************************************
 * @brief   Main function
 * @details System entry point, initializes all peripherals and runs main loop
 * @param   None
 * @return  int Program exit status
 *****************************************************************************/
int main()
{       
    SystemClockConfig();
    SysTick_Config(SystemCoreClock / 1000);
    IWDGInit(IWDG_TIMEOUT_16S);
    adc_hw_init();	
	sensor_driver_init();  
    CANInit(CAN_MODE_NORMAL, CAN_BAUDRATE_500K);
    can_buffer_init();	
    
    while(1)
    { 
		Sensor_Task();
        IWDG_Feed();
        can_task();
    }
}
