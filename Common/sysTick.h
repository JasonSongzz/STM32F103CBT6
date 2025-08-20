#ifndef __SYSTICK_H
#define __SYSTICK_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// 标准模式 (100kHz) - 需要约5μs延时
#define I2C_STD_DELAY    {for(volatile int i=0; i<70; i++);}  

// 快速模式 (400kHz) - 需要约1.5μs延时  
#define I2C_FAST_DELAY   {for(volatile int i=0; i<20; i++);}

// 高速模式 (3.4MHz) - 需要约0.3μs延时
#define I2C_HIGH_DELAY   {for(volatile int i=0; i<5; i++);}

void inc_Tick(void);
uint32_t get_Tick(void);

#endif
