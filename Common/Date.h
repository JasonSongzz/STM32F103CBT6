#ifndef __DATE_H
#define __DATE_H

#include "stm32f10x.h"                  // Device header

#define INIT_YEAR          2000
#define INIT_MONTH         1
#define INIT_DAY           1
#define INIT_HOUR          0
#define INIT_MIN           0
#define INIT_SEC           0

#pragma pack(1)
typedef struct
{
	uint8_t   second;
	uint8_t   min;
	uint8_t   hour;
	uint8_t   day;
	uint8_t   month;
	uint16_t  year;
}p_clock_t;

typedef struct
{
	uint16_t millisec;
	p_clock_t clock;
}p_clock_ex_t;

#pragma pack()

#define _RTC_GET_FLAG_RTOFF(RTC)   ((RTC)->CRL & RTC_FLAG_RTOFF)
#define _RTC_SET_FLAG_CNF(RTC)   ((RTC)->CRL |= RTC_FLAG_RSF)
#define _RTC_GET_FLAG_CNF(RTC)   ((RTC)->CRL & RTC_FLAG_RSF)

/* API */
void RTC_Init(void);
void TickToDate(uint32_t Tick, p_clock_t *pClock);
void SetDateToTick(p_clock_t *pClock);
void GetDateClock(p_clock_t *pClock);

#endif


