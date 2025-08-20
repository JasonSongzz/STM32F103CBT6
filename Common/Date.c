#include "stm32f10x.h"                  // Device header
#include "Date.h"
#include "stm32f10x_it.h"

static int is_leap(uint16_t year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

static uint8_t month_days(uint8_t month, uint16_t year)
{
    const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    return (month == 2 && is_leap(year)) ? 29 : days[month-1];
}

static void TickConversionTime(uint32_t Tick, p_clock_t *pClock)
{
    pClock->second = Tick % 60;
    Tick /= 60;
    pClock->min = Tick % 60;
    Tick /= 60;
    pClock->hour = Tick % 24;
    uint32_t days = Tick / 24;

    pClock->year = 2000;
    while(days >= (is_leap(pClock->year) ? 366UL : 365UL))
	{
        days -= is_leap(pClock->year) ? 366 : 365;
        pClock->year++;
    }

    pClock->month = 1;
    while(days >= month_days(pClock->month, pClock->year))
	{
        days -= month_days(pClock->month, pClock->year);
        pClock->month++;
    }
    pClock->day = days + 1; 
}

static inline int is_Leap(uint16_t year) 
{
    return (year & 3) ? 0 : (year % 100 != 0) || (year % 400 == 0);
}

static const uint8_t days_in_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};

static uint32_t TimeConversionTick(p_clock_t *pClock)
{
	uint32_t total_days = 0;
    
    for(uint16_t y = 2000; y < pClock->year; ++y)
	{
        total_days += is_Leap(y) ? 366 : 365;
    }

    for(uint8_t m = 1; m < pClock->month; ++m)
	{
        total_days += days_in_month[m-1];
        if(m == 2 && is_Leap(pClock->year)) ++total_days;
    }

    total_days += pClock->day - 1;

    return total_days * 86400UL 
           + pClock->hour * 3600UL 
           + pClock->min * 60UL 
           + pClock->second;
}

void TickToDate(uint32_t Tick, p_clock_t *pClock)
{
	TickConversionTime(Tick, pClock);
}

void GetDateClock(p_clock_t *pClock)
{
	uint32_t Cnt = 0;
	
	Cnt = GetTickCount();

	TickConversionTime(Cnt, pClock);
}

void SetDateToTick(p_clock_t *pClock)
{
	uint32_t Tick = 0;
	Tick = TimeConversionTick(pClock);
	SetTickCount(Tick);
}
