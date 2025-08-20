#include "sysTick.h"

static volatile uint64_t g_Tick = 0;

void inc_Tick(void)
{
    g_Tick++;
}

uint32_t get_Tick(void)
{
    return g_Tick;
}
