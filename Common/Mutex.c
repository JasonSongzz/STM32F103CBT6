#include "mutex.h"
#include "stm32f10x.h"
#include "core_cm3.h"  // 用于CMSIS核心函数
#include <stdlib.h>

// 初始化互斥锁
void mutex_init(mutex_t *mutex)
{
    if(mutex != NULL) 
	{
        mutex->locked = 0;
    }
}

// 阻塞式获取锁
void mutex_lock(mutex_t *mutex)
{
    if(mutex == NULL) return;
    
    uint32_t primask;
    while(1)
	{
        // 保存中断状态并禁用中断
        primask = __get_PRIMASK();
        __disable_irq();
        // 检查并获取锁
        if(mutex->locked == 0)
		{
            mutex->locked = 1;
            __set_PRIMASK(primask);  // 恢复中断状态
            return;
        }
        // 恢复中断并等待
        __set_PRIMASK(primask);
        // 减少CPU占用的等待策略
        for(volatile uint32_t i = 0; i < 500; i++);
    }
}

// 非阻塞式尝试获取锁
// 返回: 1=获取成功, 0=获取失败
int mutex_trylock(mutex_t *mutex)
{
    if(mutex == NULL) return 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int result = 0;
    if(mutex->locked == 0)
	{
        mutex->locked = 1;
        result = 1;
    }
    __set_PRIMASK(primask);
    return result;
}

// 释放锁
void mutex_unlock(mutex_t *mutex)
{
    if(mutex == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mutex->locked = 0;
    __set_PRIMASK(primask);
}
