#ifndef __MUTEX_H
#define __MUTEX_H

#include <stdint.h>

#pragma pack(1)

// 互斥锁结构体
typedef struct {
    volatile uint8_t locked;  // 锁状态标志 (0=空闲, 1=已锁定)
}mutex_t;

// 函数声明
void mutex_init(mutex_t *mutex);
void mutex_lock(mutex_t *mutex);
int mutex_trylock(mutex_t *mutex);
void mutex_unlock(mutex_t *mutex);

#endif /* __MUTEX_H */
