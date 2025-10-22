#ifndef __ARCH_SYS_ARCH_H__
#define __ARCH_SYS_ARCH_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

typedef SemaphoreHandle_t sys_sem_t;
typedef SemaphoreHandle_t sys_mutex_t;
typedef TaskHandle_t sys_thread_t;
typedef TickType_t sys_prot_t;

#define SYS_MBOX_NULL NULL
#define SYS_SEM_NULL  NULL

#endif /* __ARCH_SYS_ARCH_H__ */
