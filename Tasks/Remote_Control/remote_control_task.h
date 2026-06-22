#ifndef __REMOTE_CONTROL_TASK_H__
#define __REMOTE_CONTROL_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  遥控信号处理任务
  * @note   FreeRTOS 任务入口，等待 DBUS 信号量，解析并映射到底盘速度
  */
void Receive_and_Process_Signal(void *argument);

/**
  * @brief  遥控子系统初始化
  * @note   从 freertos.c MX_FREERTOS_Init 中调用
  */
void RemoteControlTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_CONTROL_TASK_H__ */

