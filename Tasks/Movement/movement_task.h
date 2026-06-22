#ifndef __MOVEMENT_TASK_H__
#define __MOVEMENT_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  运动控制任务
  * @note   FreeRTOS 任务入口，250Hz 调用 SteeringChassis_ControlLoop
  */
void Motor_Movement(void *argument);

/**
  * @brief  M3508 底盘初始化
  * @note   从 freertos.c 的 MX_FREERTOS_Init 中调用
  */
void Movement_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOVEMENT_TASK_H__ */ 
