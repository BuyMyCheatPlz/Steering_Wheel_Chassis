/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "movement_task.h"
#include "remote_control_task.h"
#include "vofa_transmit_task.h"
#include "../../Hardwares/config.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Receive_Handle */
osThreadId_t Receive_HandleHandle;
const osThreadAttr_t Receive_Handle_attributes = {
  .name = "Receive_Handle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Movement_Handle */
osThreadId_t Movement_HandleHandle;
const osThreadAttr_t Movement_Handle_attributes = {
  .name = "Movement_Handle",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Vofa_Transmit */
osThreadId_t Vofa_TransmitHandle;
const osThreadAttr_t Vofa_Transmit_attributes = {
  .name = "Vofa_Transmit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Signal_Queue */
osMessageQueueId_t Signal_QueueHandle;
const osMessageQueueAttr_t Signal_Queue_attributes = {
  .name = "Signal_Queue"
};
/* Definitions for DbusReady */
osSemaphoreId_t DbusReadyHandle;
const osSemaphoreAttr_t DbusReady_attributes = {
  .name = "DbusReady"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Receive_and_Process_Signal(void *argument);
void Motor_Movement(void *argument);
void Vofa_Debugging_Transmition(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 初始化运动控制子系统（M3508 + MT6701 + 舵轮底盘） */
  Movement_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of DbusReady */
  DbusReadyHandle = osSemaphoreNew(1, 0, &DbusReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Signal_Queue */
  Signal_QueueHandle = osMessageQueueNew(16, sizeof(ChassisVelocityCmd), &Signal_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /*
   * 遥控子系统初始化必须放在信号量和队列创建之后，
   * 否则 UART IDLE 中断可能在 DbusReadyHandle 有效前触发，
   * ISR 中 xSemaphoreGiveFromISR(NULL) → HardFault
   */
  RemoteControlTask_Init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Receive_Handle */
  Receive_HandleHandle = osThreadNew(Receive_and_Process_Signal, NULL, &Receive_Handle_attributes);

  /* creation of Movement_Handle */
  Movement_HandleHandle = osThreadNew(Motor_Movement, NULL, &Movement_Handle_attributes);

  /* creation of Vofa_Transmit */
  Vofa_TransmitHandle = osThreadNew(Vofa_Debugging_Transmition, NULL, &Vofa_Transmit_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Receive_and_Process_Signal */
/**
* @brief Function implementing the Receive_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Receive_and_Process_Signal */
__weak void Receive_and_Process_Signal(void *argument)
{
  /* USER CODE BEGIN Receive_and_Process_Signal */
  /* 任务实现在 Tasks/Remote_Control/remote_control_task.c */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Receive_and_Process_Signal */
}

/* USER CODE BEGIN Header_Motor_Movement */
/**
* @brief Function implementing the Movement_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Movement */
__weak void Motor_Movement(void *argument)
{
  /* USER CODE BEGIN Motor_Movement */
  /* 任务实现在 Tasks/Movement/movement_task.c */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Movement */
}

/* USER CODE BEGIN Header_Vofa_Debugging_Transmition */
/**
* @brief Function implementing the Vofa_Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vofa_Debugging_Transmition */
__weak void Vofa_Debugging_Transmition(void *argument)
{
  /* USER CODE BEGIN Vofa_Debugging_Transmition */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Vofa_Debugging_Transmition */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 中断回调已移至 Tasks 层：
 *   HAL_CAN_RxFifo0MsgPendingCallback  -> Tasks/Movement/movement_task.c
 *   HAL_UARTEx_RxEventCallback          -> Tasks/Remote_Control/remote_control_task.c
 */
/* USER CODE END Application */

