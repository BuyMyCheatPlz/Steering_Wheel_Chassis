#ifndef __VOFA_TRANSMIT_TASK_H__
#define __VOFA_TRANSMIT_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Vofa 调试数据传输任务
  * @note   每 10ms 读取舵轮状态并通过 UART4 发送 JustFloat 格式
  */
void Vofa_Debugging_Transmition(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_TRANSMIT_TASK_H__ */ 
