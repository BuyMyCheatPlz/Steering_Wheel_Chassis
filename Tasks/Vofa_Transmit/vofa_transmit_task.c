/**
  ******************************************************************************
  * @file           : vofa_transmit_task.c
  * @brief          : Vofa 调试数据传输任务
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vofa_transmit_task.h"
#include "vofa_send.h"
#include "steering_chassis.h"

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
  * @brief  Vofa 调试数据传输任务入口
  * @note   每 10ms 读取舵轮状态并通过 UART4 发送 JustFloat 格式
  *         16 通道: steer_angle[4], steer_target[4], drive_rpm[4], drive_target[4]
  */
void Vofa_Debugging_Transmition(void *argument)
{
    float steer_angle[4];
    float steer_target[4];
    float drive_rpm[4];
    float drive_target[4];

    /* 获取 UART4 句柄 (STM32CubeMX 生成) */
    extern UART_HandleTypeDef huart4;

    for (;;)
    {
        /* 从底盘控制模块读取调试数据 */
        SteeringChassis_GetWheelDebug(steer_angle, steer_target,
                                      drive_rpm, drive_target);

        /* 通过 UART4 发送 JustFloat 帧 */
        Vofa_SendWheelDebug(&huart4, steer_angle, steer_target,
                            drive_rpm, drive_target);

        /* 10ms 周期 = 100Hz */
        osDelay(10);
    }
}
