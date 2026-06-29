/**
  ******************************************************************************
  * @file           : remote_control_task.c
  * @brief          : 遥控任务层 — FreeRTOS 任务入口 + UART 中断回调
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "remote_control_task.h"
#include "remote_control.h"
#include "steering_chassis.h"
#include "config.h"

/* 外部引用 — freertos.c 中 CubeMX 生成的句柄 */
extern osSemaphoreId_t DbusReadyHandle;
extern osMessageQueueId_t Signal_QueueHandle;

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
  * @brief  遥控子系统初始化
  * @note   从 freertos.c MX_FREERTOS_Init 中调用
  */
void RemoteControlTask_Init(void)
{
    /*
     * 初始化遥控驱动（启动 UART2 DMA 空闲接收）
     * huart2 在 main.c 中定义，由 STM32CubeMX 生成
     */
    extern UART_HandleTypeDef huart2;
    RemoteControl_Init(&huart2);
}

/**
  * @brief  遥控信号处理任务入口
  * @note   等待 DBUS 帧解析完成信号量，映射通道值到底盘速度
 *         10ms 超时：即使无 DBUS 帧也每周期执行失联保护检查
  *
  */
void Receive_and_Process_Signal(void *argument)
{
    for (;;)
    {
        /*
         * 等待 DBUS 帧信号量，10ms 超时
         * - osOK:           收到新 DBUS 帧
         * - osErrorTimeout: 10ms 内无帧，仍执行失联保护检查
         */
        osStatus_t status = osSemaphoreAcquire(DbusReadyHandle, 10U);

        RemoteState rc_state;
        RemoteControl_GetState(&rc_state);

        /*
         * LossProtectionTick 不依赖 rc_state.connected，
         * 确保即使从未连接也能触发急停
         */
        uint8_t loss = RemoteControl_LossProtectionTick();

        if (loss == 0)
        {
            /*
             * 急停恢复确认
             * 编码器已恢复 + 收到新 DBUS 帧 + 遥控器在线 + SW1拨到UP → 确认恢复
             *
             * 要求 status == osOK 确保收到新帧
             * cached_state.connected 从未被清零，失联后保持旧值 1
             * 若不要求新帧，stale 数据即可触发恢复，绕过手动确认的安全设计
             */
            if (SteeringChassis_GetState() == CHASSIS_ESTOP_RECOVER &&
                status == osOK &&
                rc_state.connected && rc_state.sw1 == SWITCH_UP)
            {
                SteeringChassis_Recover();
            }

            /*
             * 在线且收到新帧: 映射通道到速度 → 写入队列传递至运动任务
             * status == osOK 确保仅在收到新 DBUS 帧时发送，避免超时唤醒
             * 时重复入队 stale 数据
             */
            if (rc_state.connected && status == osOK)
            {
                float vx = rc_state.ly * REMOTE_VEL_MAX_XY;
                float vy = rc_state.lx * REMOTE_VEL_MAX_XY;
                float wz = rc_state.ry * REMOTE_VEL_MAX_WZ;

                ChassisVelocityCmd cmd = { .vx = vx, .vy = vy, .wz = wz };
                osMessageQueuePut(Signal_QueueHandle, &cmd, 0U, 0U);
            }
        }
        else if (loss == 1)
        {
            /* 刚进入失联 → 执行急停并发送零速 */
            ChassisVelocityCmd stop = { .vx = 0.0f, .vy = 0.0f, .wz = 0.0f };
            osMessageQueuePut(Signal_QueueHandle, &stop, 0U, 0U);
            SteeringChassis_EmergencyStop();
        }
        else if (loss == 2)
        {
            /* 急停超时 → 关断电流 */
            SteeringChassis_EmergencyStop();
        }
    }
}

/**
  * @brief  UART 空闲中断回调 (USART2 DBUS)
  * @note   触发 DBUS 帧解析并通过信号量通知遥控任务
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    RemoteControl_IdleCallback(huart);

    /* 使用 FreeRTOS FromISR 安全释放信号量 */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSemaphoreGiveFromISR((SemaphoreHandle_t)DbusReadyHandle, &xHigherPriorityTaskWoken) != pdFALSE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
