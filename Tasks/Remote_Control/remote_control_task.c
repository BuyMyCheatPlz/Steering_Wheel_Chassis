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
  *
  *         修复 P1-3:
  *           原代码将 RemoteControl_LossProtectionTick() 调用放在
  *           if (rc_state.connected) 块内，导致从未连上遥控器时
  *           失联保护状态机永远不会被调用，急停失效。
  *           修复后: 无论 rc_state.connected 为何，每周期都调用
  *           LossProtectionTick，确保失联状态机独立运行。
  */
void Receive_and_Process_Signal(void *argument)
{
    for (;;)
    {
        /* 等待 DBUS 帧解析完成信号量 */
        osSemaphoreAcquire(DbusReadyHandle, osWaitForever);

        RemoteState rc_state;
        RemoteControl_GetState(&rc_state);

        /*
         * P1-3 修复: LossProtectionTick 移到外层调用
         * 不依赖 rc_state.connected，确保即使从未连接也能触发急停
         */
        uint8_t loss = RemoteControl_LossProtectionTick();

        if (loss == 0)
        {
            /*
             * N1 修复: 急停恢复确认
             * 编码器已恢复 + 遥控器在线 + SW1拨到UP → 确认恢复
             */
            if (SteeringChassis_GetState() == CHASSIS_ESTOP_RECOVER &&
                rc_state.connected && rc_state.sw1 == SWITCH_UP)
            {
                SteeringChassis_Recover();
            }

            /* 在线/保持急停：正常控制 */
            if (rc_state.connected)
            {
                /* 正常在线：映射通道到速度 → 写入队列传递至运动任务 */
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
