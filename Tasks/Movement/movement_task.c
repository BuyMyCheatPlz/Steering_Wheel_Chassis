/**
  ******************************************************************************
  * @file           : movement_task.c
  * @brief          : 运动控制任务层 — FreeRTOS 任务入口 + CAN 中断回调
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "movement_task.h"
#include "steering_chassis.h"
#include "soft_i2c.h"
#include "m3508.h"

/* 外部引用 — freertos.c 中 CubeMX 生成的队列句柄 */
extern osMessageQueueId_t Signal_QueueHandle;

/* Private variables ---------------------------------------------------------*/
static uint8_t initialized = 0;

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
  * @brief  底盘运动控制初始化
  * @note   从 freertos.c MX_FREERTOS_Init 中调用一次
  */
void Movement_Init(void)
{
    if (initialized) return;
    initialized = 1;

    /* 获取 4 路软 I2C 总线 */
    extern SoftI2C_Bus bus1, bus2, bus3, bus4;
    SoftI2C_Bus *i2c_buses[4] = { &bus1, &bus2, &bus3, &bus4 };

    /* 初始化舵轮底盘（含 M3508、MT6701 编码器） */
    extern CAN_HandleTypeDef hcan1;
    SteeringChassis_Init(i2c_buses, &hcan1);
}

/**
  * @brief  运动控制任务入口
  * @note   H2 修复: 使用 vTaskDelayUntil 精确控制周期
  *         周期由 control_period_ms (CONTROL_PERIOD_MS) 定义, 与 CONTROL_DT 一致
  */
void Motor_Movement(void *argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    /* 确保已初始化 */
    Movement_Init();

    /* 记录起始 tick，用于 vTaskDelayUntil 精确定时 */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /*
         * 从 Signal_Queue 取最新速度指令（非阻塞）
         * 队列存满时仅保留最新值，旧值直接丢弃
         */
        ChassisVelocityCmd cmd;
        while (osMessageQueueGet(Signal_QueueHandle, &cmd, NULL, 0U) == osOK)
        {
            SteeringChassis_SetVelocity(cmd.vx, cmd.vy, cmd.wz);
        }

        SteeringChassis_ControlLoop();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
  * @brief  CAN RX FIFO0 消息挂起回调
  * @note   转发至 M3508 驱动进行反馈帧解析
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    M3508_RxCallback(hcan);
}
