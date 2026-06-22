/**
  ******************************************************************************
  * @file           : remote_control.h
  * @brief          : DJI DT7/DR16 DBUS 遥控接收机驱动头文件
  * @description    : DMA CIRCULAR + UART IDLE 中断接收
  *                   DBUS 18 字节帧解析，归一化输出 [-1.0, +1.0]
  *                   失联保护：50ms 超时 → 急停 200ms → 关断电流
  ******************************************************************************
  */
#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "config.h"
#include <stdint.h>

/* 常量定义 ----------------------------------------------------------------*/
#define DBUS_FRAME_LENGTH       18      /* 帧长度 (字节) */
#define DBUS_FRAME_HEADER       0xA0    /* 帧头标识 */
#define DBUS_BUFFER_SIZE        64      /* DMA 循环缓冲区大小 (≥36 足够) */

/* 开关状态枚举 ------------------------------------------------------------*/
typedef enum {
    SWITCH_DOWN = 0,
    SWITCH_MID  = 1,
    SWITCH_UP   = 2
} SwitchState;

/* 遥控器全部通道原始值 ----------------------------------------------------*/
typedef struct {
    /* 摇杆通道 raw 值 [364, 1684] */
    uint16_t lx;        /* CH0: 右摇杆左右 (Roll) */
    uint16_t ly;        /* CH1: 右摇杆上下 (Pitch) */
    uint16_t rx;        /* CH2: 左摇杆上下 (Throttle) */
    uint16_t ry;        /* CH3: 左摇杆左右 (Yaw) */
    /* 拨轮 raw 值 */
    uint16_t wheel;     /* CH4: 左拨轮 */
    /* 开关 raw 值 */
    uint8_t  sw1;       /* CH5: SW1 (三档) */
    uint8_t  sw2;       /* CH6: SW2 (三档) */
} RemoteRaw;

/* 遥控器归一化通道值 [-1.0, +1.0] -----------------------------------------*/
typedef struct {
    float lx;           /* CH0: 归一化左右方向 */
    float ly;           /* CH1: 归一化前后方向 */
    float rx;           /* CH2: 归一化油门杆上下 (预留) */
    float ry;           /* CH3: 归一化油门杆左右 */
    float wheel;        /* CH4: 归一化拨轮 [-1.0, +1.0] */
    SwitchState sw1;    /* SW1 三态 */
    SwitchState sw2;    /* SW2 三态 */
    uint8_t connected;  /* 遥控在线标志 */
} RemoteState;

/* 公共 API ----------------------------------------------------------------*/

/**
  * @brief  初始化遥控接收驱动
  * @param  huart: UART 句柄指针 (应为 &huart2)
  * @note   使能 UART IDLE 中断，启动 DMA 循环接收
  */
void RemoteControl_Init(UART_HandleTypeDef *huart);

/**
  * @brief  UART IDLE 中断处理回调
  * @param  huart: 触发中断的 UART 句柄
  * @note   在 HAL_UARTEx_RxEventCallback 中调用
  *         解析 DMA 缓冲区中的 DBUS 帧
  */
void RemoteControl_IdleCallback(UART_HandleTypeDef *huart);

/**
  * @brief  获取当前遥控器归一化状态
  * @param  state: 输出参数，接收全部归一化通道值和连接状态
  * @note   需在拿到 DbusReady 信号量后调用
  */
void RemoteControl_GetState(RemoteState *state);

/**
  * @brief  查询遥控器是否在线
  * @retval 1: 在线, 0: 失联
  */
uint8_t RemoteControl_IsConnected(void);

/**
  * @brief  获取最后一帧接收时间戳
  * @retval HAL_GetTick() 值 (ms)
  */
uint32_t RemoteControl_GetLastFrameTime(void);

/**
  * @brief  检查是否需要触发失联保护动作
  * @note   供 Receive_and_Process_Signal 任务每周期调用
  * @retval 0: 正常, 1: 刚进入失联 → 需急停, 2: 保持急停中
  */
uint8_t RemoteControl_LossProtectionTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_CONTROL_H__ */ 
