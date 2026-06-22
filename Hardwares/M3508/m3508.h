/**
  ******************************************************************************
  * @file           : m3508.h
  * @brief          : M3508 直流无刷减速电机驱动头文件（CAN 通信）
  * @description    : 电流控制、反馈解析（累计角度/转速/电流）、多电机管理
  ******************************************************************************
  */
#ifndef __M3508_H__
#define __M3508_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "config.h"
#include <stdint.h>

/* M3508 反馈帧结构 ----------------------------------------------------------*/
/* M3508 标准反馈：每个 CAN 帧含 2 个电机的数据，共 8 字节
 * 字节 0-1: 电机1 的累计角度 (uint16, 0-8191 => 0-360°)
 * 字节 2-3: 电机1 的转速 (int16, rpm)
 * 字节 4-5: 电机1 的电流 (int16, mA)
 * 字节 6:   电机2 的累计角度高 8 位
 * 字节 7:   电机2 的转速高 8 位
 * 注意：M3508 有 2 种反馈变体，部分回传 2 电机/帧，部分只 1 电机/帧
 * 本驱动按每个 CAN ID 对应 1 个电机（独立地址模式）设计
 */
#define M3508_FB_ANGLE_OFFSET   0
#define M3508_FB_SPEED_OFFSET   2
#define M3508_FB_CURRENT_OFFSET 4
#define M3508_FB_DATA_LEN       8

/* M3508 单电机反馈信息 -------------------------------------------------------*/
typedef struct
{
    uint16_t encoder;       /* 累计角度 (0-8191 对应 0-360°) */
    int16_t  rpm;           /* 转速 (rpm) */
    int16_t  current;       /* 实际电流 (mA) */
    uint8_t  updated;       /* 本周期是否有新反馈数据 */
} M3508_Feedback;

/* M3508 单电机控制 -----------------------------------------------------------*/

/**
  * @brief  初始化 M3508 驱动模块
  * @param  hcan: 指向 CAN_HandleTypeDef 的指针（通常 &hcan1）
  * @note   会注册 CAN RX 中断、初始化反馈数组
  */
void M3508_Init(CAN_HandleTypeDef *hcan);

/**
  * @brief  设置电机电流指令（不立即发送，由 M3508_SendAll 统一发送）
  * @param  motor_id: 电机编号 (1-8)
  * @param  current:  目标电流值 (mA, 范围 -16384 ~ +16384)
  */
void M3508_SetCurrent(uint8_t motor_id, int16_t current);

/**
  * @brief  批量发送所有 8 个电机的电流指令
  * @note   每个控制周期调用一次，内部逐帧发送 CAN 消息
  */
void M3508_SendAll(void);

/**
  * @brief  获取指定电机的最新反馈数据
  * @param  motor_id: 电机编号 (1-8)
  * @param  fb: 输出参数，接收反馈数据
  * @retval 1: 自上次调用后数据已刷新, 0: 无新数据
  */
uint8_t M3508_GetFeedback(uint8_t motor_id, M3508_Feedback *fb);

/**
  * @brief  CAN RX 中断回调，解析接收到的 M3508 反馈帧
  * @param  hcan: CAN 句柄指针
  * @note   应在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
  */
void M3508_RxCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* __M3508_H__ */ 
