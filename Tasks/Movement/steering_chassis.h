/**
  ******************************************************************************
  * @file           : steering_chassis.h
  * @brief          : 舵轮底盘控制头文件
  * @description    : 运动学解算 + 编码器管理 + 串级 PID + 控制主循环
  ******************************************************************************
  */
#ifndef __STEERING_CHASSIS_H__
#define __STEERING_CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../config.h"
#include "m3508.h"
#include "soft_i2c.h"
#include <stdint.h>

/* 底盘控制状态枚举 ---------------------------------------------------------*/
typedef enum
{
    CHASSIS_HOMING = 0,         /* 上电回正中 */
    CHASSIS_NORMAL = 1,         /* 正常控制 */
    CHASSIS_ESTOP  = 2,         /* 急停 */
    CHASSIS_ESTOP_RECOVER = 3   /* 急停恢复：编码器健康检查 + 等待确认 */
} ChassisState;

/* 方向结构体 ---------------------------------------------------------------*/
typedef struct
{
    float x;    /* X 分量 */
    float y;    /* Y 分量 */
} Vector2D;

/* 单个舵轮状态 -------------------------------------------------------------*/
typedef struct
{
    /* --- 编码器角度 (度) --- */
    float encoder_angle;        /* 当前实际角度 [0, 360) */
    uint8_t encoder_valid;      /* 编码器本次读取成功 */

    /* --- 舵向电机反馈 --- */
    float steer_rpm;            /* 舵向电机转速 (rpm) */
    int16_t steer_current_fb;   /* 舵向电机电流 (mA) */

    /* --- 驱动电机反馈 --- */
    float drive_rpm;            /* 驱动电机转速 (rpm) */
    int16_t drive_current_fb;   /* 驱动电机电流 (mA) */

    /* --- 上电回正目标 --- */
    float homing_target_angle;  /* 回正目标角度 (度) [0, 360) */

    /* --- 目标值 --- */
    float target_steer_angle;   /* 目标舵角 (度) [0, 360) */
    float target_drive_speed;   /* 目标驱动轮速度 (rpm) */

    /* --- PID 中间量 --- */
    float steer_pos_error;      /* 舵向位置误差 (±180°) */
    float steer_vel_target;     /* 舵向速度环目标 (rpm) */

    /* --- 输出电流 --- */
    int16_t steer_current_out;  /* 舵向电机电流输出 (mA) */
    int16_t drive_current_out;  /* 驱动电机电流输出 (mA) */

    /* --- 上一次电流值 (ramp 用) --- */
    int16_t steer_current_prev;
    int16_t drive_current_prev;

    /* --- 编码器故障计数 --- */
    uint8_t encoder_fail_count;

    /* --- 最短路径反转标志（滞回用） --- */
    uint8_t inverted;           /* 1: 驱动方向已反转, 0: 正常 */

} SteeringWheel;

/* PID 控制器结构体 ---------------------------------------------------------*/
typedef struct
{
    float kp, ki, kd;           /* PID 系数 */
    float integral;             /* 积分累计 */
    float prev_error;           /* 上次误差 (D 项) */
    float integral_max;         /* 积分限幅 */
    float output_max;           /* 输出限幅 (±) */
    float deadband;             /* 死区 */
} PID_Controller;

/* 底盘控制接口 -------------------------------------------------------------*/

/**
  * @brief  底盘控制初始化
  * @param  i2c_buses: 4 个 I2C 总线指针数组 [轮1, 轮2, 轮3, 轮4]
  * @param  hcan: CAN 句柄指针
  * @note   初始化编码器 I2C、M3508、PID 参数
  */
void SteeringChassis_Init(SoftI2C_Bus *i2c_buses[4],
                          CAN_HandleTypeDef *hcan);

/**
  * @brief  底盘控制主循环（每个控制周期调用一次）
  * @note   内部流程：
  *         1. 读取 4 路 MT6701 编码器
  *         2. 获取 8 路 M3508 反馈
  *         3. 从 Signal_Queue 获取 Vx Vy Wz（队列传递，运动任务消费）
  *         4. 逆运动学解算 -> 各轮目标
  *         5. 串级 PID 计算电流指令
  *         6. 电流 ramp 限幅
  *         7. CAN 发送
  */
void SteeringChassis_ControlLoop(void);

/**
  * @brief  设置底盘速度指令（外部接口，由遥控器解析任务调用）
  * @param  vx: X 方向速度 (m/s)
  * @param  vy: Y 方向速度 (m/s)
  * @param  wz: 旋转角速度 (rad/s)
  */
void SteeringChassis_SetVelocity(float vx, float vy, float wz);

/**
  * @brief  急停：所有电机电流置零
  */
void SteeringChassis_EmergencyStop(void);

/**
  * @brief  急停恢复确认 (N1 修复)
  * @note   由遥控任务在检测到 SW1=UP 时调用
  *         从 CHASSIS_ESTOP_RECOVER 切回 CHASSIS_HOMING 重新回正
  */
void SteeringChassis_Recover(void);

/**
  * @brief  检查底盘是否处于急停状态
  * @retval 1: 急停中, 0: 正常
  */
uint8_t SteeringChassis_IsStopped(void);

/**
  * @brief  获取当前底盘状态
  * @retval ChassisState 枚举值
  */
ChassisState SteeringChassis_GetState(void);

/**
  * @brief  检查回正是否完成
  * @retval 1: 已完成, 0: 未完成
  */
uint8_t SteeringChassis_IsHomingDone(void);

/**
  * @brief  重置 PID 控制器（清零积分 + 上次误差）
  * @param  pid: PID 控制器指针
  */
void PID_Reset(PID_Controller *pid);

/**
  * @brief  检查编码器是否全部恢复健康
  * @note   用于急停恢复判断，每控制周期调用一次
  * @retval 1: 全部编码器连续 N 次读取成功, 0: 未恢复
  */
uint8_t SteeringChassis_IsEncoderHealthy(void);

/**
  * @brief  获取 4 轮调试数据 (供 Vofa 任务读取)
  * @param  steer_angle[4]:  输出舵向当前角度 (度)
  * @param  steer_target[4]: 输出舵向目标角度 (度)
  * @param  drive_rpm[4]:    输出驱动当前转速 (rpm)
  * @param  drive_target[4]: 输出驱动目标转速 (rpm)
  */
void SteeringChassis_GetWheelDebug(float steer_angle[4],
                                   float steer_target[4],
                                   float drive_rpm[4],
                                   float drive_target[4]);

#ifdef __cplusplus
}
#endif

#endif /* __STEERING_CHASSIS_H__ */ 
