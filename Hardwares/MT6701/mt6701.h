/**
  ******************************************************************************
  * @file           : mt6701.h
  * @brief          : MT6701 磁性角度编码器驱动头文件
  * @description    : 通过软件 I2C 读取 MT6701 的 14 位角度值
  ******************************************************************************
  */
#ifndef __MT6701_H__
#define __MT6701_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "soft_i2c.h"
#include <stdint.h>

/* MT6701 I2C 地址 -----------------------------------------------------------*/
#define MT6701_ADDR             0x06    /* 7 位 I2C 设备地址 */

/* MT6701 寄存器地址 ---------------------------------------------------------*/
#define MT6701_REG_ANGLE_MSB    0x03    /* 角度值高 6 位 [13:8] */
#define MT6701_REG_ANGLE_LSB    0x04    /* 角度值低 8 位 [7:0]  */

/* MT6701 设备结构体 ---------------------------------------------------------*/
typedef struct
{
    SoftI2C_Bus *i2c_bus;          /* 指向已初始化的 I2C 总线 */
    uint16_t     raw_angle;        /* 原始角度值 (14 位, 0-16383) */
    float        angle_degrees;    /* 角度值 (0.0 - 360.0) */
    uint8_t      last_status;      /* 最后一次通信状态 */
} MT6701_Device;

/* 函数声明 ------------------------------------------------------------------*/

/**
  * @brief  初始化 MT6701 设备
  * @param  dev:  指向 MT6701_Device 结构体的指针
  * @param  bus:  指向已初始化的 SoftI2C_Bus 结构体的指针
  */
void MT6701_Init(MT6701_Device *dev, SoftI2C_Bus *bus);

/**
  * @brief  读取 MT6701 原始角度值 (14 位)
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval SOFT_I2C_OK: 读取成功
  * @retval SOFT_I2C_ERR: 读取失败
  * @note   读取成功后，raw_angle 和 angle_degrees 会同步更新
  */
uint8_t MT6701_ReadAngle(MT6701_Device *dev);

/**
  * @brief  获取当前角度值（度）
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval 角度值 (0.0f - 360.0f)
  */
float MT6701_GetAngleDegrees(MT6701_Device *dev);

/**
  * @brief  获取当前原始角度值
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval 14 位原始角度值 (0-16383)
  */
uint16_t MT6701_GetRawAngle(MT6701_Device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __MT6701_H__ */ 
