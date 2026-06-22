/**
  ******************************************************************************
  * @file           : mt6701.c
  * @brief          : MT6701 磁性角度编码器驱动实现
  * @description    : 通过软件 I2C 读取 14 位角度值，转换为度数
  *                   MT6701 I2C 地址: 0x06
  *                   角度寄存器: 0x03 (高字节) + 0x04 (低字节)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mt6701.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define MT6701_ANGLE_MASK       0x3FFF  /* 14 位角度掩码 */
#define MT6701_RESOLUTION       16384.0f /* 14 位分辨率 (2^14) */

/* 公共函数实现 --------------------------------------------------------------*/

/**
  * @brief  初始化 MT6701 设备
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @param  bus: 指向已初始化的 SoftI2C_Bus 结构体的指针
  */
void MT6701_Init(MT6701_Device *dev, SoftI2C_Bus *bus)
{
    if (dev == NULL || bus == NULL) return;

    dev->i2c_bus     = bus;
    dev->raw_angle   = 0;
    dev->angle_degrees = 0.0f;
    dev->last_status = SOFT_I2C_ERR;
}

/**
  * @brief  读取 MT6701 原始角度值 (14 位)
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval SOFT_I2C_OK: 读取成功，raw_angle 和 angle_degrees 已更新
  * @retval SOFT_I2C_ERR: 读取失败
  * @note   从 0x03 开始连续读 2 字节，合并为 14 位角度值
  *         字节序: 寄存器 0x03 为高 6 位 [13:8]
  *                 寄存器 0x04 为低 8 位 [7:0]
  */
uint8_t MT6701_ReadAngle(MT6701_Device *dev)
{
    uint8_t buf[2];
    uint8_t status;

    if (dev == NULL || dev->i2c_bus == NULL) return SOFT_I2C_ERR;

    /* 从角度寄存器地址 0x03 连续读取 2 字节 */
    status = SoftI2C_ReadRegs(dev->i2c_bus, MT6701_ADDR,
                              MT6701_REG_ANGLE_MSB, buf, 2);

    dev->last_status = status;

    if (status == SOFT_I2C_OK)
    {
        /* 合并两个字节: MSB 为高 6 位, LSB 为低 8 位 */
        dev->raw_angle = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];

        /* 仅保留低 14 位有效数据 */
        dev->raw_angle &= MT6701_ANGLE_MASK;

        /* 转换为度数: angle = raw * 360.0 / 16384.0 */
        dev->angle_degrees = (float)dev->raw_angle * (360.0f / MT6701_RESOLUTION);
    }

    return status;
}

/**
  * @brief  获取当前角度值（度）
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval 角度值 (0.0f - 360.0f)
  * @note   返回上一次成功读取的角度值，不会发起新的 I2C 通信
  */
float MT6701_GetAngleDegrees(MT6701_Device *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->angle_degrees;
}

/**
  * @brief  获取当前原始角度值
  * @param  dev: 指向 MT6701_Device 结构体的指针
  * @retval 14 位原始角度值 (0 - 16383)
  * @note   返回上一次成功读取的原始值，不会发起新的 I2C 通信
  */
uint16_t MT6701_GetRawAngle(MT6701_Device *dev)
{
    if (dev == NULL) return 0;
    return dev->raw_angle;
}
