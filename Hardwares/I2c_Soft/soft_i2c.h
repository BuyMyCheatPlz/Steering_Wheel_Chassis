/**
  ******************************************************************************
  * @file           : soft_i2c.h
  * @brief          : 软件模拟 I2C 驱动头文件
  * @description    : 基于 DWT 延时的软件 I2C，SCL 频率 ~100kHz
  *                   支持多总线实例，通过 SoftI2C_Bus 结构体抽象
  ******************************************************************************
  */
#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* 软件 I2C 总线结构体 -------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef *sda_port;     /* SDA 所在 GPIO 端口 */
    uint16_t      sda_pin;      /* SDA 引脚编号     */
    GPIO_TypeDef *scl_port;     /* SCL 所在 GPIO 端口 */
    uint16_t      scl_pin;      /* SCL 引脚编号     */
    uint8_t       delay_us;     /* 半周期延时 (us), 默认 5us -> ~100kHz */
} SoftI2C_Bus;

/* I2C 状态 */
#define SOFT_I2C_OK     0
#define SOFT_I2C_ERR    1

/* 函数声明 ------------------------------------------------------------------*/
void SoftI2C_Init(SoftI2C_Bus *bus);
void SoftI2C_DelayUs(uint32_t us);

/* 底层总线操作 --------------------------------------------------------------*/
void SoftI2C_Start(SoftI2C_Bus *bus);
void SoftI2C_Stop(SoftI2C_Bus *bus);
uint8_t SoftI2C_WaitAck(SoftI2C_Bus *bus);
void SoftI2C_SendAck(SoftI2C_Bus *bus);
void SoftI2C_SendNack(SoftI2C_Bus *bus);
void SoftI2C_WriteByte(SoftI2C_Bus *bus, uint8_t data);
uint8_t SoftI2C_ReadByte(SoftI2C_Bus *bus);

/* 高层传输操作 --------------------------------------------------------------*/
uint8_t SoftI2C_WriteReg(SoftI2C_Bus *bus, uint8_t dev_addr,
                         uint8_t reg_addr, uint8_t data);
uint8_t SoftI2C_ReadReg(SoftI2C_Bus *bus, uint8_t dev_addr,
                        uint8_t reg_addr, uint8_t *data);
uint8_t SoftI2C_ReadRegs(SoftI2C_Bus *bus, uint8_t dev_addr,
                         uint8_t reg_addr, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_I2C_H__ */ 
