/**
  ******************************************************************************
  * @file           : soft_i2c.c
  * @brief          : 软件模拟 I2C 驱动实现
  * @description    : 基于 DWT 周期计数器实现微秒级延时，
  *                   SCL 频率 ~100kHz (半周期 5us)
  *                   使用 HAL_GPIO 操作，支持多总线实例
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "soft_i2c.h"

/* 全局总线定义 — 由 movement_task.c 引用 ----------------------------------*/
SoftI2C_Bus bus1 = { GPIOA, SDA_1_Pin, GPIOA, SCL_1_Pin, 5 };
SoftI2C_Bus bus2 = { GPIOB, SDA_2_Pin, GPIOB, SCL_2_Pin, 5 };
SoftI2C_Bus bus3 = { GPIOC, SDA_3_Pin, GPIOD, SCL_3_Pin, 5 };
SoftI2C_Bus bus4 = { GPIOC, SDA_4_Pin, GPIOC, SCL_4_Pin, 5 };

/* 私有宏定义 ----------------------------------------------------------------*/

/* SDA 操作宏 */
#define SDA_H(bus)  HAL_GPIO_WritePin((bus)->sda_port, (bus)->sda_pin, GPIO_PIN_SET)
#define SDA_L(bus)  HAL_GPIO_WritePin((bus)->sda_port, (bus)->sda_pin, GPIO_PIN_RESET)
#define SDA_IN(bus) HAL_GPIO_ReadPin((bus)->sda_port, (bus)->sda_pin)

/* SCL 操作宏 */
#define SCL_H(bus)  HAL_GPIO_WritePin((bus)->scl_port, (bus)->scl_pin, GPIO_PIN_SET)
#define SCL_L(bus)  HAL_GPIO_WritePin((bus)->scl_port, (bus)->scl_pin, GPIO_PIN_RESET)

/* 默认半周期延时 (us)，对应 ~100kHz SCL */
#define SOFT_I2C_DEFAULT_DELAY_US  5

/* 私有函数声明 --------------------------------------------------------------*/

/**
  * @brief  检查并初始化 DWT 周期计数器
  * @note   必须先启用 DWT 的 TRC 位
  */
static void DWT_Init(void)
{
    /* 解锁 DWT 访问 */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* 重置周期计数器 */
    DWT->CYCCNT = 0;

    /* 使能周期计数器 */
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/* 公共函数实现 --------------------------------------------------------------*/

/**
  * @brief  微秒级延时（基于 DWT 周期计数器）
  * @param  us: 延时的微秒数
  * @note   HCLK = 168MHz, 1us = 168 个周期
  *         首次调用会自动初始化 DWT
  */
void SoftI2C_DelayUs(uint32_t us)
{
    static uint8_t dwt_initialized = 0;

    if (!dwt_initialized)
    {
        DWT_Init();
        dwt_initialized = 1;
    }

    uint32_t start_tick = DWT->CYCCNT;
    uint32_t wait_ticks = us * (SystemCoreClock / 1000000U);
    uint32_t elapsed;

    /* 等待指定周期数，处理 32 位计数器溢出 */
    do
    {
        elapsed = DWT->CYCCNT - start_tick;
    }
    while (elapsed < wait_ticks);
}

/**
  * @brief  初始化软件 I2C 总线
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  * @note   将 SDA 和 SCL 都置为高电平（空闲状态）
  */
void SoftI2C_Init(SoftI2C_Bus *bus)
{
    if (bus == NULL) return;

    /* 确保 DWT 已初始化 */
    SoftI2C_DelayUs(1);

    /* 设置默认延时 */
    if (bus->delay_us == 0)
    {
        bus->delay_us = SOFT_I2C_DEFAULT_DELAY_US;
    }

    /* 总线空闲状态：SDA 和 SCL 都拉高 */
    SDA_H(bus);
    SCL_H(bus);
}

/**
  * @brief  发送 I2C 起始信号
  *         SCL 高电平期间 SDA 从高变低
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  */
void SoftI2C_Start(SoftI2C_Bus *bus)
{
    SDA_H(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_H(bus);
    SoftI2C_DelayUs(bus->delay_us);

    SDA_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
}

/**
  * @brief  发送 I2C 停止信号
  *         SCL 高电平期间 SDA 从低变高
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  */
void SoftI2C_Stop(SoftI2C_Bus *bus)
{
    SDA_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_H(bus);
    SoftI2C_DelayUs(bus->delay_us);

    SDA_H(bus);
    SoftI2C_DelayUs(bus->delay_us);
}

/**
  * @brief  等待从机 ACK 应答
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  * @retval SOFT_I2C_OK:  收到 ACK (SDA 低电平)
  * @retval SOFT_I2C_ERR: 收到 NACK (SDA 高电平) 或超时
  */
uint8_t SoftI2C_WaitAck(SoftI2C_Bus *bus)
{
    uint8_t ack;
    uint32_t timeout = 0;

    /* 释放 SDA，设为输入检测 */
    SDA_H(bus);                         /* 输出高 = 释放总线 (开漏) */
    SoftI2C_DelayUs(bus->delay_us);

    /* 产生第 9 个时钟脉冲 */
    SCL_H(bus);
    SoftI2C_DelayUs(bus->delay_us);

    /* 检测 SDA 电平，带超时 */
    do
    {
        ack = SDA_IN(bus);
        timeout++;
    }
    while (ack == GPIO_PIN_SET && timeout < 1000);

    SCL_L(bus);
    SoftI2C_DelayUs(bus->delay_us);

    if (ack == GPIO_PIN_RESET)
    {
        return SOFT_I2C_OK;
    }
    else
    {
        return SOFT_I2C_ERR;
    }
}

/**
  * @brief  主设备发送 ACK 应答
  *         在第 9 个时钟位拉低 SDA
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  */
void SoftI2C_SendAck(SoftI2C_Bus *bus)
{
    SDA_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_H(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SDA_H(bus);
}

/**
  * @brief  主设备发送 NACK 应答
  *         在第 9 个时钟位保持 SDA 高电平
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  */
void SoftI2C_SendNack(SoftI2C_Bus *bus)
{
    SDA_H(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_H(bus);
    SoftI2C_DelayUs(bus->delay_us);
    SCL_L(bus);
    SoftI2C_DelayUs(bus->delay_us);
}

/**
  * @brief  向 I2C 总线写入一个字节
  *         高位在前 (MSB first)
  * @param  bus:  指向 SoftI2C_Bus 结构体的指针
  * @param  data: 要发送的 8 位数据
  */
void SoftI2C_WriteByte(SoftI2C_Bus *bus, uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        /* 先放数据，后拉高时钟 */
        if (data & 0x80)
        {
            SDA_H(bus);
        }
        else
        {
            SDA_L(bus);
        }
        SoftI2C_DelayUs(bus->delay_us);

        SCL_H(bus);
        SoftI2C_DelayUs(bus->delay_us);
        SCL_L(bus);
        SoftI2C_DelayUs(bus->delay_us);

        data <<= 1;
    }
}

/**
  * @brief  从 I2C 总线读取一个字节
  *         高位在前 (MSB first)
  * @param  bus: 指向 SoftI2C_Bus 结构体的指针
  * @retval 读取到的 8 位数据
  */
uint8_t SoftI2C_ReadByte(SoftI2C_Bus *bus)
{
    uint8_t i, data = 0;

    /* 释放 SDA，准备读取 */
    SDA_H(bus);
    SoftI2C_DelayUs(bus->delay_us);

    for (i = 0; i < 8; i++)
    {
        data <<= 1;

        SCL_H(bus);
        SoftI2C_DelayUs(bus->delay_us);

        /* 在 SCL 高电平期间读取 SDA */
        if (SDA_IN(bus) == GPIO_PIN_SET)
        {
            data |= 0x01;
        }

        SCL_L(bus);
        SoftI2C_DelayUs(bus->delay_us);
    }

    return data;
}

/* 高层传输操作 --------------------------------------------------------------*/

/**
  * @brief  向 I2C 设备的指定寄存器写入一个字节
  * @param  bus:      指向 SoftI2C_Bus 结构体的指针
  * @param  dev_addr: 7 位设备地址
  * @param  reg_addr: 寄存器地址
  * @param  data:     要写入的数据
  * @retval SOFT_I2C_OK:  写入成功
  * @retval SOFT_I2C_ERR: 写入失败（设备无应答）
  */
uint8_t SoftI2C_WriteReg(SoftI2C_Bus *bus, uint8_t dev_addr,
                         uint8_t reg_addr, uint8_t data)
{
    uint8_t result;

    SoftI2C_Start(bus);

    /* 发送设备地址 + 写标志 (R/W=0) */
    SoftI2C_WriteByte(bus, (dev_addr << 1) | 0x00);
    if (SoftI2C_WaitAck(bus) != SOFT_I2C_OK)
    {
        SoftI2C_Stop(bus);
        return SOFT_I2C_ERR;
    }

    /* 发送寄存器地址 */
    SoftI2C_WriteByte(bus, reg_addr);
    if (SoftI2C_WaitAck(bus) != SOFT_I2C_OK)
    {
        SoftI2C_Stop(bus);
        return SOFT_I2C_ERR;
    }

    /* 发送数据 */
    SoftI2C_WriteByte(bus, data);
    result = SoftI2C_WaitAck(bus);

    SoftI2C_Stop(bus);
    return result;
}

/**
  * @brief  从 I2C 设备的指定寄存器读取一个字节
  * @param  bus:      指向 SoftI2C_Bus 结构体的指针
  * @param  dev_addr: 7 位设备地址
  * @param  reg_addr: 寄存器地址
  * @param  data:     指向接收缓冲区的指针
  * @retval SOFT_I2C_OK:  读取成功
  * @retval SOFT_I2C_ERR: 读取失败（设备无应答）
  */
uint8_t SoftI2C_ReadReg(SoftI2C_Bus *bus, uint8_t dev_addr,
                        uint8_t reg_addr, uint8_t *data)
{
    return SoftI2C_ReadRegs(bus, dev_addr, reg_addr, data, 1);
}

/**
  * @brief  从 I2C 设备的指定寄存器连续读取多字节
  * @param  bus:      指向 SoftI2C_Bus 结构体的指针
  * @param  dev_addr: 7 位设备地址
  * @param  reg_addr: 起始寄存器地址
  * @param  data:     指向接收缓冲区的指针
  * @param  len:      要读取的字节数
  * @retval SOFT_I2C_OK:  读取成功
  * @retval SOFT_I2C_ERR: 读取失败（设备无应答）
  */
uint8_t SoftI2C_ReadRegs(SoftI2C_Bus *bus, uint8_t dev_addr,
                         uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == NULL || len == 0) return SOFT_I2C_ERR;

    /* 第一阶段：发送写地址 + 寄存器地址 */
    SoftI2C_Start(bus);

    SoftI2C_WriteByte(bus, (dev_addr << 1) | 0x00);
    if (SoftI2C_WaitAck(bus) != SOFT_I2C_OK)
    {
        SoftI2C_Stop(bus);
        return SOFT_I2C_ERR;
    }

    SoftI2C_WriteByte(bus, reg_addr);
    if (SoftI2C_WaitAck(bus) != SOFT_I2C_OK)
    {
        SoftI2C_Stop(bus);
        return SOFT_I2C_ERR;
    }

    /* 第二阶段：重复起始 + 读地址 */
    SoftI2C_Start(bus);

    SoftI2C_WriteByte(bus, (dev_addr << 1) | 0x01);
    if (SoftI2C_WaitAck(bus) != SOFT_I2C_OK)
    {
        SoftI2C_Stop(bus);
        return SOFT_I2C_ERR;
    }

    /* 连续读取数据 */
    for (i = 0; i < len; i++)
    {
        data[i] = SoftI2C_ReadByte(bus);

        /* 最后一个字节发 NACK，其余发 ACK */
        if (i == (len - 1))
        {
            SoftI2C_SendNack(bus);
        }
        else
        {
            SoftI2C_SendAck(bus);
        }
    }

    SoftI2C_Stop(bus);
    return SOFT_I2C_OK;
}
