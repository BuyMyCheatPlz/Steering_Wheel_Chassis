#ifndef __VOFA_SEND_H__
#define __VOFA_SEND_H__

#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Vofa JustFloat 协议发送
  * @param  huart: UART 句柄
  * @param  data:  float 数组指针
  * @param  count: float 个数 (最大 16)
  * @note   协议格式: [float0 LSB...MSB] ... [floatN LSB...MSB] [0x00 0x00 0x80 0x7F]
  *         每个 float 4 字节小端序，帧尾固定 4 字节
  */
void Vofa_SendJustFloat(UART_HandleTypeDef *huart, const float *data, uint8_t count);

/**
  * @brief  Vofa 舵轮底盘调试帧 (16 通道 JustFloat)
  * @param  steer_angle[4]:     4 轮舵向当前角度 (度)
  * @param  steer_target[4]:    4 轮舵向目标角度 (度)
  * @param  drive_rpm[4]:       4 轮驱动当前转速 (rpm)
  * @param  drive_target[4]:    4 轮驱动目标转速 (rpm)
  * @note   使用 huart4 发送，16 个 float 共 68 字节
  */
void Vofa_SendWheelDebug(UART_HandleTypeDef *huart,
                         const float steer_angle[4],
                         const float steer_target[4],
                         const float drive_rpm[4],
                         const float drive_target[4]);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_SEND_H__ */
