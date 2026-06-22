/**
  ******************************************************************************
  * @file           : vofa_send.c
  * @brief          : Vofa JustFloat 协议发送 — 硬件抽象层
  ******************************************************************************
  */

#include "vofa_send.h"
#include <string.h>

/*==============================================================================
 * JustFloat 协议发送
 *============================================================================*/

/**
  * @brief  发送 JustFloat 格式数据帧
  * @param  huart: UART 句柄
  * @param  data:  float 数组 (小端序)
  * @param  count: float 个数
  * @note   帧尾: [0x00 0x00 0x80 0x7F] = +Infinity (JustFloat 结束标志)
  */
void Vofa_SendJustFloat(UART_HandleTypeDef *huart, const float *data, uint8_t count)
{
    if (huart == NULL || data == NULL || count == 0) return;

    /* 帧缓冲区: count 个 float × 4 字节 + 4 字节帧尾 */
    uint8_t buf[68];  /* 最大 16 float = 64 + 4 = 68 字节 */
    uint8_t *p = buf;

    /* 拷贝 float 数据 (小端序) */
    memcpy(p, data, count * sizeof(float));
    p += count * sizeof(float);

    /* 帧尾: 0x00 0x00 0x80 0x7F (JustFloat 结束标志, 大端序表示 +Inf) */
    p[0] = 0x00;
    p[1] = 0x00;
    p[2] = 0x80;
    p[3] = 0x7F;
    p += 4;

    /* UART 阻塞发送 (Vofa 任务中调用, 不在中断上下文) */
    HAL_UART_Transmit(huart, buf, (uint16_t)(p - buf), 10);
}

/**
  * @brief  Vofa 舵轮底盘调试帧 (16 通道)
  * @param  huart:           UART 句柄 (通常为 huart4)
  * @param  steer_angle[4]:  舵向当前角度 (度)
  * @param  steer_target[4]: 舵向目标角度 (度)
  * @param  drive_rpm[4]:    驱动当前转速 (rpm)
  * @param  drive_target[4]: 驱动目标转速 (rpm)
  */
void Vofa_SendWheelDebug(UART_HandleTypeDef *huart,
                         const float steer_angle[4],
                         const float steer_target[4],
                         const float drive_rpm[4],
                         const float drive_target[4])
{
    float data[16];

    /* 通道 0-3:  舵向当前角度 */
    data[0] = steer_angle[0];
    data[1] = steer_angle[1];
    data[2] = steer_angle[2];
    data[3] = steer_angle[3];

    /* 通道 4-7:  舵向目标角度 */
    data[4] = steer_target[0];
    data[5] = steer_target[1];
    data[6] = steer_target[2];
    data[7] = steer_target[3];

    /* 通道 8-11: 驱动当前转速 */
    data[8]  = drive_rpm[0];
    data[9]  = drive_rpm[1];
    data[10] = drive_rpm[2];
    data[11] = drive_rpm[3];

    /* 通道 12-15: 驱动目标转速 */
    data[12] = drive_target[0];
    data[13] = drive_target[1];
    data[14] = drive_target[2];
    data[15] = drive_target[3];

    Vofa_SendJustFloat(huart, data, 16);
}
