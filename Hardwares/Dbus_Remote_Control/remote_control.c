/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : DJI DT7/DR16 DBUS 遥控接收机驱动实现
  * @description    : DMA CIRCULAR + UART IDLE 中断接收
  *                   DBUS 18 字节帧解析，归一化输出 [-1.0, +1.0]
  *                   失联保护状态机
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "remote_control.h"
#include <string.h>
#include <math.h>

/* 私有变量 ----------------------------------------------------------------*/
static UART_HandleTypeDef *remote_huart = NULL;

/* DMA 循环缓冲区 */
static uint8_t  dbus_rx_buffer[DBUS_BUFFER_SIZE];
static uint8_t  dbus_frame[DBUS_FRAME_LENGTH];

/* 时间戳 */
static uint32_t last_frame_tick = 0;
static uint32_t loss_entry_tick = 0;

/* 失联保护状态机 */
typedef enum {
    LOSS_IDLE   = 0,    /* 在线正常 */
    LOSS_ESTOP  = 1,    /* 急停保持中（刹车） */
    LOSS_CUTOFF = 2     /* 电流关断（自由滑行） */
} LossState;
static LossState loss_state = LOSS_IDLE;

/* 归一化后的遥控状态 */
static RemoteState cached_state;

/* --------------------------------------------------------------------------*/
/* 内部函数：摇杆 raw → 归一化 [-1.0, +1.0] 含死区                          */
/* --------------------------------------------------------------------------*/
static float RawToNormalized(uint16_t raw)
{
    float normalized;

    if (raw <= REMOTE_RAW_MIN) {
        normalized = -1.0f;
    } else if (raw >= REMOTE_RAW_MAX) {
        normalized = 1.0f;
    } else if (raw < REMOTE_RAW_MID) {
        /* 中位偏下 → 负区间 */
        normalized = -(float)(REMOTE_RAW_MID - raw) / (float)(REMOTE_RAW_MID - REMOTE_RAW_MIN);
    } else {
        /* 中位偏上 → 正区间 */
        normalized = (float)(raw - REMOTE_RAW_MID) / (float)(REMOTE_RAW_MAX - REMOTE_RAW_MID);
    }

    /* 死区处理 */
    if (normalized > -REMOTE_DEADBAND && normalized < REMOTE_DEADBAND) {
        normalized = 0.0f;
    }

    return normalized;
}

/* --------------------------------------------------------------------------*/
/* 内部函数：开关 raw → 三态枚举                                            */
/* --------------------------------------------------------------------------*/
static SwitchState RawToSwitch(uint8_t raw)
{
    /* DR16 开关三档：0x00=上, 0x01=中, 0x02=下 */
    if (raw == 0) return SWITCH_UP;
    if (raw == 1) return SWITCH_MID;
    return SWITCH_DOWN;
}

/* --------------------------------------------------------------------------*/
/* 内部函数：解析 DBUS 18 字节帧                                            */
/* --------------------------------------------------------------------------*/
static int ParseDbusFrame(const uint8_t *data)
{
    /* 帧头校验 */
    if (data[0] != DBUS_FRAME_HEADER) {
        return 0;
    }

    RemoteRaw raw;

    /* DR16 帧格式 (18 字节):
     * byte[0]   : 帧头 0xA0
     * byte[1-2] : CH0 (lx) 低字节在前, 11-bit
     * byte[3-4] : CH1 (ly)
     * byte[5-6] : CH2 (rx)
     * byte[7-8] : CH3 (ry)
     * byte[9-10]: CH4 (wheel)
     * byte[11]  : [7:0] 为 CH5 低 8 位
     * byte[12]  : [7:6] 为 CH5 高 2 位, [5:0] 为 CH6 低 6 位
     * byte[13]  : [7:5] 为 CH6 高 3 位
     * byte[14-17]: 保留及校验
     */

    raw.lx    = ((uint16_t)data[1] | ((uint16_t)data[2] << 8)) & 0x07FF;
    raw.ly    = ((uint16_t)data[3] | ((uint16_t)data[4] << 8)) & 0x07FF;
    raw.rx    = ((uint16_t)data[5] | ((uint16_t)data[6] << 8)) & 0x07FF;
    raw.ry    = ((uint16_t)data[7] | ((uint16_t)data[8] << 8)) & 0x07FF;
    raw.wheel = ((uint16_t)data[9] | ((uint16_t)data[10] << 8)) & 0x07FF;

    raw.sw1   = ((data[11] >> 0) & 0x03) | ((data[12] & 0xC0) >> 4);  /* 低 2 位 + 高 2 位 */
    raw.sw2   = ((data[12] & 0x3F) >> 0) | ((data[13] & 0xE0) >> 1);  /* 低 6 位 + 高 3 位 */

    /* 归一化 */
    cached_state.lx    = RawToNormalized(raw.lx);
    cached_state.ly    = RawToNormalized(raw.ly);
    cached_state.rx    = RawToNormalized(raw.rx);
    cached_state.ry    = RawToNormalized(raw.ry);
    cached_state.wheel = RawToNormalized(raw.wheel);
    cached_state.sw1   = RawToSwitch(raw.sw1);
    cached_state.sw2   = RawToSwitch(raw.sw2);
    cached_state.connected = 1;

    return 1;
}

/* ==========================================================================*/
/*                          公共 API 实现                                    */
/* ==========================================================================*/

/**
  * @brief  初始化遥控接收驱动
  */
void RemoteControl_Init(UART_HandleTypeDef *huart)
{
    remote_huart = huart;

    /* 清空缓冲区 */
    memset(dbus_rx_buffer, 0, DBUS_BUFFER_SIZE);
    memset(dbus_frame, 0, DBUS_FRAME_LENGTH);
    memset(&cached_state, 0, sizeof(RemoteState));

    loss_state  = LOSS_IDLE;
    last_frame_tick = 0;
    loss_entry_tick = 0;

    /* 启动 DMA 循环接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, dbus_rx_buffer, DBUS_BUFFER_SIZE);

    /* 使能 UART IDLE 中断 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

/**
  * @brief  UART IDLE 中断回调
  */
void RemoteControl_IdleCallback(UART_HandleTypeDef *huart)
{
    if (huart != remote_huart || remote_huart == NULL) {
        return;
    }

    uint32_t tick = HAL_GetTick();
    uint16_t ndtr;
    uint16_t received_bytes;

    /* 获取 DMA 剩余计数 → 计算本次接收字节数 */
    ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    received_bytes = DBUS_BUFFER_SIZE - ndtr;

    if (received_bytes < DBUS_FRAME_LENGTH) {
        /* 数据不足一帧，不处理 */
        return;
    }

    /* 在循环缓冲区中扫描帧头 */
    uint16_t i;
    for (i = 0; i <= received_bytes - DBUS_FRAME_LENGTH; i++) {
        if (dbus_rx_buffer[i] == DBUS_FRAME_HEADER) {
            /* 找到帧头，拷贝 18 字节并解析 */
            memcpy(dbus_frame, &dbus_rx_buffer[i], DBUS_FRAME_LENGTH);
            if (ParseDbusFrame(dbus_frame)) {
                last_frame_tick = tick;

                /* 从失联状态恢复 */
                if (loss_state != LOSS_IDLE) {
                    loss_state = LOSS_IDLE;
                    loss_entry_tick = 0;
                }
            }
            break;  /* 只处理找到的第一个帧头 */
        }
    }

    /* 重置 DMA 计数器，准备接收下一帧 */
    __HAL_DMA_DISABLE(huart->hdmarx);
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @brief  获取当前遥控器归一化状态
  */
void RemoteControl_GetState(RemoteState *state)
{
    if (state != NULL) {
        /*
         * P0-2 修复: 临界区保护
         * cached_state 可能被 ISR 中的 RemoteControl_IdleCallback 并发写入,
         * 此处禁用中断确保 memcpy 读到一致的状态 (Cortex-M4 上的原子性读)
         */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(state, &cached_state, sizeof(RemoteState));
        if (!primask) {
            __enable_irq();
        }
    }
}

/**
  * @brief  查询遥控器是否在线
  */
uint8_t RemoteControl_IsConnected(void)
{
    uint32_t tick = HAL_GetTick();
    uint32_t elapsed;

    /* 处理 tick 回绕 */
    if (tick >= last_frame_tick) {
        elapsed = tick - last_frame_tick;
    } else {
        elapsed = (0xFFFFFFFF - last_frame_tick) + tick + 1;
    }

    return (elapsed < REMOTE_TIMEOUT_MS) ? 1 : 0;
}

/**
  * @brief  获取最后一帧接收时间戳
  */
uint32_t RemoteControl_GetLastFrameTime(void)
{
    return last_frame_tick;
}

/**
  * @brief  失联保护状态机
  * @retval 0: 正常在线
  *         1: 刚进入失联 → 执行急停
  *         2: 急停保持中，超过 REMOTE_ESTOP_HOLD_MS → 关断电流
  */
uint8_t RemoteControl_LossProtectionTick(void)
{
    uint32_t tick = HAL_GetTick();
    uint32_t elapsed;

    /* 检查是否在线 */
    if (RemoteControl_IsConnected()) {
        return 0;   /* 正常在线 */
    }

    /* 失联处理 */
    switch (loss_state) {
        case LOSS_IDLE:
            /* 刚失联 → 进入急停状态 */
            loss_state = LOSS_ESTOP;
            loss_entry_tick = tick;
            return 1;   /* 通知上层执行急停 */

        case LOSS_ESTOP:
            /* 急停中，检查是否超时 */
            if (tick >= loss_entry_tick) {
                elapsed = tick - loss_entry_tick;
            } else {
                elapsed = (0xFFFFFFFF - loss_entry_tick) + tick + 1;
            }

            if (elapsed >= REMOTE_ESTOP_HOLD_MS) {
                loss_state = LOSS_CUTOFF;
                return 2;   /* 通知上层关断电流 */
            }
            return 0;   /* 保持急停 */

        case LOSS_CUTOFF:
            return 0;   /* 电流已关断 */

        default:
            return 0;
    }
}
