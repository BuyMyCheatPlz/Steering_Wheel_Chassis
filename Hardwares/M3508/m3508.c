/**
  ******************************************************************************
  * @file           : m3508.c
  * @brief          : M3508 直流无刷减速电机驱动实现（CAN 通信）
  * @description    : 电流控制 + CAN 反馈解析
  *                   控制帧: 0x200 (电机 ID 1-4), 0x1FF (电机 ID 5-8)
  *                   反馈帧: 0x201-0x208 (每 ID 独立回传)
  *                   每反馈帧 8B: 角度(2B) + 转速(2B) + 电流(2B) + 保留(2B)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "m3508.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define M3508_MOTOR_COUNT       8       /* 总电机数 */
#define M3508_CTRL_GROUP1       0x200   /* 电机 1-4 电流控制帧 ID */
#define M3508_CTRL_GROUP2       0x1FF   /* 电机 5-8 电流控制帧 ID */

/* 私有变量 ------------------------------------------------------------------*/
static CAN_HandleTypeDef *m3508_hcan = NULL;

/* 电流指令缓存 [1..8] */
static int16_t m3508_current_cmd[M3508_MOTOR_COUNT + 1];

/* 反馈数据缓存 [1..8] */
static M3508_Feedback m3508_feedback[M3508_MOTOR_COUNT + 1];

/* 初始化标志 */
static uint8_t m3508_initialized = 0;

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
  * @brief  初始化 M3508 驱动模块
  * @param  hcan: 指向 CAN_HandleTypeDef 的指针（&hcan1）
  * @note   配置 CAN 过滤器接收 0x201-0x208 反馈帧，使能 RX 中断
  */
void M3508_Init(CAN_HandleTypeDef *hcan)
{
    uint8_t i;

    if (hcan == NULL) return;

    m3508_hcan = hcan;

    /* 初始化电流指令和反馈 */
    for (i = 0; i <= M3508_MOTOR_COUNT; i++)
    {
        m3508_current_cmd[i]    = 0;
        m3508_feedback[i].encoder = 0;
        m3508_feedback[i].rpm     = 0;
        m3508_feedback[i].current = 0;
        m3508_feedback[i].updated = 0;
    }

    /*
     * 配置 CAN 过滤器：接收 0x201 - 0x208 反馈帧
     *
     * STM32F4 32位 ID 掩码寄存器布局 (位 31 → 0):
     *   [31:21] = STDID[10:0]
     *   [20:3]  = EXID[17:0]  (标准帧时应忽略)
     *   [2]     = IDE  (0=标准帧)
     *   [1]     = RTR  (0=数据帧)
     *   [0]     = 0
     *
     * 目标范围: STDID = 0b 010 0000 0XXX (0x201-0x208)
     * 匹配: STDID[10:3]=0b01000000, IDE=0, RTR=0
     * 忽略: STDID[2:0], EXID 全部, 最低位
     */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x201 << 5;    /* Filter[31:16]: STDID=0x201 */
    sFilterConfig.FilterIdLow          = 0x0000;         /* Filter[15:0]: IDE=0, RTR=0 */
    sFilterConfig.FilterMaskIdHigh     = 0x00FF;         /* Mask[31:16]: 忽略 STDID[7:0] (匹配 0x200-0x2FF) */
    sFilterConfig.FilterMaskIdLow      = 0xFFF8;         /* Mask[15:3]=1忽略EXID, [2:1]=0匹配IDE/RTR */
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        m3508_initialized = 0;
        return;
    }

    /* 启动 CAN */
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        m3508_initialized = 0;
        return;
    }

    /* 使能 RX FIFO0 消息挂起中断 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    m3508_initialized = 1;
}

/**
  * @brief  设置指定电机电流指令（不立即发送，由 M3508_SendAll 统一发送）
  * @param  motor_id: 电机编号 (1-8)
  * @param  current:  目标电流值 (mA, -16384 ~ +16384)
  */
void M3508_SetCurrent(uint8_t motor_id, int16_t current)
{
    if (motor_id < 1 || motor_id > M3508_MOTOR_COUNT) return;

    /* 电流限幅 */
    if (current > M3508_CURRENT_MAX)  current = M3508_CURRENT_MAX;
    if (current < M3508_CURRENT_MIN)  current = M3508_CURRENT_MIN;

    m3508_current_cmd[motor_id] = current;
}

/**
  * @brief  批量发送全部 8 个电机的电流指令
  * @note   发送 2 帧：
  *         0x200: motor 1-4 电流数据（8 字节连续存放）
  *         0x1FF: motor 5-8 电流数据
  */
void M3508_SendAll(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t            tx_mailbox;
    uint8_t             tx_data[8];

    if (!m3508_initialized || m3508_hcan == NULL) return;

    tx_header.IDE      = CAN_ID_STD;
    tx_header.RTR      = CAN_RTR_DATA;
    tx_header.DLC      = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 帧 1: 电机 1-4 (ID: 0x200) */
    tx_header.StdId = M3508_CTRL_GROUP1;
    tx_data[0] = (uint8_t)((m3508_current_cmd[1] >> 8) & 0xFF);
    tx_data[1] = (uint8_t)( m3508_current_cmd[1]       & 0xFF);
    tx_data[2] = (uint8_t)((m3508_current_cmd[2] >> 8) & 0xFF);
    tx_data[3] = (uint8_t)( m3508_current_cmd[2]       & 0xFF);
    tx_data[4] = (uint8_t)((m3508_current_cmd[3] >> 8) & 0xFF);
    tx_data[5] = (uint8_t)( m3508_current_cmd[3]       & 0xFF);
    tx_data[6] = (uint8_t)((m3508_current_cmd[4] >> 8) & 0xFF);
    tx_data[7] = (uint8_t)( m3508_current_cmd[4]       & 0xFF);
    HAL_CAN_AddTxMessage(m3508_hcan, &tx_header, tx_data, &tx_mailbox);

    /* 帧 2: 电机 5-8 (ID: 0x1FF) */
    tx_header.StdId = M3508_CTRL_GROUP2;
    tx_data[0] = (uint8_t)((m3508_current_cmd[5] >> 8) & 0xFF);
    tx_data[1] = (uint8_t)( m3508_current_cmd[5]       & 0xFF);
    tx_data[2] = (uint8_t)((m3508_current_cmd[6] >> 8) & 0xFF);
    tx_data[3] = (uint8_t)( m3508_current_cmd[6]       & 0xFF);
    tx_data[4] = (uint8_t)((m3508_current_cmd[7] >> 8) & 0xFF);
    tx_data[5] = (uint8_t)( m3508_current_cmd[7]       & 0xFF);
    tx_data[6] = (uint8_t)((m3508_current_cmd[8] >> 8) & 0xFF);
    tx_data[7] = (uint8_t)( m3508_current_cmd[8]       & 0xFF);
    HAL_CAN_AddTxMessage(m3508_hcan, &tx_header, tx_data, &tx_mailbox);
}

/**
  * @brief  获取指定电机的最新反馈数据
  * @param  motor_id: 电机编号 (1-8)
  * @param  fb: 输出参数，接收反馈数据
  * @retval 1: 数据已更新, 0: 无新数据或参数无效
  */
uint8_t M3508_GetFeedback(uint8_t motor_id, M3508_Feedback *fb)
{
    if (motor_id < 1 || motor_id > M3508_MOTOR_COUNT || fb == NULL)
        return 0;

    *fb = m3508_feedback[motor_id];

    if (m3508_feedback[motor_id].updated)
    {
        m3508_feedback[motor_id].updated = 0;
        return 1;
    }
    return 0;
}

/**
  * @brief  CAN RX FIFO0 消息挂起中断回调
  * @param  hcan: CAN 句柄指针
  * @note   解析 0x201-0x208 反馈帧
  *         应在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
  */
void M3508_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
    uint8_t             motor_id;

    if (hcan == NULL) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        /* 仅处理标准帧 & 数据帧 */
        if (rx_header.IDE != CAN_ID_STD || rx_header.RTR != CAN_RTR_DATA)
            continue;

        /* 仅处理 0x201-0x208 */
        if (rx_header.StdId < 0x201 || rx_header.StdId > 0x208)
            continue;

        motor_id = (uint8_t)(rx_header.StdId - 0x200);

        if (motor_id < 1 || motor_id > 8) continue;

        /*
         * 反馈数据格式（每帧 8 字节）:
         *   [0-1] 机械角度 (uint16, 0-8191 => 0-360°)
         *   [2-3] 转子转速 (int16, rpm)
         *   [4-5] 实际转矩电流 (int16, mA)
         *   [6-7] 保留
         */
        m3508_feedback[motor_id].encoder = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        m3508_feedback[motor_id].rpm     = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
        m3508_feedback[motor_id].current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
        m3508_feedback[motor_id].updated = 1;
    }
}
