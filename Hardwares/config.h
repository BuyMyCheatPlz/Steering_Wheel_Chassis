/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : 舵轮底盘全局参数配置（所有可调宏均在此文件）
  ******************************************************************************
  */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 1. 底盘几何参数
 *============================================================================*/
#define WHEEL_BASE_A            0.15f   /* 前后轮半轴距 (m) */
#define TRACK_WIDTH_B           0.15f   /* 左右轮半轮距 (m) */
#define WHEEL_RADIUS            0.076f  /* 车轮半径 (m) */

/*==============================================================================
 * 2. 驱动轮减速比
 *============================================================================*/
#define REDUCTION_DRIVE_1       14.0f   /* 轮1 驱动减速比 */
#define REDUCTION_DRIVE_2       19.0f   /* 轮2 驱动减速比 */
#define REDUCTION_DRIVE_3       19.0f   /* 轮3 驱动减速比 */
#define REDUCTION_DRIVE_4       14.0f   /* 轮4 驱动减速比 */

/*==============================================================================
 * 3. 舵向轮减速比（未知，预留可配）
 *============================================================================*/
#define REDUCTION_STEER_1       14.0f   /* 轮1 舵向减速比 (待标定) */
#define REDUCTION_STEER_2       14.0f   /* 轮2 舵向减速比 (待标定) */
#define REDUCTION_STEER_3       14.0f   /* 轮3 舵向减速比 (待标定) */
#define REDUCTION_STEER_4       14.0f   /* 轮4 舵向减速比 (待标定) */

/*==============================================================================
 * 4. M3508 电机 CAN ID 配置
 *============================================================================*/
/* 驱动电机 ID 1-4 -> CAN StdID: 0x200 + ID */
#define CAN_ID_DRIVE_1          0x201
#define CAN_ID_DRIVE_2          0x202
#define CAN_ID_DRIVE_3          0x203
#define CAN_ID_DRIVE_4          0x204

/* 舵向电机 ID 5-8 -> CAN StdID: 0x200 + ID */
#define CAN_ID_STEER_1          0x205
#define CAN_ID_STEER_2          0x206
#define CAN_ID_STEER_3          0x207
#define CAN_ID_STEER_4          0x208

/*==============================================================================
 * 5. MT6701 编码器参数
 *============================================================================*/
#define MT6701_RESOLUTION       16384.0f /* 14位分辨率 */

/*==============================================================================
 * 6. 舵向上电回正目标原始值（14位原始值，标定后填入）
 *============================================================================*/
#define STEER_INIT_RAW_1        13600
#define STEER_INIT_RAW_2        11160
#define STEER_INIT_RAW_3        15568
#define STEER_INIT_RAW_4        3188

/*==============================================================================
 * 7. 舵向零位偏移（顺时针补偿，单位：度）
 *============================================================================*/
#define STEER_OFFSET_1          175.8f
#define STEER_OFFSET_2          126.6f
#define STEER_OFFSET_3          213.0f
#define STEER_OFFSET_4          313.0f

/*==============================================================================
 * 8. 回正到位阈值 (度)
 *============================================================================*/
#define HOMING_THRESHOLD_DEG    3.0f

/*==============================================================================
 * 9. 控制周期定义
 *============================================================================*/
#define CONTROL_DT              0.004f  /* PID 计算时间增量 (s), 250Hz */
#define CONTROL_PERIOD_MS       4       /* 主控制周期 (ms), 250Hz */
#define CONTROL_PERIOD_2MS      4       /* 主控制周期 (兼容旧宏) */
#define CAN_SEND_PERIOD_MS      4       /* CAN 发送周期 (ms) */
#define CAN_SEND_PERIOD_2MS     4       /* CAN 发送周期 (兼容旧宏) */

/*==============================================================================
 * 10. M3508 电流限制 (mA)
 *============================================================================*/
#define M3508_CURRENT_MAX       16384   /* 最大电流 16384mA */
#define M3508_CURRENT_MIN      -16384   /* 最小电流 -16384mA */

/*==============================================================================
 * 11. 电流 Ramp 斜率限幅 (mA/周期)
 *============================================================================*/
#define CURRENT_RAMP_RATE       4000    /* 每控制周期最大电流变化量 */

/*==============================================================================
 * 12. 舵向到位死区 (±度)
 *============================================================================*/
#define STEER_DEADBAND_DEG      1.0f    /* 舵角误差小于此值视为到位 */

/*==============================================================================
 * 13. 编码器故障容错
 *============================================================================*/
#define ENCODER_MAX_FAIL_COUNT  10      /* 连续失败次数阈值 */

/*==============================================================================
 * 14. PID — 舵向位置环（外环，每轮独立 Kp/Ki/Kd）
 *     轮1/4 减速比 14:1，轮2/3 减速比 19:1，允许差异化调参
 *============================================================================*/
#define STEER_POS_KP_1          8.0f
#define STEER_POS_KP_2          8.0f
#define STEER_POS_KP_3          8.0f
#define STEER_POS_KP_4          8.0f
#define STEER_POS_KI_1          0.2f
#define STEER_POS_KI_2          0.2f
#define STEER_POS_KI_3          0.2f
#define STEER_POS_KI_4          0.2f
#define STEER_POS_KD_1          0.0f
#define STEER_POS_KD_2          0.0f
#define STEER_POS_KD_3          0.0f
#define STEER_POS_KD_4          0.0f
#define STEER_POS_INTEGRAL_MAX  500.0f  /* 积分限幅（统一） */
#define STEER_POS_OUTPUT_MAX    1000.0f /* 输出限幅 (目标转速 dps)（统一） */

/*==============================================================================
 * 15. PID — 舵向速度环（内环，每轮独立 Kp/Ki/Kd）
 *============================================================================*/
#define STEER_VEL_KP_1          10.0f
#define STEER_VEL_KP_2          10.0f
#define STEER_VEL_KP_3          10.0f
#define STEER_VEL_KP_4          10.0f
#define STEER_VEL_KI_1          0.5f
#define STEER_VEL_KI_2          0.5f
#define STEER_VEL_KI_3          0.5f
#define STEER_VEL_KI_4          0.5f
#define STEER_VEL_KD_1          0.0f
#define STEER_VEL_KD_2          0.0f
#define STEER_VEL_KD_3          0.0f
#define STEER_VEL_KD_4          0.0f
#define STEER_VEL_INTEGRAL_MAX  5000.0f /* 积分限幅（统一） */
#define STEER_VEL_OUTPUT_MAX    16384.0f/* 输出限幅 (mA)（统一） */

/*==============================================================================
 * 16. PID — 驱动速度环（每轮独立 Kp/Ki/Kd）
 *     轮1/4 减速比 14:1，轮2/3 减速比 19:1，需要不同 PID
 *============================================================================*/
#define DRIVE_VEL_KP_1          8.0f
#define DRIVE_VEL_KP_2          8.0f
#define DRIVE_VEL_KP_3          8.0f
#define DRIVE_VEL_KP_4          8.0f
#define DRIVE_VEL_KI_1          0.3f
#define DRIVE_VEL_KI_2          0.3f
#define DRIVE_VEL_KI_3          0.3f
#define DRIVE_VEL_KI_4          0.3f
#define DRIVE_VEL_KD_1          0.0f
#define DRIVE_VEL_KD_2          0.0f
#define DRIVE_VEL_KD_3          0.0f
#define DRIVE_VEL_KD_4          0.0f
#define DRIVE_VEL_INTEGRAL_MAX  5000.0f
#define DRIVE_VEL_OUTPUT_MAX    16384.0f

/*==============================================================================
 * 17. GPIO 引脚定义（4 组独立软 I2C）
 *============================================================================*/
#define I2C1_SCL_PORT           GPIOA
#define I2C1_SCL_PIN            GPIO_PIN_10
#define I2C1_SDA_PORT           GPIOA
#define I2C1_SDA_PIN            GPIO_PIN_9

#define I2C2_SCL_PORT           GPIOB
#define I2C2_SCL_PIN            GPIO_PIN_11
#define I2C2_SDA_PORT           GPIOB
#define I2C2_SDA_PIN            GPIO_PIN_10

#define I2C3_SCL_PORT           GPIOD
#define I2C3_SCL_PIN            GPIO_PIN_2
#define I2C3_SDA_PORT           GPIOC
#define I2C3_SDA_PIN            GPIO_PIN_12

#define I2C4_SCL_PORT           GPIOC
#define I2C4_SCL_PIN            GPIO_PIN_7
#define I2C4_SDA_PORT           GPIOC
#define I2C4_SDA_PIN            GPIO_PIN_6

/*==============================================================================
 * 18. 遥控器参数
 *============================================================================*/
#define REMOTE_DEADBAND          0.05f   /* 摇杆死区（归一化值，5%） */
#define REMOTE_TIMEOUT_MS        50      /* 遥控失联超时 (ms) */
#define REMOTE_ESTOP_HOLD_MS     200     /* 失联急停保持时间 (ms) */

/* DR16 摇杆原始值范围 */
#define REMOTE_RAW_MIN           364     /* 摇杆最小值 */
#define REMOTE_RAW_MID           1024    /* 摇杆中位值 */
#define REMOTE_RAW_MAX           1684    /* 摇杆最大值 */

/* 最大运动速度 */
#define REMOTE_VEL_MAX_XY        3.0f    /* 最大平动速度 (m/s) */
#define REMOTE_VEL_MAX_WZ        6.283f  /* 最大旋转角速度 (rad/s, ≈1rps) */

/*==============================================================================
 * 19. 底盘速度指令结构体（队列元素）
 *============================================================================*/
typedef struct {
    float vx;   /* X 方向速度 (m/s) */
    float vy;   /* Y 方向速度 (m/s) */
    float wz;   /* 旋转角速度 (rad/s) */
} ChassisVelocityCmd;

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H__ */
