/**
  ******************************************************************************
  * @file           : steering_chassis.c
  * @brief          : 舵轮底盘控制实现
  * @description    : 运动学解算 + 编码器读取 + 串级 PID + 电流控制
  *                   + 上电回正状态机 + 急停恢复机制
  *                   (P0/P1/P2 修复版 — 2024 诊断审查后)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "steering_chassis.h"
#include "mt6701.h"
#include "../../Hardwares/config.h"
#include <math.h>
#include <string.h>

/*==============================================================================
 * 宏定义
 *============================================================================*/
#define WHEEL_COUNT             4
#define MOTOR_ID_STEER_BASE     5       /* 舵向电机 ID 起始 (5,6,7,8) */
#define MOTOR_ID_DRIVE_BASE     1       /* 驱动电机 ID 起始 (1,2,3,4) */
#define RAD2DEG                 57.295779513f

/*
 * 最短路径反转滞回阈值 (度)
 * 进入反转需 > 95°, 退出反转需 < 85°, 10° 滞回区抗噪声
 */
#define STEER_HYST_ENTER        95.0f
#define STEER_HYST_EXIT         85.0f

/*
 * 急停恢复：编码器需连续健康读取次数
 * 连续 5 次 (约 500ms @100Hz 健康检查) 全部成功才允许恢复
 */
#define ENCODER_RECOVERY_COUNT  5

/*==============================================================================
 * 全局变量
 *============================================================================*/

static SteeringWheel   g_wheel[WHEEL_COUNT];      /* 4 个舵轮状态 */
static MT6701_Device   g_mt6701_dev[WHEEL_COUNT]; /* 4 个编码器设备 */
/* 底盘速度指令 */
static float g_vx = 0.0f, g_vy = 0.0f, g_wz = 0.0f;

/* 状态机 */
static ChassisState g_state = CHASSIS_HOMING;

/* 各轮零位偏移 (度) — SteeringChassis_Init 与 ReadEncoders 共用 */
static const float g_steer_offsets[WHEEL_COUNT] = {
    STEER_OFFSET_1, STEER_OFFSET_2,
    STEER_OFFSET_3, STEER_OFFSET_4
};

/*
 * P0-1 修复: PID 控制器改为每轮独立实例
 *
 * 原代码: PID_Controller g_steer_pos_pid;  (4 轮共享)
 * 问题:   ComputeSteerPID(0) 写入 integral → ComputeSteerPID(1) 读到被污染的积分
 * 修复:   改为数组，每轮独立 PID 状态
 */
static PID_Controller g_steer_pos_pid[WHEEL_COUNT]; /* 舵向位置环 ×4 */
static PID_Controller g_steer_vel_pid[WHEEL_COUNT]; /* 舵向速度环 ×4 */
static PID_Controller g_drive_vel_pid[WHEEL_COUNT]; /* 驱动速度环 ×4 */

/* H1 修复: 回正 PID 改为每轮独立实例，防止 4 轮共用导致积分交叉污染 */
static PID_Controller g_homing_pos_pid[WHEEL_COUNT];

/* 急停恢复：编码器连续健康次数计数器 */
static uint8_t g_encoder_healthy_count = 0;

/*==============================================================================
 * 私有函数声明
 *============================================================================*/
static void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
                     float integral_max, float output_max, float deadband);
static float PID_Compute(PID_Controller *pid, float error, float dt);
static void ReadEncoders(void);
static void ReadMotorFeedback(void);
static void InverseKinematics(void);
static void ComputeSteerPID(uint8_t wheel_idx);
static void ComputeDrivePID(uint8_t wheel_idx);
static void ApplyCurrentRamp(uint8_t wheel_idx);
static void SendMotorCurrents(void);
static float Wrap180(float angle_diff);
static float NormalizeAngle(float angle);

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
  * @brief  底盘控制初始化
  */
void SteeringChassis_Init(SoftI2C_Bus *i2c_buses[4],
                          CAN_HandleTypeDef *hcan)
{
    uint8_t i;

    /* 初始化 4 路软 I2C 总线（置空闲电平 + 校验延时参数） */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        SoftI2C_Init(i2c_buses[i]);
    }

    /* 初始化 MT6701 编码器设备 */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        MT6701_Init(&g_mt6701_dev[i], i2c_buses[i]);
    }

    /* 初始化 M3508 */
    M3508_Init(hcan);

    /* 初始化舵轮状态 */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        memset(&g_wheel[i], 0, sizeof(SteeringWheel));
        g_wheel[i].target_steer_angle = 0.0f;
        g_wheel[i].target_drive_speed = 0.0f;
        g_wheel[i].inverted = 0;
    }

    /*
     * 设置各轮回正目标角度 (14位原始值 -> 角度 -> 减去零位偏移)
     * 宏定义: STEER_INIT_RAW_1 ~ 4 (可在 config.h 中修改)
     */
    const uint16_t init_raw[WHEEL_COUNT] = {
        STEER_INIT_RAW_1, STEER_INIT_RAW_2,
        STEER_INIT_RAW_3, STEER_INIT_RAW_4
    };

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        /* 原始值 -> 度数 (14位 / 16384 * 360) */
        float raw_deg = (float)init_raw[i] / MT6701_RESOLUTION * 360.0f;
        /* 减去零位偏移，归一化到 [0, 360) */
        g_wheel[i].homing_target_angle =
            NormalizeAngle(raw_deg - g_steer_offsets[i]);
    }

    /*
     * P0-1 修复: 初始化 12 个独立 PID 实例 (3组 × 4轮)
     * 每轮使用独立的 Kp/Ki/Kd（config.h 中可独立配置，适配不同减速比）
     */
    const float steer_pos_kp[WHEEL_COUNT] = {
        STEER_POS_KP_1, STEER_POS_KP_2, STEER_POS_KP_3, STEER_POS_KP_4
    };
    const float steer_pos_ki[WHEEL_COUNT] = {
        STEER_POS_KI_1, STEER_POS_KI_2, STEER_POS_KI_3, STEER_POS_KI_4
    };
    const float steer_pos_kd[WHEEL_COUNT] = {
        STEER_POS_KD_1, STEER_POS_KD_2, STEER_POS_KD_3, STEER_POS_KD_4
    };
    const float steer_vel_kp[WHEEL_COUNT] = {
        STEER_VEL_KP_1, STEER_VEL_KP_2, STEER_VEL_KP_3, STEER_VEL_KP_4
    };
    const float steer_vel_ki[WHEEL_COUNT] = {
        STEER_VEL_KI_1, STEER_VEL_KI_2, STEER_VEL_KI_3, STEER_VEL_KI_4
    };
    const float steer_vel_kd[WHEEL_COUNT] = {
        STEER_VEL_KD_1, STEER_VEL_KD_2, STEER_VEL_KD_3, STEER_VEL_KD_4
    };
    const float drive_vel_kp[WHEEL_COUNT] = {
        DRIVE_VEL_KP_1, DRIVE_VEL_KP_2, DRIVE_VEL_KP_3, DRIVE_VEL_KP_4
    };
    const float drive_vel_ki[WHEEL_COUNT] = {
        DRIVE_VEL_KI_1, DRIVE_VEL_KI_2, DRIVE_VEL_KI_3, DRIVE_VEL_KI_4
    };
    const float drive_vel_kd[WHEEL_COUNT] = {
        DRIVE_VEL_KD_1, DRIVE_VEL_KD_2, DRIVE_VEL_KD_3, DRIVE_VEL_KD_4
    };

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        /* H3 修复: 启用 STEER_DEADBAND_DEG 死区 */
        PID_Init(&g_steer_pos_pid[i],
                 steer_pos_kp[i], steer_pos_ki[i], steer_pos_kd[i],
                 STEER_POS_INTEGRAL_MAX, STEER_POS_OUTPUT_MAX,
                 STEER_DEADBAND_DEG);

        PID_Init(&g_steer_vel_pid[i],
                 steer_vel_kp[i], steer_vel_ki[i], steer_vel_kd[i],
                 STEER_VEL_INTEGRAL_MAX, STEER_VEL_OUTPUT_MAX, 0.0f);

        PID_Init(&g_drive_vel_pid[i],
                 drive_vel_kp[i], drive_vel_ki[i], drive_vel_kd[i],
                 DRIVE_VEL_INTEGRAL_MAX, DRIVE_VEL_OUTPUT_MAX, 0.0f);
    }

    /*
     * H1 修复: 每轮回正 PID 独立实例
     * 可以用较温和的 Kp 防止到位过冲
     * 位置环输出限幅也缩小为正常的一半
     * H3 修复: 启用 STEER_DEADBAND_DEG 死区
     */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        PID_Init(&g_homing_pos_pid[i],
                 steer_pos_kp[i] * 0.5f,
                 steer_pos_ki[i] * 0.5f,
                 steer_pos_kd[i],
                 STEER_POS_INTEGRAL_MAX * 0.5f,
                 STEER_POS_OUTPUT_MAX * 0.5f,
                 HOMING_THRESHOLD_DEG);
    }

    /* 初始状态: 回正 */
    g_state = CHASSIS_HOMING;
}

/**
  * @brief  设置底盘速度指令 (公共接口，由遥控器解析任务或上层控制调用)
  * @param  vx: X 方向速度 (m/s)
  * @param  vy: Y 方向速度 (m/s)
  * @param  wz: 旋转角速度 (rad/s)
  */
void SteeringChassis_SetVelocity(float vx, float vy, float wz)
{
    g_vx = vx;
    g_vy = vy;
    g_wz = wz;
}

/**
  * @brief  急停
  */
void SteeringChassis_EmergencyStop(void)
{
    uint8_t i;

    g_state = CHASSIS_ESTOP;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        g_wheel[i].steer_current_out = 0;
        g_wheel[i].drive_current_out = 0;
        g_wheel[i].steer_current_prev = 0;
        g_wheel[i].drive_current_prev = 0;
    }

    /* 清零全部 PID 积分 — 防止急停期间 windup (P2-7: 用 PID_Reset) */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        PID_Reset(&g_steer_pos_pid[i]);
        PID_Reset(&g_steer_vel_pid[i]);
        PID_Reset(&g_drive_vel_pid[i]);
        PID_Reset(&g_homing_pos_pid[i]);
    }

    /* 重置恢复计数器 */
    g_encoder_healthy_count = 0;
}

/**
  * @brief  检查是否急停
  */
uint8_t SteeringChassis_IsStopped(void)
{
    return (g_state == CHASSIS_ESTOP) || (g_state == CHASSIS_ESTOP_RECOVER);
}

/**
  * @brief  获取当前底盘状态
  */
ChassisState SteeringChassis_GetState(void)
{
    return g_state;
}

/**
  * @brief  检查回正是否完成
  * @note   P2-8 修复: 原代码 return (g_state != CHASSIS_HOMING)
  *         急停时也返回 true，语义不精确
  *         修正为仅在 NORMAL 状态返回 true
  */
uint8_t SteeringChassis_IsHomingDone(void)
{
    return (g_state == CHASSIS_NORMAL);
}

/**
  * @brief  重置 PID 控制器 (P2-7: 新增函数)
  * @note   同时清零积分累计和上次误差，防止模式切换时虚假 D 项跳变
  */
void PID_Reset(PID_Controller *pid)
{
    if (pid != NULL)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
    }
}

/**
  * @brief  检查编码器是否全部恢复健康 (P1-4: 急停恢复用)
  * @retval 1: 全部编码器连续 N 次读取成功, 0: 未恢复
  */
uint8_t SteeringChassis_IsEncoderHealthy(void)
{
    uint8_t i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        if (!g_wheel[i].encoder_valid)
        {
            g_encoder_healthy_count = 0;
            return 0;
        }
    }

    g_encoder_healthy_count++;
    if (g_encoder_healthy_count >= ENCODER_RECOVERY_COUNT)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  急停恢复确认 (N1 修复)
  * @note   由遥控任务在检测到 SW1=UP 时调用
  *         从 CHASSIS_ESTOP_RECOVER 切回 CHASSIS_HOMING 重新回正
  */
void SteeringChassis_Recover(void)
{
    if (g_state == CHASSIS_ESTOP_RECOVER)
    {
        /* 重置编码器健康计数器，切回回正状态重新校准 */
        g_encoder_healthy_count = 0;
        g_state = CHASSIS_HOMING;
    }
}

/**
  * @brief  控制主循环 (每个控制周期调用一次)
  * @note   建议调用频率: 250Hz (4ms)
  */
void SteeringChassis_ControlLoop(void)
{
    uint8_t i;

    /* Step 1: 读取 4 路 MT6701 编码器 */
    ReadEncoders();

    /* Step 2: 获取 8 路 M3508 反馈 */
    ReadMotorFeedback();

    switch (g_state)
    {
    case CHASSIS_ESTOP:
        /* 急停模式下仅发送零电流 */
        for (i = 0; i < WHEEL_COUNT; i++)
        {
            g_wheel[i].steer_current_out = 0;
            g_wheel[i].drive_current_out = 0;
        }
        SendMotorCurrents();

        /* N1 修复: 每周期检查编码器是否恢复健康 */
        if (SteeringChassis_IsEncoderHealthy())
        {
            g_state = CHASSIS_ESTOP_RECOVER;
        }
        return;

    case CHASSIS_ESTOP_RECOVER:
        /*
         * P1-4 修复: 急停恢复状态
         * 每周期检查编码器健康状态，达标后等待遥控器 SW1=UP 确认
         * 确认后重新回正 → 进入正常模式
         *
         * 注意: 恢复条件由遥控任务层判断，此处仅发送零电流等待
         */
        for (i = 0; i < WHEEL_COUNT; i++)
        {
            g_wheel[i].steer_current_out = 0;
            g_wheel[i].drive_current_out = 0;
        }
        SendMotorCurrents();
        return;

    case CHASSIS_HOMING:
    {
        /* 回正阶段: 驱动电机不动，舵向电机走位置 PID 回到初始角度 */
        uint8_t all_done = 1;

        for (i = 0; i < WHEEL_COUNT; i++)
        {
            float error;
            float vel_target, vel_error;

            /* 驱动电机静止 */
            g_wheel[i].drive_current_out = 0;

            /* P2-10 修复: 舵向编码器无效 → 发送零电流，不保持旧电流 */
            if (!g_wheel[i].encoder_valid)
            {
                g_wheel[i].steer_current_out = 0;
                all_done = 0;
                continue;
            }

            /* 位置误差 (Wrap to ±180°) */
            error = Wrap180(g_wheel[i].homing_target_angle -
                            g_wheel[i].encoder_angle);

            /* 检查是否到位 */
            if (fabsf(error) > HOMING_THRESHOLD_DEG)
            {
                all_done = 0;
            }

            /* H1 修复: 位置环 -> 目标转速 (每轮独立回正 PID) */
            vel_target = PID_Compute(&g_homing_pos_pid[i], error, CONTROL_DT);

            /* 速度环 -> 电流 (回正复用 g_steer_vel_pid) */
            vel_error = vel_target - g_wheel[i].steer_rpm;
            g_wheel[i].steer_current_out =
                (int16_t)PID_Compute(&g_steer_vel_pid[i], vel_error, CONTROL_DT);
        }

        /* 电流 ramp + 发送 */
        for (i = 0; i < WHEEL_COUNT; i++)
        {
            ApplyCurrentRamp(i);
        }
        SendMotorCurrents();

        /* 全部到位 → 用 PID_Reset 清零所有 PID，切到正常模式 */
        if (all_done)
        {
            for (i = 0; i < WHEEL_COUNT; i++)
            {
                PID_Reset(&g_steer_pos_pid[i]);
                PID_Reset(&g_steer_vel_pid[i]);
                PID_Reset(&g_drive_vel_pid[i]);
                /* H1 修复: 回正 PID 也需清零 */
                PID_Reset(&g_homing_pos_pid[i]);
            }
            g_state = CHASSIS_NORMAL;
        }
        return;
    }

    case CHASSIS_NORMAL:
    default:
        break;
    }

    /* --- 正常控制模式 --- */

    /* Step 3: 逆运动学解算 */
    InverseKinematics();

    /* Step 4: 串级 PID */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        ComputeSteerPID(i);
        ComputeDrivePID(i);
    }

    /* Step 5: 电流 Ramp 限幅 */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        ApplyCurrentRamp(i);
    }

    /* Step 6: CAN 发送电流指令 */
    SendMotorCurrents();
}

/*==============================================================================
 * 私有函数实现
 *============================================================================*/

/**
  * @brief  初始化 PID 控制器
  */
static void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
                     float integral_max, float output_max, float deadband)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->deadband = deadband;
}

/**
  * @brief  PID 计算
  * @param  pid:   PID 控制器
  * @param  error: 当前误差
  * @param  dt:    时间增量 (s)
  * @return PID 输出
  */
static float PID_Compute(PID_Controller *pid, float error, float dt)
{
    float p_out, i_out, d_out, output;

    /* 死区 */
    if (fabsf(error) < pid->deadband)
    {
        error = 0.0f;
    }

    /* P */
    p_out = pid->kp * error;

    /* I (抗饱和积分) */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    i_out = pid->ki * pid->integral;

    /* D */
    if (dt > 0.0001f)
    {
        d_out = pid->kd * (error - pid->prev_error) / dt;
    }
    else
    {
        d_out = 0.0f;
    }
    pid->prev_error = error;

    output = p_out + i_out + d_out;

    /* 输出限幅 */
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < -pid->output_max)
        output = -pid->output_max;

    return output;
}

/**
  * @brief  读取 4 路 MT6701 编码器 (使用已有的 MT6701_ReadAngle API)
  * @note   读取成功后应用零位偏移归一化到 [0, 360)
  *         连续失败超阈值触发急停
  */
static void ReadEncoders(void)
{
    uint8_t  i, ret;
    float    angle;
    uint8_t  fail_count = 0;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        /* 使用 MT6701 驱动库读取角度 */
        ret = MT6701_ReadAngle(&g_mt6701_dev[i]);

        if (ret == SOFT_I2C_OK)
        {
            /* 获取角度并应用零位偏移 */
            angle = g_mt6701_dev[i].angle_degrees;
            g_wheel[i].encoder_angle =
                NormalizeAngle(angle - g_steer_offsets[i]);

            g_wheel[i].encoder_valid = 1;
            g_wheel[i].encoder_fail_count = 0;
        }
        else
        {
            /* 读取失败 */
            g_wheel[i].encoder_valid = 0;
            g_wheel[i].encoder_fail_count++;

            if (g_wheel[i].encoder_fail_count > ENCODER_MAX_FAIL_COUNT)
            {
                fail_count++;
            }
        }
    }

    /* 任一轮连续失败超阈值 -> 急停 */
    if (fail_count > 0)
    {
        SteeringChassis_EmergencyStop();
    }
}

/**
  * @brief  读取 8 路 M3508 反馈
  */
static void ReadMotorFeedback(void)
{
    uint8_t i;
    M3508_Feedback fb;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        /* 驱动电机反馈 (CAN ID 1-4) */
        if (M3508_GetFeedback(MOTOR_ID_DRIVE_BASE + i, &fb))
        {
            g_wheel[i].drive_rpm = (float)fb.rpm;
            g_wheel[i].drive_current_fb = fb.current;
        }

        /* 舵向电机反馈 (CAN ID 5-8) */
        if (M3508_GetFeedback(MOTOR_ID_STEER_BASE + i, &fb))
        {
            g_wheel[i].steer_rpm = (float)fb.rpm;
            g_wheel[i].steer_current_fb = fb.current;
        }
    }
}

/**
  * @brief  逆运动学解算
  * @note   从底盘 Vx, Vy, Wz 计算每轮目标舵角 + 目标轮速
  *
  *         坐标系 (俯视):
  *              Y↑ (左)
  *              |
  *         轮3  |  轮1
  *         ---- + ---- → X (前)
  *         轮4  |  轮2
  *
  *         轮坐标: 轮1(+a,+b), 轮2(+a,-b), 轮3(-a,+b), 轮4(-a,-b)
  *
  *         每轮速度 = V_body + Wz × r_i:
  *           V_ix = Vx - Wz * y_i
  *           V_iy = Vy + Wz * x_i
  *
  *         目标舵角 = atan2(V_iy, V_ix)
  *         目标轮速 = |V_i| / R_wheel -> rpm (WHEEL_RADIUS 来自 config.h)
  *
  *         最短路径优化 (P1-5 修复: 增加滞回):
  *           若 |目标舵角 - 当前舵角| > 95°, 进入反转
  *           若已反转且 |误差| < 85°, 退出反转
  *           反转驱动轮方向，目标舵角转 180°
  */
static void InverseKinematics(void)
{
    uint8_t i;
    float a = WHEEL_BASE_A;     /* 前后半轴距 (m) */
    float b = TRACK_WIDTH_B;    /* 左右半轮距 (m) */

    /* 轮坐标 (x, y) — x 前向, y 左向 */
    const float wheel_x[WHEEL_COUNT] = {  a,  a, -a, -a };
    const float wheel_y[WHEEL_COUNT] = {  b, -b,  b, -b };

    /* P2-9 修复: 驱动减速比 (电机转数 / 车轮转数) */
    const float drive_ratio[WHEEL_COUNT] = {
        REDUCTION_DRIVE_1, REDUCTION_DRIVE_2,
        REDUCTION_DRIVE_3, REDUCTION_DRIVE_4
    };

    /* 底盘速度 (m/s, rad/s) */
    float vx = g_vx;
    float vy = g_vy;
    float wz = g_wz;

    float V_ix, V_iy, speed, angle_rad;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        /* 轮心速度 = 底盘平动 + 旋转分量 */
        V_ix = vx - wz * wheel_y[i];
        V_iy = vy + wz * wheel_x[i];

        /* 合速度大小 -> 驱动轮转速 */
        speed = sqrtf(V_ix * V_ix + V_iy * V_iy);

        /*
         * 线速度 (m/s) -> rpm
         * rpm = (v / (2πR)) * 60
         * P2-9 修复: 乘以减速比 drive_ratio[i]
         * 车轮半径 WHEEL_RADIUS 在 config.h 中配置
         */
        g_wheel[i].target_drive_speed = speed *
            (60.0f / (2.0f * 3.141592654f * WHEEL_RADIUS)) *
            drive_ratio[i];

        /*
         * 目标舵角 = atan2(Vy, Vx)
         * 当合速度极小时保持当前角度不变
         */
        if (speed < 0.001f)
        {
            /* 停车时保持当前角度并清除反转标志 */
            if (g_wheel[i].encoder_valid)
            {
                g_wheel[i].target_steer_angle =
                    g_wheel[i].encoder_angle;
            }
            g_wheel[i].inverted = 0;
        }
        else
        {
            angle_rad = atan2f(V_iy, V_ix);
            g_wheel[i].target_steer_angle =
                NormalizeAngle(angle_rad * RAD2DEG);

            /* ─── 最短路径优化 (P1-5 修复: 施密特滞回) ───────
             * 使用 g_wheel[i].inverted 标志位 + 滞回阈值
             * 进入反转需 > STEER_HYST_ENTER (95°)
             * 退出反转需 < STEER_HYST_EXIT (85°)
             * 10° 滞回区消除编码器噪声导致的震荡
             */
            if (g_wheel[i].encoder_valid)
            {
                float steer_error = Wrap180(g_wheel[i].target_steer_angle -
                                            g_wheel[i].encoder_angle);
                float abs_error = fabsf(steer_error);

                if (!g_wheel[i].inverted && abs_error > STEER_HYST_ENTER)
                {
                    /* 进入反转 */
                    g_wheel[i].inverted = 1;
                    g_wheel[i].target_drive_speed = -g_wheel[i].target_drive_speed;
                    g_wheel[i].target_steer_angle =
                        NormalizeAngle(g_wheel[i].target_steer_angle + 180.0f);
                }
                else if (g_wheel[i].inverted && abs_error < STEER_HYST_EXIT)
                {
                    /* 退出反转 */
                    g_wheel[i].inverted = 0;
                    g_wheel[i].target_drive_speed = -g_wheel[i].target_drive_speed;
                    g_wheel[i].target_steer_angle =
                        NormalizeAngle(g_wheel[i].target_steer_angle + 180.0f);
                }
            }
        }
    }
}

/**
  * @brief  舵向串级 PID
  *         外环(位置) -> 目标转速 -> 内环(速度) -> 电流输出
  * @note   P0-1 修复: 使用 g_steer_pos_pid[wheel_idx] 独立实例
  */
static void ComputeSteerPID(uint8_t wheel_idx)
{
    float error, vel_target, vel_error;

    /* 位置环: 角度误差 (Wrap to ±180°) */
    error = Wrap180(g_wheel[wheel_idx].target_steer_angle -
                    g_wheel[wheel_idx].encoder_angle);

    g_wheel[wheel_idx].steer_pos_error = error;

    /*
     * P2-10 修复: 编码器无效时设 steer_current_out = 0
     * 而不是保持上一周期电流（避免舵角锁死在坏位置）
     */
    if (!g_wheel[wheel_idx].encoder_valid)
    {
        g_wheel[wheel_idx].steer_current_out = 0;
        return;
    }

    /* P0-1 修复: 使用独立 PID 实例 */
    vel_target = PID_Compute(&g_steer_pos_pid[wheel_idx], error, CONTROL_DT);

    g_wheel[wheel_idx].steer_vel_target = vel_target;

    /* 速度环: 转速误差 */
    vel_error = vel_target - g_wheel[wheel_idx].steer_rpm;

    g_wheel[wheel_idx].steer_current_out =
        (int16_t)PID_Compute(&g_steer_vel_pid[wheel_idx], vel_error, CONTROL_DT);
}

/**
  * @brief  驱动速度 PID
  * @note   P0-1 修复: 使用 g_drive_vel_pid[wheel_idx] 独立实例
  */
static void ComputeDrivePID(uint8_t wheel_idx)
{
    float error;

    error = g_wheel[wheel_idx].target_drive_speed -
            g_wheel[wheel_idx].drive_rpm;

    g_wheel[wheel_idx].drive_current_out =
        (int16_t)PID_Compute(&g_drive_vel_pid[wheel_idx], error, CONTROL_DT);
}

/**
  * @brief  电流 Ramp 限幅 (限制每周期电流变化量)
  */
static void ApplyCurrentRamp(uint8_t wheel_idx)
{
    int32_t diff;

    /* 舵向电机 */
    diff = (int32_t)g_wheel[wheel_idx].steer_current_out -
           (int32_t)g_wheel[wheel_idx].steer_current_prev;
    if (diff > CURRENT_RAMP_RATE)
    {
        g_wheel[wheel_idx].steer_current_out =
            g_wheel[wheel_idx].steer_current_prev + (int16_t)CURRENT_RAMP_RATE;
    }
    else if (diff < -CURRENT_RAMP_RATE)
    {
        g_wheel[wheel_idx].steer_current_out =
            g_wheel[wheel_idx].steer_current_prev - (int16_t)CURRENT_RAMP_RATE;
    }
    g_wheel[wheel_idx].steer_current_prev =
        g_wheel[wheel_idx].steer_current_out;

    /* 驱动电机 */
    diff = (int32_t)g_wheel[wheel_idx].drive_current_out -
           (int32_t)g_wheel[wheel_idx].drive_current_prev;
    if (diff > CURRENT_RAMP_RATE)
    {
        g_wheel[wheel_idx].drive_current_out =
            g_wheel[wheel_idx].drive_current_prev + (int16_t)CURRENT_RAMP_RATE;
    }
    else if (diff < -CURRENT_RAMP_RATE)
    {
        g_wheel[wheel_idx].drive_current_out =
            g_wheel[wheel_idx].drive_current_prev - (int16_t)CURRENT_RAMP_RATE;
    }
    g_wheel[wheel_idx].drive_current_prev =
        g_wheel[wheel_idx].drive_current_out;
}

/**
  * @brief  发送全部 8 路电流至 M3508
  */
static void SendMotorCurrents(void)
{
    uint8_t i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        M3508_SetCurrent(MOTOR_ID_DRIVE_BASE + i,
                         g_wheel[i].drive_current_out);
        M3508_SetCurrent(MOTOR_ID_STEER_BASE + i,
                         g_wheel[i].steer_current_out);
    }
    M3508_SendAll();
}

/**
  * @brief  角度差归一化到 [-180, 180]
  */
static float Wrap180(float angle_diff)
{
    while (angle_diff > 180.0f)  angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    return angle_diff;
}

/**
  * @brief  角度归一化到 [0, 360)
  */
static float NormalizeAngle(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;
    return angle;
}

/*==============================================================================
 * 调试数据获取 (供 Vofa 任务使用)
 *============================================================================*/

/**
  * @brief  获取 4 轮调试数据
  */
void SteeringChassis_GetWheelDebug(float steer_angle[4],
                                   float steer_target[4],
                                   float drive_rpm[4],
                                   float drive_target[4])
{
    uint8_t i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        steer_angle[i]  = g_wheel[i].encoder_angle;
        steer_target[i] = g_wheel[i].target_steer_angle;
        drive_rpm[i]    = g_wheel[i].drive_rpm;
        drive_target[i] = g_wheel[i].target_drive_speed;
    }
}
