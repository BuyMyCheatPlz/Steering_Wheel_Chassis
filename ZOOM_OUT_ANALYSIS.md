# 舵轮底盘控制系统 — Zoom Out 架构分析

> **文档目的**：从高层次视角审视整个系统架构，梳理模块关系、数据流、控制流、领域术语，并整合逻辑审查发现的架构性改进机会。

---

## 目录

1. [系统俯瞰](#1-系统俯瞰)
2. [领域术语表](#2-领域术语表)
3. [模块地图](#3-模块地图)
4. [数据流与线程模型](#4-数据流与线程模型)
5. [控制流：从遥控到电机](#5-控制流从遥控到电机)
6. [架构决策记录](#6-架构决策记录)
7. [逻辑审查发现汇总](#7-逻辑审查发现汇总)
8. [架构改进机会](#8-架构改进机会)

---

## 1. 系统俯瞰

```
┌──────────────────────────────────────────────────────────────────┐
│                    舵轮底盘控制系统 (Steering Wheel Chassis)        │
│                                                                  │
│  硬件平台：STM32F405RGTx  @168MHz (HSI→PLL)                       │
│  RTOS：    FreeRTOS (CMSIS-OS v2 封装)                            │
│  控制周期： 250Hz (4ms) 运动控制 | 100Hz (10ms) 遥测 | vTaskDelayUntil 精确定时 │
│  通信：    CAN1 (8×M3508), 软I2C×4 (4×MT6701),                    │
│            USART2 (DBUS遥控), UART4 (Vofa调试)                   │
└──────────────────────────────────────────────────────────────────┘
```

本系统是一个 **四轮独立舵向 + 四轮独立驱动**（4WS4WD）的移动底盘控制器。每个轮组由两个 M3508 电机驱动：一个控制舵向转角，一个控制驱动轮转速。舵向角度通过 MT6701 14位磁编码器反馈，形成串级 PID 闭环控制。

底盘速度指令来自 DJI DR16 遥控器（DBUS 协议），通过逆运动学解算出各轮的独立目标舵角和轮速，经串级 PID 计算电机电流指令，通过 CAN 总线发送给 8 路 M3508 电机。

---

## 2. 领域术语表

| 术语 | 英文 | 定义 | 涉及模块 |
|------|------|------|---------|
| **舵轮** | Steering Wheel | 一个集成了舵向电机 + 驱动电机的完整轮组模块。底盘有 4 个舵轮 | `SteeringWheel` 结构体 |
| **舵向电机** | Steer Motor | 控制车轮旋转方向（转角）的 M3508 电机，ID 5-8 | `CAN_ID_STEER_1~4` |
| **驱动电机** | Drive Motor | 控制车轮前进/后退转速的 M3508 电机，ID 1-4 | `CAN_ID_DRIVE_1~4` |
| **回正** | Homing | 上电后将 4 个舵向轮旋转到预设的物理零位（直行方向） | `CHASSIS_HOMING` 状态 |
| **零位偏移** | Steer Offset | 每个舵轮物理安装角度与编码器读数之间的固定偏差 | `STEER_OFFSET_1~4` |
| **逆运动学** | Inverse Kinematics | 从底盘速度指令 (Vx, Vy, Wz) 解算 4 轮目标舵角 + 目标轮速 | `InverseKinematics()` |
| **最短路径优化** | Shortest Path Optimisation | 当目标舵角与当前舵角差 >90° 时，反转驱动轮方向，目标舵角转180° | `InverseKinematics()` 内 |
| **电流 Ramp** | Current Ramp | 限制每控制周期电流变化量，防止电流突变损坏硬件 | `ApplyCurrentRamp()` |
| **急停** | E-Stop (Emergency Stop) | 所有电机电流置零，进入安全状态 | `CHASSIS_ESTOP` 状态 |
| **失联保护** | Loss Protection | 遥控器信号丢失后的三级响应：在线 → 急停 → 关断 | `LossState` 状态机 |
| **DBUS** | DBUS | DJI 遥控接收机串行协议，18字节帧，500kHz UART | `remote_control.c` |
| **Vofa** | Vofa | 上位机调试可视化工具，通过 UART4 发送 JustFloat 协议帧 | `vofa_send.c` |
| **JustFloat** | JustFloat | Vofa 的浮点数据流协议，直接发送 float 字节流 | `Vofa_SendWheelDebug()` |
| **死区** | Deadband | 摇杆/舵角误差小于此值视为零，防止微动和抖动 | `STEER_DEADBAND_DEG`, `REMOTE_DEADBAND` |
| **舵向速度环** | Steer Velocity Loop | 串级PID内环，将位置环输出的目标转速转化为电机电流 | `g_steer_vel_pid` |
| **舵向位置环** | Steer Position Loop | 串级PID外环，将目标角度转化为目标转速 | `g_steer_pos_pid` |
| **驱动速度环** | Drive Velocity Loop | 单级PID，将目标轮速转化为电机电流 | `g_drive_vel_pid` |

### 坐标系统（俯视）

```
        Y↑ (左)
        |
   轮3  |  轮1
   ---- + ---- → X (前)
   轮4  |  轮2
        |
```

| 轮号 | 位置 (x, y) | 驱动电机 CAN ID | 舵向电机 CAN ID | 编码器 I2C | 回正原始值 | 零位偏移 |
|------|------------|:--------------:|:--------------:|:---------:|:--------:|:--------:|
| 轮1 | (+a, +b)   | 0x201 | 0x205 | I2C2 | 13600 | 175.8° |
| 轮2 | (+a, -b)   | 0x202 | 0x206 | I2C1 | 11160 | 126.6° |
| 轮3 | (-a, +b)   | 0x203 | 0x207 | I2C3 | 15568 | 213.0° |
| 轮4 | (-a, -b)   | 0x204 | 0x208 | I2C4 | 3188  | 313.0° |

---

## 3. 模块地图

### 3.1 模块分层

```
┌──────────────────────────────────────────────────────────────────┐
│                         任务层 (Tasks)                            │
│  ┌─────────────────┐ ┌──────────────────┐ ┌───────────────────┐  │
│  │ remote_control_  │ │  movement_task.c │ │ vofa_transmit_    │  │
│  │ task.c           │ │  (运动控制任务)   │ │ task.c            │  │
│  │ (遥控任务)        │ │  500Hz 主循环     │ │ (遥测任务)        │  │
│  │ 等待信号量→映射   │ │  队列消费→控制循环 │ │  100Hz 发送调试帧  │  │
│  │ 摇杆→速度→队列    │ │                  │ │                  │  │
│  └────────┬────────┘ └────────┬─────────┘ └────────┬──────────┘  │
│           │                   │                    │              │
├───────────┼───────────────────┼────────────────────┼──────────────┤
│           │          控制层 (Steering Chassis)      │              │
│           │    ┌──────────────────────────────┐     │              │
│           └───→│  steering_chassis.c          │←────┘              │
│                │  - 逆运动学解算              │                    │
│                │  - 串级PID控制 (舵向+驱动)    │                    │
│                │  - 上电回正状态机            │                    │
│                │  - 电流Ramp限幅              │                    │
│                │  - 急停                      │                    │
│                └──────┬───────────┬───────────┘                    │
│                       │           │                                │
├───────────────────────┼───────────┼────────────────────────────────┤
│              硬件驱动层 (Hardwares)  │                                │
│  ┌──────────┐ ┌────────┐ ┌───────┐│┌──────────┐ ┌──────────┐     │
│  │remote_   │ │M3508   │ │MT6701 │││soft_i2c  │ │vofa_send │     │
│  │control.c │ │.c      │ │.c     │││.c        │ │.c        │     │
│  │DBUS解析  │ │CAN电流 │ │磁编码  │││4路GPIO   │ │JustFloat │     │
│  │失联保护  │ │反馈读取 │ │器读取  │││模拟I2C   │ │协议帧    │     │
│  └──────────┘ └────────┘ └───────┘│└──────────┘ └──────────┘     │
│                                   │                                │
├───────────────────────────────────┼────────────────────────────────┤
│                          配置层   │                                 │
│                    ┌──────────────┴──────┐                         │
│                    │  config.h            │                         │
│                    │  - 几何参数           │                         │
│                    │  - 减速比             │                         │
│                    │  - CAN ID             │                         │
│                    │  - PID系数            │                         │
│                    │  - 回正目标           │                         │
│                    │  - GPIO定义           │                         │
│                    └─────────────────────┘                         │
└──────────────────────────────────────────────────────────────────┘
```

### 3.2 调用关系图

```
main.c
└── MX_FREERTOS_Init()
    ├── Movement_Init()                    [Tasks/Movement/movement_task.c]
    │   ├── MT6701_Init() ×4              [Hardwares/MT6701/mt6701.c]
    │   ├── M3508_Init()                   [Hardwares/M3508/m3508.c]
    │   └── SteeringChassis_Init()         [Tasks/Movement/steering_chassis.c]
    │       ├── PID_Init() ×4              (内部)
    │       └── 回正目标计算               (内部)
    ├── RemoteControlTask_Init()           [Tasks/Remote_Control/remote_control_task.c]
    │   └── RemoteControl_Init()           [Hardwares/Dbus_Remote_Control/remote_control.c]
    │       └── HAL_UARTEx_ReceiveToIdle_DMA()
    ├── osSemaphoreNew(DbusReady)
    ├── osMessageQueueNew(Signal_Queue, 16, ChassisVelocityCmd)
    ├── osThreadNew(StartDefaultTask)      → osDelay(1) 空转
    ├── osThreadNew(Receive_and_Process_Signal)  → 遥控任务
    ├── osThreadNew(Motor_Movement)              → 运动任务
    └── osThreadNew(Vofa_Debugging_Transmition)  → 遥测任务

中断回调 (ISR)：
├── HAL_CAN_RxFifo0MsgPendingCallback()    → M3508_RxCallback()
└── HAL_UARTEx_RxEventCallback(USART2)     → RemoteControl_IdleCallback()
                                              → ParseDbusFrame()
                                              → xSemaphoreGiveFromISR(DbusReady)
```

---

## 4. 数据流与线程模型

### 4.1 线程拓扑

```
任务名              优先级    栈大小    周期      职责
──────────────────────────────────────────────────────────────
defaultTask         Normal   128×4    (1ms空转)  保留
Receive_Handle      Low      256×4    信号量驱动  遥控解析→速度映射→队列
Movement_Handle     Low      512×4    4ms固定 (vTaskDelayUntil)   队列消费→运动学→PID→CAN发送
Vofa_Transmit       Low      128×4    10ms固定   读取调试数据→UART4发送
```

**关键观察**：遥控任务、运动任务、遥测任务均为 `osPriorityLow` 同级，由 FreeRTOS 时间片轮转调度。运动任务使用 `vTaskDelayUntil` 精确定时 4ms (250Hz)，与 `CONTROL_DT` 保持严格一致。遥控任务由信号量驱动，遥测任务为周期性 10ms。

### 4.2 数据流图

```
                          [遥控器]
                             │
                        2.4GHz无线
                             │
                        [DR16 接收机]
                             │
                        DBUS UART (500kbps)
                             │
                     USART2 DMA CIRCULAR
                             │
                  ┌──────────┴──────────┐
                  │  ISR: UART IDLE      │
                  │  ↓                   │
                  │  ParseDbusFrame()    │
                  │  ↓                   │
                  │  cached_state 写入   │ ← 可能存在竞态 (ISR vs Task)
                  │  ↓                   │
                  │  xSemaphoreGive      │
                  └──────────┬──────────┘
                             │
                  ┌──────────┴──────────┐
                  │  Task: Receive_Handle │
                  │  ↓                   │
                  │  osSemaphoreAcquire   │
                  │  ↓                   │
                  │  RemoteControl_       │
                  │  GetState()           │
                  │  ↓                   │
                  │  摇杆映射:            │
                  │  vx = ly × MAX_XY     │
                  │  vy = lx × MAX_XY     │
                  │  wz = ry × MAX_WZ     │
                  │  ↓                   │
                  │  osMessageQueuePut ───┼──→ Signal_Queue (FIFO, 16元素)
                  └──────────────────────┘          │
                                                    │
                  ┌─────────────────────────────────┘
                  │
                  ┌──────────────────────────┐
                  │  Task: Movement_Handle    │
                  │  ↓                       │
                  │  while(osMessageQueue-    │
                  │        Get())  排空队列   │ → 取最新指令
                  │  ↓                       │
                  │  SteeringChassis_         │
                  │  SetVelocity(vx,vy,wz)    │ → 写 g_vx, g_vy, g_wz
                  │  ↓                       │
                  │  SteeringChassis_         │
                  │  ControlLoop()            │
                  │  ├── ReadEncoders()      │ → MT6701 ReadAngle ×4
                  │  ├── ReadMotorFeedback() │ → M3508 GetFeedback ×8
                  │  ├── InverseKinematics() │ → 计算 target_steer_angle, target_drive_speed
                  │  ├── ComputeSteerPID()   │ → 串级PID (位置→速度→电流)
                  │  ├── ComputeDrivePID()   │ → 速度PID→电流
                  │  ├── ApplyCurrentRamp()  │ → 电流限幅
                  │  └── SendMotorCurrents() │ → M3508 SetCurrent ×8 → CAN
                  │                          │
                  │  vTaskDelayUntil         │
                  │  (CONTROL_PERIOD_MS=4ms) │
                  └──────────────────────────┘
                             │
                        [CAN1 总线]
                             │
                    ┌────────┴────────┐
                    │ 8路 M3508 电机   │
                    │ (4舵向+4驱动)    │
                    └─────────────────┘

          ┌──────────────────────────┐
          │  Task: Vofa_Transmit     │
          │  ↓                       │
          │  SteeringChassis_        │
          │  GetWheelDebug()         │ → 读取 16 个 float
          │  ↓                       │
          │  Vofa_SendWheelDebug()   │ → UART4 JustFloat
          │  osDelay(10)             │
          └──────────────────────────┘
                    │
               [UART4 → PC Vofa上位机]
```

---

## 5. 控制流：从遥控到电机

### 5.1 完整路径 (时序)

```
T=0ms    遥控器发射 DBUS 帧 (约每 7-14ms 一帧，取决于 DR16 模式)
T=1ms    USART2 DMA 接收完成，UART IDLE 中断触发
          ├── HAL_UARTEx_RxEventCallback → RemoteControl_IdleCallback
          ├── 扫描循环缓冲区，找帧头 0xA0
          ├── ParseDbusFrame: 解析 6 通道 + 2 开关 → 归一化
          └── xSemaphoreGiveFromISR(DbusReady) → 唤醒 Receive_Handle

T=1ms+   Receive_Handle 任务就绪，获取信号量
          ├── RemoteControl_GetState → 读取 cached_state
          ├── RemoteControl_LossProtectionTick → 检查失联状态
          ├── 摇杆映射: vx/vy/wz → ChassisVelocityCmd
          └── osMessageQueuePut(Signal_Queue, &cmd)

T=4ms    Movement_Handle 定时唤醒 (vTaskDelayUntil, CONTROL_PERIOD_MS=4ms)
          ├── osMessageQueueGet 排空队列 → SteeringChassis_SetVelocity
          └── SteeringChassis_ControlLoop:
              ├── [Step 1] ReadEncoders: 软I2C 依次读 4 路 MT6701
              │     角度原始值 → 应用零位偏移 → 归一化 [0,360)
              │     失败计数累加，超阈值 → 急停
              ├── [Step 2] ReadMotorFeedback: 读取 M3508 CAN 反馈缓存
              │     drive_rpm, steer_rpm, 电流反馈
              ├── [Step 3] InverseKinematics:
              │     每轮: V_ix=Vx-Wz×y_i, V_iy=Vy+Wz×x_i
              │     目标舵角 = atan2(V_iy, V_ix)
              │     目标轮速 = |V|/(2πR)×60 → rpm
              │     + 最短路径优化 (90°阈值)
              ├── [Step 4] ComputeSteerPID ×4:
              │     位置误差 = Wrap180(目标舵角 - 当前角度)
              │     位置环PID → 目标转速 (dps)
              │     速度环PID → 电流指令 (mA)
              ├── [Step 5] ComputeDrivePID ×4:
              │     速度误差 = 目标转速 - 当前转速
              │     速度环PID → 电流指令 (mA)
              ├── [Step 6] ApplyCurrentRamp ×8:
              │     限制电流变化 ≤ CURRENT_RAMP_RATE
              └── [Step 7] SendMotorCurrents:
                     M3508_SetCurrent ×8 → M3508_SendAll → CAN1 发送

T=4ms+   M3508 电机收到 CAN 指令，执行电流闭环
         MT6701 编码器持续反馈角度变化
```

### 5.2 状态机

```
                    ┌─────────────┐
                    │  上电初始化   │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
          ┌────────│   CHASSIS_   │──────────┐
          │        │   HOMING     │          │
          │        │  (舵向回正)   │          │
          │        └──────┬──────┘          │
          │               │                 │
          │      全部4轮到  │        编码器故障│
          │      位误差<3°  │        连续失败  │
          │               │        或遥控失联  │
          │        ┌──────▼──────┐          │
          │        │   CHASSIS_  │◄─────────┘
          │        │   NORMAL    │
          │        │  (正常控制)  │
          │        └──────┬──────┘
          │               │
          │      遥控失联/ │
          │      急停指令   │
          │               │
          │        ┌──────▼──────┐
          └───────►│   CHASSIS_  │
                   │   ESTOP     │
                   │  (急停)      │
                   └─────────────┘
                   (无自动恢复路径)
```

**遥控失联保护子状态机**（独立于底盘状态机）：

```
    LOSS_IDLE ──失联──► LOSS_ESTOP ──超时200ms──► LOSS_CUTOFF
        ▲                  │                          │
        └──收到新帧─────────┘                          │
        (恢复)                                        │
                                                      │
        (电流关断，无恢复路径 ── 需人工重启)
```

---

## 6. 架构决策记录

### ADR-001: 软 I2C 而非硬件 I2C
- **决策**: 使用 4 路 GPIO 模拟 I2C（`soft_i2c.c`），而非 STM32 硬件 I2C 外设
- **原因**: MT6701 编码器可能对 I2C 时序有特殊要求；STMF4 硬件 I2C 有已知的 silicon bug
- **代价**: 占用更多 CPU 时间（GPIO 翻转），I2C 读取阻塞控制循环

### ADR-002: 串级 PID 而非单级 PID
- **决策**: 舵向控制使用位置外环 + 速度内环的串级结构
- **原因**: 提升舵向响应速度和抗干扰能力
- **隐含假设**: 两个 PID 环共享同一控制器实例 (`g_steer_pos_pid`, `g_steer_vel_pid`)

### ADR-003: 回正后切换 PID
- **决策**: 回正阶段使用较温和的 PID 参数（Kp/Ki 减半）
- **原因**: 防止回正到位时过冲振荡
- **问题**: 切换时 `prev_error` 未清空，可能产生 D 项跳变

### ADR-004: 电流 Ramp 限幅
- **决策**: 每控制周期限制电流变化不超过 4000 mA
- **原因**: 保护电机驱动器和机械结构
- **代价**: 可能在快速制动时引入响应延迟

### ADR-005: 编码器故障→急停
- **决策**: 任一编码器连续失败 10 次 → 直接进入不可恢复的急停
- **原因**: 安全优先，无角度反馈时无法正确控制
- **代价**: 瞬时干扰也可能导致永久停车

### ADR-006: vTaskDelayUntil 精确定时 (H2 修复)
- **日期**: 2026-06-22
- **决策**: 使用 `vTaskDelayUntil` 替代 `osDelay(2)`，周期由 `CONTROL_PERIOD_MS` (4ms) 统一定义
- **原因**: 历史使用 `osDelay(2)` 导致控制频率不准确（5ms vs 目标4ms），且与 `CONTROL_DT` (0.004s) 不一致
- **影响**: PID 计算的时间增量与物理周期现在严格一致

### ADR-007: 每轮独立 PID 实例 (H1 修复)
- **日期**: 2026-06-22
- **决策**: 舵向位置/速度环、驱动速度环、回正位置环均使用 `[WHEEL_COUNT]` 数组，每轮独立 PID 实例
- **原因**: 原代码 4 轮共享单一 PID 实例，导致积分状态交叉污染——轮1的积分影响轮2，轮2影响轮3
- **影响**: 各轮控制独立、可预测

### ADR-008: 死区参数化 (H3 修复)
- **日期**: 2026-06-22
- **决策**: `PID_Init` 接受 `deadband` 参数，启用 `STEER_DEADBAND_DEG` (1.0°) 死区
- **原因**: 原代码死区参数 `0.0f`，导致舵向误差微小时持续微调，引发振荡和电机发热
- **影响**: 舵向到位后停止微调

---

## 7. 逻辑审查发现汇总

> 完整诊断过程见 diagnose 技能输出。以下仅列出与架构相关的发现。

### 🔴 P0 — 架构级缺陷

| # | 问题 | 根因 | 影响 |
|---|------|------|------|
| 1 | **PID 控制器被 4 轮共享导致积分串扰** | `g_steer_pos_pid`、`g_steer_vel_pid`、`g_drive_vel_pid` 是全局单一实例，4 轮循环计算时积分状态未隔离 | 轮1的积分影响轮2，轮2影响轮3...控制行为不可预测，各轮耦合 |
| 2 | **ISR 与任务间 cached_state 存在竞态** | `RemoteState` 包含 8 个字段，`memcpy` 读取时 ISR 可能在写入 | 偶发性读到半新半旧的速度指令 |

### 🟡 P1 — 安全/鲁棒性缺陷

| # | 问题 | 根因 | 影响 |
|---|------|------|------|
| 3 | **失联保护状态机可被绕过** | `rc_state.connected == 0` 时跳过 `LossProtectionTick()` | 单帧解析失败→失联保护失效，不发送急停 |
| 4 | **急停无恢复路径** | 编码器故障或失联进入 `CHASSIS_ESTOP` 后无退出条件 | 需人工重启 |
| 5 | **最短路径优化缺少滞回** | `fabsf(steer_error) > 90.0f` 无滞回区间 | 90° 边界噪声导致驱动方向震荡 |
| 6 | **队列满时静默丢帧** | `osMessageQueuePut(..., 0U)` 满时直接丢弃 | 停止指令可能丢失 |

### 🟢 P2 — 代码质量

| # | 问题 | 根因 | 影响 |
|---|------|------|------|
| 7 | 回正→正常切换时 `prev_error` 未清空 | PID_Compute 中 prev_error 残留 | 第一个控制周期 D 项跳变 |
| 8 | `IsHomingDone()` 语义不精确 | `!= HOMING` 包含了 `ESTOP` | API 调用者误判 |
| 9 | 减速比不一致 | 轮1/4 = 14:1, 轮2/3 = 19:1 | 运动学解算中未考虑减速比差异 |
| 10 | 编码器读取失败时舵向保持 | `encoder_valid=0` 时跳过 PID 计算 | 舵机可能在错误位置锁定 |

---

## 8. 架构改进机会

### 8.1 短期修复（低风险，高收益）

1. **PID 实例隔离**: 将 `g_steer_pos_pid` 等改为 `PID_Controller g_steer_pos_pid[4]`，每个轮子独立 PID。或者每次使用前调用 `PID_Reset()` 重置积分和 `prev_error`。

2. **cached_state 竞态修复**: 使用双缓冲（double buffering）或临界区保护 `cached_state` 的读写。考虑到性能，推荐使用 `__disable_irq()/__enable_irq()` 短暂关闭中断包裹 `memcpy`。

3. **最短路径滞回**: 将 `> 90.0f` 改为 `> 95.0f`（进入反转），`< 85.0f`（退出反转）引入滞回。

### 8.2 中期改进（增强鲁棒性）

4. **急停恢复机制**: 在 `CHASSIS_ESTOP` 状态下增加编码器健康检查，若连续 N 次编码器读取成功且遥控信号恢复，可尝试回正后重新进入 `CHASSIS_NORMAL`。

5. **失联保护不依赖 connected 标志**: 将 `LossProtectionTick()` 的调用移到 `if(rc_state.connected)` 之外，使其无条件执行。

6. **队列满告警**: `osMessageQueuePut` 返回值检查，若满则记录日志或触发 LED 告警。

### 8.3 长期架构升级

7. **运动学模型补全**: 当前逆运动学解算未考虑舵向角速度限制和驱动轮减速比差异。建议引入每轮独立的减速比系数。

8. **编码器冗余**: 考虑使用 M3508 内置编码器作为 MT6701 的备份，实现编码器冗余。

9. **PID 自整定**: 引入在线 PID 自动整定，减少手动调参工作量。

10. **控制频率提升**: 当前 500Hz 对于舵轮底盘是合适的，但如果通信负载允许，可考虑将 CAN 发送和控制解耦，控制计算 500Hz、CAN 发送 250Hz。

---

## 附录 A: 文件索引

| 文件路径 | 行数 | 职责 |
|----------|:--:|------|
| `Hardwares/config.h` | 178 | 全局参数配置（单点配置源） |
| `Core/Src/main.c` | 223 | 硬件初始化 + FreeRTOS 启动 |
| `Core/Src/freertos.c` | 248 | 任务/队列/信号量创建 + 类型定义 |
| `Tasks/Movement/steering_chassis.c` | 718 | 核心控制：运动学 + PID + 电流控制 |
| `Tasks/Movement/steering_chassis.h` | 158 | 底盘控制公开接口 |
| `Tasks/Movement/movement_task.c` | 82 | 运动任务入口 + CAN 中断回调 |
| `Tasks/Remote_Control/remote_control_task.c` | 103 | 遥控任务入口 + UART 中断回调 |
| `Hardwares/Dbus_Remote_Control/remote_control.c` | 277 | DBUS 协议驱动 + 失联保护 |
| `Tasks/Vofa_Transmit/vofa_transmit_task.c` | 45 | 遥测任务 |
| `Hardwares/M3508/m3508.c` | — | M3508 CAN 驱动 |
| `Hardwares/MT6701/mt6701.c` | — | MT6701 磁编码器驱动 |
| `Hardwares/I2c_Soft/soft_i2c.c` | — | GPIO 模拟 I2C 驱动 |
| `Hardwares/Vofa_Send/vofa_send.c` | — | Vofa JustFloat 协议驱动 |

---

*文档生成时间：2026-06-22*
*基于：完整的代码静态审查 + diagnose 技能诊断 + zoom-out 技能分析*