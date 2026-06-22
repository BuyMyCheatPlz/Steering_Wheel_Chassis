# Steering Wheel Chassis — 舵轮底盘控制系统

基于 STM32F405 + FreeRTOS 的舵轮底盘电机驱动，支持 DJI DT7/DR16 遥控器控制、M3508 无刷电机 CAN 控制、MT6701 角度编码器反馈、Vofa 实时调试数据上传。

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     FreeRTOS Task Layer                      │
├──────────────────┬──────────────────┬───────────────────────┤
│  Receive_Handle  │  Movement_Handle │  Vofa_Transmit        │
│  遥控信号处理任务  │  运动控制任务     │  调试数据传输任务     │
├──────────────────┼──────────────────┼───────────────────────┤
│  remote_control  │  steering_chassis│  vofa_send            │
│  遥控器解析驱动   │  舵轮底盘控制核心  │  Vofa 协议封装        │
├──────────────────┴──────────────────┴───────────────────────┤
│                      Hardware Layer                          │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│ M3508    │ MT6701   │ DBUS     │ Soft I2C │ Vofa Send       │
│ CAN 电机  │ 角度编码器│ 遥控接收  │ 模拟 I2C  │ 串口调试发送     │
├──────────┴──────────┴──────────┴──────────┴─────────────────┤
│                   STM32 HAL (CAN / UART / GPIO)              │
└─────────────────────────────────────────────────────────────┘
```

### 1.1 数据流

```
遥控器 (DBUS)
    │  UART2 DMA 循环接收
    ▼
Receive_and_Process_Signal (任务)
    │  解析 DBUS → 归一化 → 速度映射
    │  写入 Signal_Queue
    ▼
Motor_Movement (任务)
    │  从 Signal_Queue 读取 vx, vy, wz
    ▼
SteeringChassis_ControlLoop()
    │  ① 读取 4 路 MT6701 编码器 (4 组软 I2C 并发)
    │  ② 获取 8 路 M3508 电机反馈 (CAN 2ms 轮询)
    │  ③ 逆运动学解算 → 4 轮目标舵角 + 驱动转速
    │  ④ 串级 PID (舵向位置/速度环 + 驱动速度环)
    │  ⑤ 电流 ramp 限幅
    │  ⑥ CAN 发送 8 路电流指令
    ▼
M3508 电机执行
```

### 1.2 文件树

```
Steering_Wheel_Chassis/
├── Core/
│   ├── Inc/                          # STM32 外设头文件 (CubeMX)
│   └── Src/
│       └── freertos.c                # FreeRTOS 任务创建 + 队列/信号量定义
├── Hardwares/
│   ├── config.h                      # 全局可调参数 (PID/几何/ID/引脚)
│   ├── M3508/
│   │   ├── m3508.h                   # M3508 电机驱动接口
│   │   └── m3508.c                   # CAN 电流控制 + 反馈解析
│   ├── MT6701/
│   │   ├── mt6701.h                  # MT6701 编码器驱动接口
│   │   └── mt6701.c                  # 软 I2C 角度读取
│   ├── Dbus_Remote_Control/
│   │   ├── remote_control.h          # DJI DBUS 接收驱动接口
│   │   └── remote_control.c          # DMA 循环帧解析 + 失联保护
│   ├── I2c_Soft/
│   │   ├── soft_i2c.h                # 软件 I2C 接口 (GPIO 模拟)
│   │   └── soft_i2c.c                # 4 组独立 I2C 总线
│   └── Vofa_Send/
│       ├── vofa_send.h               # Vofa JustFloat 协议接口
│       └── vofa_send.c               # UART 发送封装
├── Tasks/
│   ├── Movement/
│   │   ├── movement_task.h           # 运动任务接口
│   │   ├── movement_task.c           # 250Hz 控制循环任务 (vTaskDelayUntil 精确定时)
│   │   ├── steering_chassis.h        # 舵轮底盘核心接口
│   │   └── steering_chassis.c        # 运动学 + 串级 PID + 编码器管理
│   ├── Remote_Control/
│   │   ├── remote_control_task.h     # 遥控任务接口
│   │   └── remote_control_task.c     # 信号解析 + 速度映射任务
│   └── Vofa_Transmit/
│       ├── vofa_transmit_task.h      # Vofa 任务接口
│       └── vofa_transmit_task.c      # 100Hz 调试数据任务
└── Drivers/                          # STM32 HAL + CMSIS (CubeMX)
```

---

## 2. FreeRTOS 任务 & 通信

### 2.1 任务列表

| 任务名 | 函数入口 | 优先级 | 频率 | 栈大小 | 职责 |
|--------|---------|--------|------|--------|------|
| `defaultTask` | `StartDefaultTask` | Normal | idle | 128×4 | 占位 (自由扩展) |
| `Receive_Handle` | `Receive_and_Process_Signal` | Low | 事件驱动 | 256×4 | 遥控解析 → 速度写入队列 |
| `Movement_Handle` | `Motor_Movement` | Low | 250Hz | 512×4 | 控制循环 → 舵轮底盘 (vTaskDelayUntil 精确定时) |
| `Vofa_Transmit` | `Vofa_Debugging_Transmition` | Low | 100Hz | 128×4 | 调试数据发送 |

### 2.2 IPC (Inter-Process Communication)

| 类型 | 名称 | 方向 | 用途 |
|------|------|------|------|
| Queue | `Signal_Queue` | 遥控任务 → 运动任务 | 传递 `ChassisVelocityCmd { vx, vy, wz }` |
| Semaphore | `DbusReady` | ISR → 遥控任务 | DBUS 帧接收完毕信号 |

### 2.3 ISR 回调 → Task 转发

| ISR 回调 | 转发到 | 功能 |
|----------|--------|------|
| `HAL_CAN_RxFifo0MsgPendingCallback` | `Movement_CANCallback` → `M3508_RxCallback` | 解析 CAN反馈帧 |
| `HAL_UARTEx_RxEventCallback` (USART2) | `RemoteControl_UARTCallback` → `RemoteControl_IdleCallback` | 解析 DBUS帧、释放信号量 |

---

## 3. 核心函数及位置

### 3.1 M3508 电机驱动 (`Hardwares/M3508/m3508.c`)

| 函数 | 位置 (行号) | 功能 |
|------|------------|------|
| `M3508_Init()` | m3508.c:51 | 初始化 CAN + 反馈数组清零 |
| `M3508_SetCurrent(id, current)` | m3508.c:82 | 缓存指定电机电流指令 (±16384mA) |
| `M3508_SendAll()` | m3508.c:110 | 批量发送 8 路 CAN 电流指令 (4 帧, 每帧 2 电机) |
| `M3508_GetFeedback(id, *fb)` | m3508.c:165 | 获取电机最新反馈 (累计角度/转速/电流) |
| `M3508_RxCallback(hcan)` | m3508.c:195 | CAN RX 中断解析反馈帧 |

### 3.2 MT6701 编码器驱动 (`Hardwares/MT6701/mt6701.c`)

| 函数 | 位置 (行号) | 功能 |
|------|------------|------|
| `MT6701_Init(*dev, *bus)` | mt6701.c:37 | 绑定 I2C 总线、清零设备 |
| `MT6701_ReadAngle(*dev)` | mt6701.c:50 | 软 I2C 读取 14 位角度 → 转换为 0-360° |
| `MT6701_GetAngleDegrees(*dev)` | mt6701.c:78 | 返回上次读取的角度值 (度) |
| `MT6701_GetRawAngle(*dev)` | mt6701.c:83 | 返回上次读取的原始值 (0-16383) |

### 3.3 DBUS 遥控接收驱动 (`Hardwares/Dbus_Remote_Control/remote_control.c`)

| 函数 | 位置 (行号) | 功能 |
|------|------------|------|
| `RemoteControl_Init(*huart)` | remote_control.c:45 | 使能 UART IDLE + DMA 循环接收 |
| `RemoteControl_IdleCallback(*huart)` | remote_control.c:75 | 帧头 `0xA0` 验证 → 18 字节解析 → 释放信号量 |
| `RemoteControl_GetState(*state)` | remote_control.c:125 | 获取归一化 [-1.0, +1.0] 摇杆/开关状态 |
| `RemoteControl_IsConnected()` | remote_control.c:159 | 根据时间戳判断是否在线 |
| `RemoteControl_LossProtectionTick()` | remote_control.c:170 | 失联保护状态机 (normal→esrop→hold) |

### 3.4 软件 I2C (`Hardwares/I2c_Soft/soft_i2c.c`)

| 函数 | 位置 (行号) | 功能 |
|------|------------|------|
| `SoftI2C_Init(*bus, ...)` | soft_i2c.c:40 | 绑定 GPIO 端口/引脚/时序参数 |
| `SoftI2C_Read(*bus, addr, reg, *data, len)` | soft_i2c.c:120 | 从设备寄存器连续读取 |

### 3.5 Vofa 发送 (`Hardwares/Vofa_Send/vofa_send.c`)

| 函数 | 位置 (行号) | 功能 |
|------|------------|------|
| `Vofa_SendJustFloat(*huart, *data, count)` | vofa_send.c:25 | 通用 JustFloat 格式发送 (≤16 floats) |
| `Vofa_SendWheelDebug(*huart, ...)` | vofa_send.c:52 | 16 通道舵轮数据封装发送 |

### 3.6 舵轮底盘控制核心 (`Tasks/Movement/steering_chassis.c`)

| 函数 | 位置 (行号) | 可见性 | 功能 |
|------|------------|--------|------|
| `SteeringChassis_Init()` | steering_chassis.c:105 | Public | 初始化 I2C 总线 + M3508 + PID 参数 + 编码器 |
| `SteeringChassis_ControlLoop()` | steering_chassis.c:215 | Public | 主控制循环 (编码器→反馈→解算→PID→发送) |
| `SteeringChassis_SetVelocity()` | steering_chassis.c:145 | Public | 设置底盘速度指令 (入队) |
| `SteeringChassis_EmergencyStop()` | steering_chassis.c:160 | Public | 急停 (电流清零 + 状态切换) |
| `SteeringChassis_GetState()` | steering_chassis.c:185 | Public | 返回 ChassisState |
| `SteeringChassis_IsHomingDone()` | steering_chassis.c:200 | Public | 回正完成查询 |
| `SteeringChassis_IsStopped()` | steering_chassis.c:175 | Public | 急停状态查询 |
| `SteeringChassis_GetWheelDebug()` | steering_chassis.c:? | Public | 读取 4 轮调试数据供 Vofa |
| `PID_Init()` | steering_chassis.c:260 | Static | 初始化 PID 控制器参数 |
| `PID_Compute()` | steering_chassis.c:285 | Static | PID 增量计算 (±180° 包裹 + 死区) |
| `ReadEncoders()` | steering_chassis.c:320 | Static | 4 路 MT6701 并发角度读取 |
| `ReadMotorFeedback()` | steering_chassis.c:350 | Static | 获取 8 路 M3508 反馈 |
| `InverseKinematics()` | steering_chassis.c:405 | Static | 逆运动学解算 (Vx,Vy,Wz → 4 轮目标) |
| `ComputeSteerPID(idx)` | steering_chassis.c:455 | Static | 舵向串级 PID (位置环 + 速度环) |
| `ComputeDrivePID(idx)` | steering_chassis.c:505 | Static | 驱动速度环 PID |
| `ApplyCurrentRamp(idx)` | steering_chassis.c:535 | Static | 电流变化率限幅 (防冲击) |
| `SendMotorCurrents()` | steering_chassis.c:565 | Static | 批量写入 8 路电流 → CAN 发送 |
| `Wrap180(angle_diff)` | steering_chassis.c:595 | Static | 角度误差归一化到 ±180° |
| `NormalizeAngle(angle)` | steering_chassis.c:610 | Static | 角度归一化到 [0, 360) |
| `RawToDegrees(raw)` | steering_chassis.c:625 | Static | 14 位原始值 → 角度转换 |

### 3.7 Task 实现文件

**Tasks/Movement/movement_task.c**

| 函数 | 行号 | 功能 |
|------|------|------|
| `Movement_Init()` | ~30 | 初始化 I2C 总线 + SteeringChassis_Init |
| `Motor_Movement()` | ~55 | 250Hz 循环: vTaskDelayUntil 精确定时，队列取速度 → ControlLoop |
| `Movement_CANCallback()` | ~80 | CAN RX 中断转发 → M3508_RxCallback |

**Tasks/Remote_Control/remote_control_task.c**

| 函数 | 行号 | 功能 |
|------|------|------|
| `RemoteControlTask_Init()` | ~30 | 初始化 DBUS 驱动 |
| `Receive_and_Process_Signal()` | ~55 | 事件循环: 等信号量 → 解析 → 映射速度 → 写入队列 |
| `RemoteControl_UARTCallback()` | ~100 | UART IDLE 中断转发 → DBUS 解析 |

**Tasks/Vofa_Transmit/vofa_transmit_task.c**

| 函数 | 行号 | 功能 |
|------|------|------|
| `Vofa_Debugging_Transmition()` | ~30 | 100Hz 循环: 读取调试数据 → Vofa 发送 |

### 3.8 FreeRTOS 初始化 (`Core/Src/freertos.c`)

| 函数/段 | 行号 | 功能 |
|---------|------|------|
| `SignalQueue_PutVelocity()` | 111 | 内联: 速度指令写入队列 |
| `SignalQueue_GetVelocity()` | 122 | 内联: 速度指令读取队列 |
| `MX_FREERTOS_Init()` | 141 | 初始化硬件 + 创建信号量/队列/任务 |

---

## 4. 功能实现

### 4.1 逆运动学解算

**输入:** 底盘目标速度 `Vx (m/s), Vy (m/s), Wz (rad/s)`  
**输出:** 4 轮舵角 (度) + 4 轮驱动转速 (rpm)  
**解算方程:** 舵轮底盘阿克曼逆解 (各轮独立舵角 + 转速合成)  

**最短路径优化 (P1-5 修复):**  
- 使用施密特滞回控制反转：进入反转需 `|Δθ| > 95°`，退出反转需 `|Δθ| < 85°`  
- 10° 滞回区消除编码器噪声导致的频繁反转震荡  
- 新增 `inverted` 标志位记录驱动方向  

**P2-9 修复:** 每轮独立驱动减速比 `REDUCTION_DRIVE_1~4`，精确补偿机械传动差异  

**实现位置:** `Tasks/Movement/steering_chassis.c` → `InverseKinematics()`

### 4.2 串级 PID 控制

舵向电机采用**串级 PID** (位置环 + 速度环)，驱动电机采用**单速度环**:

```
目标舵角 ──→ [位置环 PID] ──→ 目标转速 ──→ [速度环 PID] ──→ 电流输出
              (STEER_POS_*)     (rpm)        (STEER_VEL_*)     (mA)

目标转速 ──→ [速度环 PID] ──→ 电流输出
              (DRIVE_VEL_*)     (mA)
```

**P0-1 修复 (关键):** PID 控制器改为**每轮独立实例** (3组 × 4轮 = 12个 PID 对象)。  
原代码 4 轮共享同一个 PID 导致积分累计互相污染，修复后各轮 PID 状态完全隔离。  

**H1 修复:** 回正 PID 同样改为每轮独立实例 (`g_homing_pos_pid[4]`)，消除回正阶段积分交叉污染。

**H2 修复:** 运动任务由 `osDelay(2)` 改为 `vTaskDelayUntil` + `CONTROL_PERIOD_MS`(4ms)，周期与 `CONTROL_DT`(0.004s) 完全一致，消除 FreeRTOS tick 抖动导致的累积漂移。

**H3 修复:** `PID_Init` 的 deadband 参数从 `0` 改为 `STEER_DEADBAND_DEG`(1°)，所有位置环 PID 正确启用死区。

**P2-7 新增:** `PID_Reset()` 函数，模式切换时清零积分 + 上次误差，防止 D 项跳变。  

**实现位置:** `Tasks/Movement/steering_chassis.c` → `ComputeSteerPID()` / `ComputeDrivePID()`

### 4.3 电流保护

- **Ramp 限幅:** 电流变化率限制 ±4000mA/周期 (`ApplyCurrentRamp`)  
- **急停:** 编码器连续故障 ≥ 10 次 → 所有通道置零  
- **范围限幅:** ±16384mA 硬件限幅 (M3508 电气上限)  
- **P2-10 修复:** 编码器读取失败时设电流为 0 (而非保持旧值)，避免舵角锁死在错误位置  

### 4.4 回正控制

上电后自动回正，控制流程:
1. 编码器读取原始值
2. 检查各轮当前角度与 `STEER_INIT_RAW_x` 的偏差
3. 若偏差 ≥ `HOMING_THRESHOLD_DEG`(3°)，保持低速旋转 (回正 PID 参数为正常参数 ×0.5)
4. 4 轮全部到位 → 调用 `PID_Reset` 清零所有 PID → 切换 `CHASSIS_NORMAL`

**P2-8 修复:** `IsHomingDone()` 仅在 `CHASSIS_NORMAL` 状态返回 true，急停时不误报为完成。  

**实现位置:** `Tasks/Movement/steering_chassis.c` → `SteeringChassis_ControlLoop()`

### 4.5 失联保护

遥控器失联时 (帧间隔 ≥ 50ms):
1. **0-200ms:** 急停 (所有电流清零) — `loss == 1`
2. **200ms+:** 保持急停，超时关断电流 — `loss == 2`  
3. 重新收到遥控信号 → `loss_state` 自动恢复为 `LOSS_IDLE`

**P1-3 修复 (关键):** `LossProtectionTick()` 从 `if (connected)` 块内移出到外层调用。  
原代码未连遥控器时永远不会调用失联保护，修复后状态机独立运行，上电即可防护。  

**P1-4 新增:** 急停恢复机制 (`CHASSIS_ESTOP_RECOVER` 状态)：
1. 编码器需连续 5 次 (约 500ms) 全部读数成功
2. 遥控器 SW1=UP 确认恢复
3. 重新执行回正 → 进入 `CHASSIS_NORMAL`

**P0-2 修复 (关键):** `RemoteControl_GetState()` 增加中断临界区保护，确保 18 字节 `cached_state` 在 ISR 并发写入时读到一致状态。  

**实现位置:** `Hardwares/Dbus_Remote_Control/remote_control.c` → `RemoteControl_LossProtectionTick()`  
`Tasks/Remote_Control/remote_control_task.c` → `Receive_and_Process_Signal()`

### 4.6 Vofa 调试数据

每 10ms 通过 USART4 发送 16 通道 JustFloat 格式数据:
- Channel 0-3: 4 轮舵向当前角度 (度)
- Channel 4-7: 4 轮舵向目标角度 (度)
- Channel 8-11: 4 轮驱动当前转速 (rpm)
- Channel 12-15: 4 轮驱动目标转速 (rpm)

**实现位置:** `Tasks/Vofa_Transmit/vofa_transmit_task.c` → `Vofa_Debugging_Transmition()`

---

## 5. 配置说明

所有可调参数集中在 `Hardwares/config.h`:

| 分类 | 宏 | 默认值 | 说明 |
|------|-----|--------|------|
| 底盘几何 | `WHEEL_BASE_A` | 0.15m | 前后半轴距 |
| | `TRACK_WIDTH_B` | 0.15m | 左右半轮距 |
| | `WHEEL_RADIUS` | 0.076m | 车轮半径 |
| 减速比 | `REDUCTION_DRIVE_x` | 14/19 | 各轮驱动减速比 |
| 舵向零位 | `STEER_INIT_RAW_x` | 13600/... | 标定后各轮回正原始值 |
| 舵向 PID | `STEER_POS_KP/KI/KD` | 8.0/0.2/0 | 位置环参数 |
| | `STEER_VEL_KP/KI/KD` | 10.0/0.5/0 | 速度环参数 |
| 驱动 PID | `DRIVE_VEL_KP/KI/KD` | 8.0/0.3/0 | 速度环参数 |
| 电流 | `CURRENT_RAMP_RATE` | 4000 | 电流 ramp 斜率 (mA/周期) |
| 安全 | `ENCODER_MAX_FAIL_COUNT` | 10 | 编码器故障阈值 |
| | `REMOTE_TIMEOUT_MS` | 50 | 遥控失联超时 |
| 运动 | `REMOTE_VEL_MAX_XY` | 3.0 m/s | 最大平动速度 |
| | `REMOTE_VEL_MAX_WZ` | 6.283 rad/s | 最大旋转速度 |
| 编码器 | `MT6701_RESOLUTION` | 16384 | MT6701 14 位分辨率 |

---

## 6. 编译与调试

### 编译
- IDE: Keil MDK-ARM (`.uvprojx`)
- MCU: STM32F405RGTx
- HAL: STM32Cube FW_F4
- RTOS: FreeRTOS CMSIS_V2

### 调试
- 使用 Vofa+ 上位机，串口选择 USART4，协议选择 **JustFloat**
- 数据通道: 16 通道按 4.6 节排列

### VSCode IntelliSense
已配置 `.vscode/c_cpp_properties.json`，支持 Ctrl+左键跳转定义。
编译器路径: `C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc.exe`

---

## 7. 硬件连接

| 外设 | 接口 | 引脚 |
|------|------|------|
| M3508 ×8 | CAN1 | PB8 (RX) / PB9 (TX) |
| MT6701 ×4 | 4 组软 I2C | PA10/PA9, PB11/PB10, PD2/PC12, PC7/PC6 |
| DBUS 接收机 | USART2 | PA3 (RX) / PA2 (TX) |
| Vofa 调试 | USART4 | PA1 (RX) / PA0 (TX) |

---

## 8. 特殊约定

- M3508 电机编号: **驱动电机 1-4**，**舵向电机 5-8**
- CAN ID 映射: `驱动n→0x20n`，`舵向n→0x20(n+4)`
- 编码器读取的是**舵向电机**的实际角度
- 舵角目标使用 `Wrap180()` 分段优化，优先选择最短旋转路径
- **控制周期**: 250Hz (4ms)，由 `CONTROL_PERIOD_MS` 驱动，与 `CONTROL_DT`(0.004s) 一致
- **PID 架构**: 4轮 × (舵向位置环 + 舵向速度环 + 驱动速度环 + 回正位置环) = 16 个独立 PID 实例
