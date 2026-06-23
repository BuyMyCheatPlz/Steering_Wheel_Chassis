# 舵轮底盘控制系统 — 诊断交接文档

> 生成日期: 2026-06-23 | 最后修改: 4轮诊断审查后

---

## 1. 系统概述

4轮独立舵轮底盘 (Steering Wheel Chassis)，基于 **STM32F407 @168MHz** + **FreeRTOS (CMSIS V2)**，3个核心任务 + CAN/UART中断。

| 组件 | 说明 |
|---|---|
| MCU | STM32F407, HCLK=168MHz, APB1=42MHz |
| 舵向电机 | 4× M3508 (CAN ID 5-8), 减速比 14:1 (待标定) |
| 驱动电机 | 4× M3508 (CAN ID 1-4), 减速比 14:1/19:1 |
| 编码器 | 4× MT6701 (14位磁性编码器, I2C addr 0x06) |
| 遥控器 | DJI DR16 DT7 (DBUS 18B帧, 100Hz, UART2 DMA) |
| 调试 | Vofa+ (JustFloat, UART4, 100Hz) |

---

## 2. 文件结构

```
Steering_Wheel_Chassis/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── can.h
│   │   ├── stm32f4xx_it.h
│   │   └── FreeRTOSConfig.h
│   └── Src/
│       ├── main.c                 # CubeMX生成, 启动入口
│       ├── freertos.c             # CubeMX生成, 任务/信号量/队列创建
│       ├── can.c                  # CAN1初始化 (1Mbps, PB8/9)
│       ├── stm32f4xx_it.c         # 中断向量表 (CAN_RX0, UART, DMA)
│       ├── gpio.c                 # GPIO初始化
│       └── usart.c                # UART2(DBUS) + UART4(Vofa)
├── Tasks/
│   ├── Movement/
│   │   ├── movement_task.c/.h     # 运动任务 + CAN中断回调
│   │   └── steering_chassis.c/.h  # 逆运动学 + PID + 状态机
│   ├── Remote_Control/
│   │   └── remote_control_task.c/.h # 遥控任务 + UART中断回调
│   └── Vofa_Transmit/
│       └── vofa_transmit_task.c/.h  # 调试传输任务
├── Hardwares/
│   ├── config.h                   # 全局可调参数 (PID/几何/通信)
│   ├── M3508/
│   │   └── m3508.c/.h             # M3508 CAN驱动
│   ├── MT6701/
│   │   └── mt6701.c/.h            # MT6701 I2C编码器驱动
│   ├── I2c_Soft/
│   │   └── soft_i2c.c/.h          # 软I2C (GPIO模拟, ~100kHz)
│   ├── Dbus_Remote_Control/
│   │   └── remote_control.c/.h    # DR16 DBUS解析 + 失联保护
│   └── Vofa_Send/
│       └── vofa_send.c/.h         # Vofa JustFloat协议
└── DIAGNOSIS_HANDOFF.md           # ← 本文件
```

---

## 3. 任务架构

| 任务 | 入口函数 | 栈 | 优先级 | 周期 | 阻塞方式 |
|---|---|---|---|---|---|
| Receive_Handle | [Receive_and_Process_Signal](Tasks/Remote_Control/remote_control_task.c#L55) | 256×4 | osPriorityLow | 事件驱动 (10ms超时) | 信号量 |
| Movement_Handle | [Motor_Movement](Tasks/Movement/movement_task.c#L47) | 512×4 | osPriorityLow | 250Hz (4ms) | vTaskDelayUntil |
| Vofa_Transmit | [Vofa_Debugging_Transmition](Tasks/Vofa_Transmit/vofa_transmit_task.c#L22) | 128×4 | osPriorityLow | 100Hz (10ms) | osDelay |
| defaultTask | StartDefaultTask | 128×4 | osPriorityNormal | 空闲 | osDelay(1) |

** IPC:**
- `DbusReadyHandle` — 二值信号量，UART ISR 释放 → 遥控任务获取
- `Signal_QueueHandle` — 消息队列 (16深 × `ChassisVelocityCmd`)，遥控任务写入 → 运动任务 drain

---

## 4. 初始化顺序 (已修复)

[freertos.c:104-158](Core/Src/freertos.c#L104-L158)

```
MX_FREERTOS_Init():
  1. Movement_Init()                        # 软I2C + MT6701 + M3508 + PID
  2. DbusReadyHandle = osSemaphoreNew()     # ★ 先创建信号量
  3. Signal_QueueHandle = osMessageQueueNew() # ★ 再创建队列
  4. RemoteControlTask_Init()               # ★ 最后使能UART IDLE中断
  5. osThreadNew × 4                        # 创建4个任务 (调度器未启动)
```

**关键:** `RemoteControlTask_Init()` 必须放在信号量创建之后，否则 UART IDLE 中断可能在 `DbusReadyHandle` 为 NULL 时触发 `xSemaphoreGiveFromISR(NULL)` → HardFault。

---

## 5. 数据流路径

```
遥控器 DR16 (100Hz, ~10ms/帧)
  │ DBUS 18B, UART2 100kbps, DMA循环缓冲(64B)
  ▼
UART IDLE中断 → HAL_UARTEx_RxEventCallback
  │ [remote_control_task.c:121-133](Tasks/Remote_Control/remote_control_task.c#L121-L133)
  │ RemoteControl_IdleCallback: 扫描0xA0帧头 → 18B拷贝 → ParseDbusFrame
  ▼
cached_state (6×float + 2×SwitchState + connected)
  │ [remote_control.c:200-215](Hardwares/Dbus_Remote_Control/remote_control.c#L200-L215)
  │ xSemaphoreGiveFromISR(DbusReadyHandle) ★
  ▼
Receive_and_Process_Signal [10ms超时唤醒, P2修复]
  │ [remote_control_task.c:55-119](Tasks/Remote_Control/remote_control_task.c#L55-L119)
  │ osSemaphoreAcquire(DbusReadyHandle, 10U) ← 10ms超时
  │ RemoteControl_GetState (临界区 __disable_irq 保护)
  │ RemoteControl_LossProtectionTick ★ 每周期调用
  │
  │ loss==0: 正常 → ly→vx(±3m/s), lx→vy(±3m/s), ry→wz(±6.28rad/s)
  │          恢复检查: ESTOP_RECOVER + 新帧 + connected + SW1_UP → Recover
  │ loss==1: 失联 → 队列零速 + SteeringChassis_EmergencyStop
  │ loss==2: 关断 → SteeringChassis_EmergencyStop
  ▼
Signal_Queue (16深 × ChassisVelocityCmd {vx,vy,wz})
  │ osMessageQueuePut (遥控任务, timeout=0)
  ▼
Motor_Movement [250Hz, vTaskDelayUntil]
  │ [movement_task.c:47-73](Tasks/Movement/movement_task.c#L47-L73)
  │ while-drain队列 (非阻塞, 仅保留最新值)
  │ SteeringChassis_SetVelocity → g_vx/g_vy/g_wz
  │ SteeringChassis_ControlLoop:
  │   1. ReadEncoders (4×MT6701 I2C, 失败>10次→急停)
  │   2. ReadMotorFeedback (8×M3508 CAN)
  │   3. 状态机分支 (ESTOP/HOMING/NORMAL)
  │   4. InverseKinematics (atan2 + 最短路径反转+滞回)
  │   5. ComputeSteerPID (位置环→rpm目标→速度环→电流)
  │   6. ComputeDrivePID (速度环→电流)
  │   7. ApplyCurrentRamp (±4000mA/周期)
  │   8. SendMotorCurrents (CAN 0x200 + 0x1FF)
  ▼
M3508电机 8× ←─ CAN 1Mbps ─→ 反馈帧 0x201-0x208
  │ [m3508.c:191-231](Hardwares/M3508/m3508.c#L191-L231)
  │ M3508_RxCallback (CAN RX中断): 解析角度/rpm/电流
```

---

## 6. 状态机

```
┌─────────┐  4轮到位(误差<3°)  ┌─────────┐
│  HOMING │ ─────────────────→ │ NORMAL  │
│ 回正中   │                    │ 正常控制 │
└────┬────┘                    └────┬────┘
     │ Recover() 确认              │ 编码器连续失败>10次
     │ (新帧+SW1_UP)              │ 或遥控失联
     │                            ▼
┌────┴──────────────┐      ┌──────────┐
│ ESTOP_RECOVER     │ ←─── │  ESTOP   │
│ 等待手动确认       │ 5次  │ 急停     │
│ 发送零电流         │ 连续 │ 零电流   │
└───────────────────┘ 成功 └──────────┘
     编码器每周期检查
```

**状态流转代码位置:** [steering_chassis.c:348-470](Tasks/Movement/steering_chassis.c#L348-L470)

---

## 7. 修复历史 (按时间倒序)

### 第四轮诊断 (2026-06-23)

| ID | 级别 | 问题 | 修复位置 | 状态 |
|---|---|---|---|---|
| H1 | 🔴 | CAN过滤器遗漏电机8反馈帧(0x208)，轮4舵向速度环开环 | [m3508.c:79-84](Hardwares/M3508/m3508.c#L79-L84) `FilterMaskIdHigh: 0x00FF→0x01FF` | ✅ |
| H2 | 🔴 | `rc_state.connected`永不清零，失联期间stale数据可触发急停恢复，绕过手动确认 | [remote_control_task.c:85-86](Tasks/Remote_Control/remote_control_task.c#L85-L86) 增加`status == osOK` | ✅ |

### 第三轮诊断 (2026-06-23)

| ID | 级别 | 问题 | 修复位置 | 状态 |
|---|---|---|---|---|
| #1 | 🔴 | UART IDLE中断在`DbusReadyHandle`创建前使能 → 启动时可能HardFault | [freertos.c:133-138](Core/Src/freertos.c#L133-L138) `RemoteControlTask_Init`后移至RTOS_QUEUES | ✅ |
| #2 | 🔴 | `osWaitForever`阻塞 → 失联后无DBUS帧 → 任务永久阻塞 → 急停永不触发 | [remote_control_task.c:64](Tasks/Remote_Control/remote_control_task.c#L64) `osWaitForever→10U` | ✅ |
| — | 🟡 | 软I2C总线未初始化 (SDA/SCL空闲电平不确定) | [steering_chassis.c:97-101](Tasks/Movement/steering_chassis.c#L97-L101) 增加 `SoftI2C_Init` 循环 | ✅ |

### 早期修复 (已内嵌于代码注释)

| 标记 | 问题 | 位置 |
|---|---|---|
| P0-1 | PID 4轮共享实例→积分交叉污染 | [steering_chassis.c:53-64](Tasks/Movement/steering_chassis.c#L53-L64) 改为数组 |
| P1-3 | LossProtectionTick 仅在 if(connected) 内调用 | [remote_control_task.c:69-73](Tasks/Remote_Control/remote_control_task.c#L69-L73) 提到外层 |
| P1-4 | 急停恢复状态机缺失 | [steering_chassis.c:366-381](Tasks/Movement/steering_chassis.c#L366-L381) |
| P1-5 | 最短路径反转无滞回→编码器噪声导致振荡 | [steering_chassis.c:707-735](Tasks/Movement/steering_chassis.c#L707-L735) 85°/95°滞回 |
| P2-7 | PID_Reset 只清零积分，未清零 prev_error→D项跳变 | [steering_chassis.c:285-292](Tasks/Movement/steering_chassis.c#L285-L292) |
| P2-8 | IsHomingDone 在急停时返回true | [steering_chassis.c:276-279](Tasks/Movement/steering_chassis.c#L276-L279) 改为仅NORMAL=true |
| P2-9 | 驱动减速比未应用到rpm计算 | [steering_chassis.c:656-659](Tasks/Movement/steering_chassis.c#L656-L659) 乘以减速比 |
| P2-10 | 编码器无效时保持旧电流 | [steering_chassis.c:759-763](Tasks/Movement/steering_chassis.c#L759-L763) 直接设0 |
| H3 | 舵向死区未启用 | [steering_chassis.c:172-175](Tasks/Movement/steering_chassis.c#L172-L175) PID_Init含deadband |
| N1 | 急停恢复确认机制 | [steering_chassis.c:324-332](Tasks/Movement/steering_chassis.c#L324-L332) + remote_control_task |

---

## 8. 待确认/未修复问题

| # | 级别 | 问题 | 位置 | 建议 |
|---|---|---|---|---|
| P1 | 🟡 | 反转退出逻辑: 退出时`target+180°`导致PID误差瞬间变为~175°，舵向电机抖动一周期 | [steering_chassis.c:727-733](Tasks/Movement/steering_chassis.c#L727-L733) | 退出反转时若`|raw_err|<85°`，仅翻转速不加180° |
| H3 | 🟢 | 摇杆右推→`lx正`→`vy正`→底盘左移（坐标系Y↑=左）。可能与操作直觉相反 | [remote_control_task.c:95](Tasks/Remote_Control/remote_control_task.c#L95) | 上机测试，如反向则改为`vy = -lx * MAX_XY` |
| — | 🟢 | [config.h:167-185](Hardwares/config.h#L167-L185) GPIO宏未使用，实际管脚在[soft_i2c.c:15-18](Hardwares/I2c_Soft/soft_i2c.c#L15-L18)硬编码 | 两处 | 统一到config.h或删除冗余 |
| — | 🟢 | 三任务同为 osPriorityLow，250Hz控制回路可能抖动 | [freertos.c:63-68](Core/Src/freertos.c#L63-L68) | Movement_Handle 提升至 AboveNormal |
| — | 🟢 | `HAL_CAN_AddTxMessage` 返回值未检查 | [m3508.c:149,161](Hardwares/M3508/m3508.c#L149) | 增加错误计数/日志 |
| — | 🟢 | PID 全部 Kd=0 (无微分项)，不影响稳态但响应略慢 | [config.h:120-159](Hardwares/config.h#L120-L159) | 调参优化时可启用 |
| — | 🟢 | `REDUCTION_STEER_1~4` 宏定义但未使用 | [config.h:32-35](Hardwares/config.h#L32-L35) | 集成到舵向PID或删除 |

---

## 9. 关键配置参数

### 几何与机械

| 宏 | 值 | 说明 |
|---|---|---|
| `WHEEL_BASE_A` | 0.15f | 前后半轴距 (m) |
| `TRACK_WIDTH_B` | 0.15f | 左右半轮距 (m) |
| `WHEEL_RADIUS` | 0.076f | 车轮半径 (m) |
| `REDUCTION_DRIVE_1/4` | 14.0f | 轮1/4驱动减速比 |
| `REDUCTION_DRIVE_2/3` | 19.0f | 轮2/3驱动减速比 |
| `REDUCTION_STEER_1~4` | 14.0f | 舵向减速比 (待标定) |

### 编码器标定

| 宏 | 值 | 说明 |
|---|---|---|
| `STEER_INIT_RAW_1` | 13600 | 轮1回正原始值 (14位) |
| `STEER_INIT_RAW_2` | 11160 | 轮2 |
| `STEER_INIT_RAW_3` | 15568 | 轮3 |
| `STEER_INIT_RAW_4` | 3188 | 轮4 |
| `STEER_OFFSET_1` | 175.8f | 轮1零位偏移 (度) |
| `STEER_OFFSET_2` | 126.6f | 轮2 |
| `STEER_OFFSET_3` | 213.0f | 轮3 |
| `STEER_OFFSET_4` | 313.0f | 轮4 |

### 控制时序

| 宏 | 值 | 说明 |
|---|---|---|
| `CONTROL_PERIOD_MS` | 4 | 主控制周期 (ms), 250Hz |
| `CONTROL_DT` | 0.004f | PID dt (s), 必须 = CONTROL_PERIOD_MS/1000 |
| `CURRENT_RAMP_RATE` | 4000 | 电流变化限幅 (mA/周期) |
| `STEER_DEADBAND_DEG` | 1.0f | 舵向到位死区 (±度) |
| `HOMING_THRESHOLD_DEG` | 3.0f | 回正到位精度 |

### 遥控器

| 宏 | 值 | 说明 |
|---|---|---|
| `REMOTE_TIMEOUT_MS` | 50 | 失联判定超时 |
| `REMOTE_ESTOP_HOLD_MS` | 200 | 急停→关断延时 |
| `REMOTE_DEADBAND` | 0.05f | 摇杆死区 (5%) |
| `REMOTE_VEL_MAX_XY` | 3.0f | 最大平动速度 (m/s) |
| `REMOTE_VEL_MAX_WZ` | 6.283f | 最大旋转速度 (rad/s, ≈1rps) |
| `REMOTE_RAW_MIN/MID/MAX` | 364/1024/1684 | DR16 摇杆原始值范围 |

### 编码器容错

| 宏 | 值 | 说明 |
|---|---|---|
| `ENCODER_MAX_FAIL_COUNT` | 10 | 连续失败触发急停 |
| `ENCODER_RECOVERY_COUNT` | 5 | 连续成功确认恢复 |

---

## 10. PID 参数表

### 舵向位置环 (外环: °→rpm)

| 轮 | Kp | Ki | Kd | 积分限幅 | 输出限幅(rpm) |
|---|---|---|---|---|---|
| 1 | 8.0 | 0.2 | 0 | 500 | 1000 |
| 2 | 8.0 | 0.2 | 0 | 500 | 1000 |
| 3 | 8.0 | 0.2 | 0 | 500 | 1000 |
| 4 | 8.0 | 0.2 | 0 | 500 | 1000 |

### 舵向速度环 (内环: rpm→mA)

| 轮 | Kp | Ki | Kd | 积分限幅 | 输出限幅(mA) |
|---|---|---|---|---|---|
| 1-4 | 10.0 | 0.5 | 0 | 5000 | 16384 |

### 驱动速度环 (rpm→mA)

| 轮 | Kp | Ki | Kd | 积分限幅 | 输出限幅(mA) |
|---|---|---|---|---|---|
| 1-4 | 8.0 | 0.3 | 0 | 5000 | 16384 |

### 回正位置环 (HOMING专用)

| 轮 | Kp | Ki | Kd | 积分限幅 | 输出限幅(rpm) | Deadband |
|---|---|---|---|---|---|---|
| 1-4 | 4.0 | 0.1 | 0 | 250 | 500 | 3.0° |

---

## 11. CAN 通信规格

| 参数 | 值 |
|---|---|
| 波特率 | 1Mbps (42MHz / Prescaler=3 / BS1=9TQ / BS2=4TQ) |
| 控制帧 | 0x200 (电机1-4), 0x1FF (电机5-8), 标准帧, DLC=8 |
| 反馈帧 | 0x201-0x208, 标准帧, 1000Hz (每电机每1ms回传1帧) |
| 电流范围 | -16384 ~ +16384 (硬件限幅 ~±20A) |
| 过滤器 | Bank0, 32位IDMASK, 硬件匹配 0x200-0x20F, 软件过滤 0x201-0x208 |

### CAN 数据格式

**电流指令 (TX, 大端):** `{motor_N_current_H, motor_N_current_L}` × 4电机

**反馈帧 (RX, 大端):** `{encoder_H, encoder_L, rpm_H, rpm_L, current_H, current_L, reserved_H, reserved_L}`

---

## 12. 调试数据 (Vofa)

[Tasks/Vofa_Transmit/vofa_transmit_task.c](Tasks/Vofa_Transmit/vofa_transmit_task.c)

- 协议: JustFloat (4B浮点 + 尾帧)
- 周期: 100Hz (osDelay 10ms)
- 通道: 16通道 = steer_angle[4] + steer_target[4] + drive_rpm[4] + drive_target[4]
- UART: UART4

---

## 13. 中断优先级

| 中断 | 优先级 | 用途 |
|---|---|---|
| CAN1_RX0 | 5,0 | M3508 反馈帧接收 |
| USART2 | (CubeMX默认) | DBUS DMA IDLE |
| UART4 | (CubeMX默认) | Vofa 发送DMA |
| DMA1_Stream4 | (CubeMX默认) | UART4 TX DMA |
| DMA1_Stream5 | (CubeMX默认) | USART2 RX DMA |

---

## 14. 上机测试检查清单

- [ ] 上电 → 4轮回正完成 → CHASSIS_NORMAL
- [ ] 遥控器右推→底盘右移（否则H3需修复）
- [ ] 遥控器前推→底盘前进
- [ ] 遥控器旋转→底盘原地旋转
- [ ] 遥控器断电 → 50ms内急停 → 200ms关断电流
- [ ] 遥控器重新上电 + SW1拨UP → 回正 → 正常控制
- [ ] Vofa 16通道数据正常刷新
- [ ] 轮4舵向电机反馈正常（H1修复验证：steer_rpm[3] 非恒零）
- [ ] 急停恢复仅在新DBUS帧+SW1 UP时触发（H2修复验证）

---

## 15. 建议的后续技能

日后修改时建议调用:
- `/diagnose` — 针对具体问题的诊断
- `/review` — 修改后的代码审查
- `/code-review` — diff级别的正确性审查
