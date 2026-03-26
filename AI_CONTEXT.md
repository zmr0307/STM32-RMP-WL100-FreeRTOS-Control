# 核心交接档案：项目当前状态上下文 (AI Inter-Conversation Context)

> **⚠️ 给下一代 AI 助手的铁律指令：**
> 当你接手这个项目时，必须**强制、优先、完整**地逐字阅读本文件。
> 这是无数次实车测试踩坑后凝结的**架构红线**！
> 绝不可擅自违背其中确立的通信策略、字节序解析法则和发包频率限制。

---

## 0. 极简当前状态摘要 (TL;DR)

- **阶段**: STM32 固件端**工程化稳定性增强阶段**完成，准备室内巡检部署。
- **已完成**: CAN 双向通信 ✅ | 自愈重连机制 ✅ | USART3 串口接收引擎 ✅ | 实车热插拔测试 ✅ | Jetson 下发控制 ✅ | 里程计高速上报 ✅ | **故障检测与紧急停车** ✅ | **IWDG 硬件看门狗** ✅ | **遥控器抢权感知** ✅ | **CAN Bus-Off 自动恢复** ✅ | **USART3 竞态修复** ✅ | **状态缓存快照** ✅
- **当前测试方式**: STM32 与底盘 CAN 正常通信，USART3 50Hz 上报里程计给 Jetson，具备完整的故障安全拦截和遥控器模式感知。
- **下一步**: 开发 Jetson 端 ROS2 里程计接收节点；电池状态上报 (TYPE=0x02)。

---

## 1. 硬件选型与引脚分配（绝对不能改动）

| 硬件 | 型号/参数 |
|------|-----------|
| 主控芯片 | STM32F407ZGT6（正点原子探索者开发板） |
| 系统时钟 | 168MHz 主频（HSE 8MHz × PLL 336/8/2） |
| 底盘型号 | RMP-WL100 四驱四转向全向移动底座 |
| RTOS | FreeRTOS（1ms 节拍，configTICK_RATE_HZ=1000） |

### CAN1 接口（连接底盘）
- TX: **PA12**（复用推挽 AF9）
- RX: **PA11**（复用推挽 AF9）
- 波特率: **500 Kbps**
- 时序参数: `can_init(CAN_SJW_1TQ, CAN_BS2_5TQ, CAN_BS1_8TQ, 6, CAN_MODE_NORMAL)`
- 公式验证: `42MHz / (6 × (1+8+5)) = 500Kbps` ✅
- 过滤器: 32位掩码模式，Mask 全 0（放行所有 CAN ID）
- 接收机制: CAN1_RX0 中断（优先级 6）+ FreeRTOS 队列
- **AutoBusOff: ENABLE**（硬件自动 Bus-Off 恢复，约 2.8ms @500Kbps）

### USART3 接口（预留给 Jetson ROS2）
- TX: **PB10**（接 Jetson RX）
- RX: **PB11**（接 Jetson TX）
- 波特率: **115200**
- 接收机制: **IDLE 空闲中断 + DMA 循环接收**
- DMA 通道: **DMA1_Stream1_Channel4**（F407 硬件固定，不可改）
- 中断优先级: 6（位于 FreeRTOS 安全线 5 之下）
- **竞态保护**: `jetson_report_odom()` 阻塞发送期间禁用 IDLE 中断，防止 ISR 竞争 HAL Lock

### USART1（调试打印）
- 波特率: 115200，接电脑串口助手用于调试打印
- 使用 `safe_printf()` 互斥锁保护多任务并发打印安全

### LCD（实时仪表盘）
- 接口: FSMC 并行总线
- 用途: 显示底盘在线状态、控制模式、故障状态、电池信息、轮速

---

## 2. 核心通信协议（防坑避雷指南）

### 2.1 Jetson → STM32 串口协议（USART3，10字节固定帧）

```
[A5] [5A] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]
 帧头         X速度       Y速度       Z角速度    校验   帧尾
```

- **编码规则**: 所有速度均为**真实值 × 1000 的 16位有符号整数，大端序（高位在前）**
  - 例: Vx=0.1m/s → `0x00 64`（100）；Vx=-0.1m/s → `0xFF 9C`（-100，补码）
- **XOR 校验**: 将前 8 个字节（A5 5A + 6字节数据）全部异或，结果放在第 9 字节
- **三重校验机制**: 帧头 `A5 5A` + 帧尾 `EE` + XOR 校验，任一不符即丢弃整帧

### 2.2 STM32 → Jetson 里程计上报串口协议（USART3，11字节固定帧）

```
[AA] [55] [TYPE] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]
 帧头    消息类型   X速度       Y速度       Z角速度    校验   帧尾
```

- **说明**: `TYPE = 0x01`（里程计速度）。与下发帧头完全区分（`A5 5A` vs `AA 55`），实现零冲突。
- **发送源**: `chassis_test.c` 中底盘反馈 `0x104` 运动状态帧触发（20ms周期 / 50Hz）。
- **编码**: 所有速度均为真实值 × 1000 的 16 位有符号整数，大端序。
- **XOR 校验**: 对 `AA 55 01 + 6字节数据` 共计 9 个字节进行异或。

### 2.3 STM32 <-> 底盘 CAN 协议（CAN1，标准数据帧）

#### 发送帧（STM32 → 底盘）

| CAN ID | 用途 | 重要说明 |
|--------|------|----------|
| `0x400` | 控制模式使能 | data[0]=`0x01` 表示请求进入 CAN 控制模式 |
| `0x401` | 运动模式选择 | data[0]=`0x02` (FORWARD) / `0x04` (PARK驻车) |
| `0x402` | 运动控制（最常用）| Vx/Vy/Vz，大端 × 1000，限幅 ±2.0 |

#### 接收帧（底盘 → STM32）

| CAN ID | 用途 | 周期 |
|--------|------|------|
| `0x100` | 系统状态（ctrl_mode/SOC/电压/电流/急停位） | 1000ms |
| `0x101` | 四轮速度 | 30ms |
| `0x102` | 四轮角度 | 30ms |
| `0x103` | 故障定位（fault_flag/walk/steer故障位） | 1000ms |
| `0x104` | 运动状态反馈 Vx/Vy/Vz + 运动模式 | 20ms |

#### ⚠️ 极重要防坑点

**【坑1】夺权使能码 vs 反馈状态码完全不一样！**
- 发送 `0x400` 时，data[0] 填 **`0x01`**（枚举 `CTRL_MODE_CAN`）
- 读取 `0x100` 反馈 ctrl_mode 字段判断是否进入 CAN 模式，要等于 **`0x02`**（宏 `CTRL_MODE_FB_CAN`）
- **两者绝对不能混用！**

**【坑2】0x402 发包频率不能超过 50Hz！**
- 历史测试：以 10ms(100Hz) 高频发送 `0x402`，底盘 MCU 邮箱被冲爆。
- 当前锁定：**`vTaskDelay(20)` = 50Hz**，稳定无误。

**【坑3】电池电流是标准大端序！**
- `battery_current = (int16_t)((data[6] << 8) | data[7])` ← 标准大端！
- 小端解码会得到离谱的 -14337mA，切勿修改回小端！

**【坑4】0x100 反馈的 ctrl_mode 含义**
- `0x00` = 待机模式（底盘刚上电）
- `0x01` = 遥控器模式（有人在用遥控器）
- `0x02` = CAN 指令控制模式（STM32 接管中）

---

## 3. FreeRTOS 任务架构

### 任务清单（共 4 个）

| 任务名 | 优先级 | 栈大小 | 周期 | 职责 |
|--------|--------|--------|------|------|
| `chassis_ctrl_task` | 2 | 512 Words | 20ms | 自愈控制+故障拦截+速度下发 |
| `chassis_rx_task` | 3（最高） | 512 Words | 200ms超时 | CAN 数据接收解码+里程计上报 |
| `led_task` | 1 | 128 Words | 500ms | 心跳闪灯 + **IWDG 喂狗** |
| `start_task` | 1 | 128 Words | 一次性 | 创建其他任务后自删 |

### chassis_ctrl_task 详解（核心任务）

```
每 20ms 循环一次:
  0. 缓存状态快照: cached_online + cached_ctrl_mode（防抢占不一致）
  1. 检查 cached_online 和 cached_ctrl_mode
  2. 若离线或模式错误:
     - 强制 target_vx/vy/vz = 0
     - LCD 显示:
       · 离线 → "Mode: OFFLINE" (红)
       · 遥控器(0x01) → "Mode: REMOTE CTRL" (黄) ← 不发0x400抢权
       · 待机(0x00) → "Mode: STANDBY" (红) ← 发0x400+0x401抢权
     - vTaskDelay(500) + continue
  3. 若在线且 CAN 控制模式:
     - LCD 显示 "Mode: JETSON CMD" (蓝)
     - 临界区读取 Jetson 指令
     - Jetson 500ms 超时归零保护
     - ❗ 故障/急停安全拦截:
       · 急停(0x100 bit0) → 零速度 + "E-STOP!!!" (红)
       · 驱动器故障(0x103) → 零速度, 3秒后切 PARK 驻车
       · 故障恢复 → 自动切回 FORWARD 模式
     - 发送 0x402 运动控制帧
```

### chassis_rx_task 详解

```
死循环:
  xQueueReceive(g_can_rx_queue, &rx_data, pdMS_TO_TICKS(200))
  if 收到数据:
    chassis_process_feedback() 解码存入 g_chassis_state
    0x100 到了: 打印串口仪表盘 + LCD 电池/轮速/UART错误计数
    0x103 到了: LCD 显示故障状态 (绿=NONE / 红=故障码)
    0x104 到了: 调用 jetson_report_odom() 上报里程计 (50Hz)
  无论是否收到数据:
    每 200ms 强制刷新 LCD 在线状态
```

---

## 4. 安全机制（完整清单）

### 4.1 故障检测与紧急停车 🔴
- `chassis_has_fault()`: 查询 0x103 故障标志位 (fault_flag != 0)
- `chassis_is_estop()`: 查询 0x100 急停开关 (peripheral_state bit0)
- **安全优先级**: 急停 > 驱动器故障 > 正常控制
- **驱动器故障处理**: 零速度 → 持续3秒 → `MOTION_MODE_PARK` 驻车锁死
- **故障恢复**: 故障消失后自动发 `MOTION_MODE_FORWARD` 恢复

### 4.2 IWDG 硬件看门狗 🟡
- 参数: 预分频=64, 重装载=2000, **超时=4秒**
- 喂狗位置: `led_task` (最低优先级, 每500ms)
- 调试保护: `__HAL_DBGMCU_FREEZE_IWDG()` 防断点复位
- 触发条件: 高优先级任务死锁/死循环饿死 led_task → 4秒后硬件复位

### 4.3 遥控器抢权感知 🟡
- 检测 `ctrl_mode == CTRL_MODE_FB_REMOTE (0x01)` 时:
  - **不发 0x400** 抢权（避免与遥控器打架）
  - LCD 显示 "Mode: REMOTE CTRL" (黄色)
  - 速度归零
- 遥控器松手 → ctrl_mode 回 0x00 → STM32 自动重新抢权接管

### 4.4 CAN Bus-Off 自动恢复 🟡
- `can.c` 中 `AutoBusOff = ENABLE`
- Bus-Off 后硬件自动恢复（~2.8ms @500Kbps），无需软件干预

### 4.5 USART3 句柄竞态保护 🟡
- `jetson_report_odom()` 阻塞发送 11 字节期间 (~1ms):
  - 发送前 `__HAL_UART_DISABLE_IT(&g_uart3_handle, UART_IT_IDLE)`
  - 发送后 `__HAL_UART_ENABLE_IT(&g_uart3_handle, UART_IT_IDLE)`
- 防止 IDLE 中断的 `AbortReceive` + `Receive_DMA` 竞争 HAL Lock
- DMA 仍在后台缓冲，不丢数据

### 4.6 状态缓存快照 🟢
- `chassis_ctrl_task` while 循环开头缓存:
  ```c
  uint8_t cached_online = chassis_is_online();
  uint8_t cached_ctrl_mode = chassis_get_state()->system_status.ctrl_mode;
  ```
- 后续判断全部使用缓存值，消除高优先级任务抢占导致的前后不一致

### 4.7 Jetson 断线超时归零保护 ⚡
- 每次解码成功记录 `HAL_GetTick()` 至 `last_valid_tick`
- 500ms 无新帧 → 强制速度归零（防断线飞车）
- 上电时 `last_valid_tick = 0` → 立刻满足超时 → 底盘静止等待

### 4.8 竞态临界区保护
- 读取 `g_jetson_cmd` 用 `taskENTER_CRITICAL()/taskEXIT_CRITICAL()`
- 防止 USART3 中断写入导致半帧撕裂

### 4.9 printf 互斥保护
- `safe_printf()` + FreeRTOS 互斥锁，多任务并发打印安全

### 4.10 栈溢出检测
- `configCHECK_FOR_STACK_OVERFLOW = 2`（水印法）
- 溢出触发 `vApplicationStackOverflowHook()`: LCD 爆红 + LED 闪烁 → 4秒后 IWDG 复位

### 4.11 速度限幅防飞车
- `encode_motion_ctrl()` 中强制限幅 ±2.0 m/s（rad/s）
- 转换倍率固定为 ×1000

---

## 5. LCD 仪表盘布局

| Y坐标 | 内容 | 刷新源 | 颜色逻辑 |
|-------|------|--------|----------|
| 130 | 通讯状态 | ctrl_task | 绿=在线 / 红=离线 / 品红=重连 |
| 150 | 控制模式 | ctrl_task | 蓝=JETSON CMD / 黄=REMOTE CTRL / 红=OFFLINE/STANDBY |
| 170 | 运动指令 | ctrl_task | 品红=JETSON CMD / 蓝=STOP / 红=E-STOP/FAULT/PARKED/CANERR |
| 190 | 在线心跳 | rx_task | 绿=ONLINE / 红=OFFLINE |
| 210 | 电池信息 | rx_task(0x100) | 蓝色，格式: BAT:25.2V 85% -570mA |
| 230 | 四轮速度 | rx_task(0x100) | 品红，格式: Spd: W1 W2 W3 W4 (mm/s) |
| 250 | 四轮角度 | rx_task(0x100) | 品红，格式: Ang: A1 A2 A3 A4 (deg) |
| 270 | UART3错误 | rx_task(0x100) | 绿=0 / 红>0，ORE 错误计数 |
| 290 | 故障状态 | rx_task(0x103) | 绿=NONE / 红=W故障码 S故障码 |

---

## 6. 已踩坑汇总（历史防雷区）

| 坑 | 错误现象 | 根因 | 正确做法 |
|----|----------|------|----------|
| ctrl_mode 混用 | 永远 WAIT FOR SYNC | 发送 0x01，反馈比较也用 0x01，但实际反馈是 0x02 | 发送用枚举，比较用 CTRL_MODE_FB_CAN(0x02) |
| 电流小端解码 | 显示 -143370mA | 误信原厂文档小端说明 | 大端：(data[6]<<8)&#124;data[7]，实测-560mA ✅ |
| CAN 100Hz 超速发包 | CANERR 报错 | 底盘 MCU 邮箱冲爆 | vTaskDelay(20) 限 50Hz |
| 断电 LCD 不更新 | LCD 残留绿色假在线 | xQueueReceive 用 portMAX_DELAY 永久阻塞 | 改为 pdMS_TO_TICKS(200) 超时 |
| 半帧撕裂风险 | 偶发抽搐抖动 | USART3 中断与主任务无保护并发 | taskENTER_CRITICAL 临界区 |
| HAL 句柄竞态 | 长时间运行 USART3 卡死 | 阻塞发送与 IDLE 中断竞争 Lock | 发送期间禁用 IDLE 中断 |
| 遥控器打架 | 底盘来回抖动 | STM32 和遥控器反复抢权 | 识别遥控器模式时不发 0x400 |

---

## 7. 下一步计划（NEXT ACTIONS）

### 任务一：Jetson ROS2 控制联调 [已完成一半]
- **下发控制**: ✅ Jetson Nano 下发 ROS2 键盘控制节点代码已完成，STM32 接收正常。

### 任务二：里程计上报（Odometry）[STM32 端已完成]
- ✅ 将 0x104 反馈的 Vx/Vy/Vz 每 20ms 以 11 字节协议包发送给 USART3。
- ⏳ **待完成**: 在 Jetson 端 ROS2 串口节点中，增加对 `AA 55 01` 的滑窗搜帧截获，执行积分并发布 `/odom` 与 TF。

### 任务三：电池状态上报
- ⏳ 暂未开始。计划将 0x100 中的 SOC、电压、电流，以 `TYPE = 0x02` 的 11 字节新数据包格式低频上报给 Jetson。

### 任务四：固件版本管理
- ⏳ 在代码中添加统一的固件版本号宏，便于烧录管理。

---

## 8. 已知非关键观察项

| # | 项目 | 影响 |
|---|------|------|
| 1 | CAN 队列深度仅 10 | 当前实测够用，未来增加功能需观察 |
| 2 | btim.c 与 gtim.c 定时器回调冲突 | gtim.c 未编译，当前无影响；若未来使用需重构 |
| 3 | HAL_GetTick 49.7天溢出 | 无符号减法天然处理，室内巡检不影响 |
| 4 | chassis_ctrl_task LCD 刷新 50Hz | 性能浪费，可降至 5Hz 节省 CPU |
