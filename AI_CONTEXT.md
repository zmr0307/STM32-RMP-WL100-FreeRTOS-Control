# 核心交接档案：项目当前状态上下文 (AI Inter-Conversation Context)

> **⚠️ 给下一代 AI 助手的铁律指令：**
> 当你接手这个项目时，必须**强制、优先、完整**地逐字阅读本文件。
> 这是无数次实车测试踩坑后凝结的**架构红线**！
> 绝不可擅自违背其中确立的通信策略、字节序解析法则和发包频率限制。

---

## 0. 极简当前状态摘要 (TL;DR)

- **阶段**: STM32 固件端**真车联调阶段**已完成基础验证。
- **已完成**: CAN 双向通信 ✅ | 自愈重连机制 ✅ | USART3 串口接收引擎 ✅ | 实车热插拔测试 ✅
- **当前测试方式**: 用**电脑串口助手（手动发 HEX 指令）**模拟 Jetson 指令，验证运动控制链路。Jetson Nano 实体**尚未接入**，ROS2 节点**尚未开发**。
- **下一步**: 开发 Jetson 端 ROS2 节点，通过 USART3 向 STM32 发送标准速度帧。

---

## 1. 硬件选型与引脚分配（绝对不能改动）

| 硬件 | 型号/参数 |
|------|-----------|
| 主控芯片 | STM32F407ZGT6（正点原子探索者开发板） |
| 系统时钟 | 168MHz 主频（HSE 25MHz × PLL 336/8/2） |
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

### USART3 接口（预留给 Jetson ROS2）
- TX: **PB10**（接 Jetson RX）
- RX: **PB11**（接 Jetson TX）
- 波特率: **115200**
- 接收机制: **IDLE 空闲中断 + DMA 循环接收**
- DMA 通道: **DMA1_Stream1_Channel4**（F407 硬件固定，不可改）
- 中断优先级: 6（位于 FreeRTOS 安全线 5 之下）

### USART1（调试打印）
- 波特率: 115200，接电脑串口助手用于调试打印
- 使用 `safe_printf()` 互斥锁保护多任务并发打印安全

### LCD（实时仪表盘）
- 接口: FSMC 并行总线
- 用途: 显示底盘在线状态、CAN 模式、电池电压/电流/SOC、轮速

---

## 2. 核心通信协议（防坑避雷指南）

### 2.1 Jetson ↔ STM32 串口协议（USART3，10字节固定帧）

```
[A5] [5A] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]
 帧头         X速度       Y速度       Z角速度    校验   帧尾
```

- **编码规则**: 所有速度均为**真实值 × 1000 的 16位有符号整数，大端序（高位在前）**
  - 例: Vx=0.1m/s → `0x00 64`（100）；Vx=-0.1m/s → `0xFF 9C`（-100，补码）
  - 例: Vx=0.2m/s → `0x00 C8`；Vx=-0.2m/s → `0xFF 38`
- **XOR 校验**: 将前 8 个字节（A5 5A + 6字节数据）全部异或，结果放在第 9 字节
- **三重校验机制**: 帧头 `A5 5A` + 帧尾 `EE` + XOR 校验，任一不符即丢弃整帧

### 2.2 STM32 ↔ 底盘 CAN 协议（CAN1，标准数据帧）

#### 发送帧（STM32 → 底盘）

| CAN ID | 用途 | 重要说明 |
|--------|------|----------|
| `0x400` | 控制模式使能 | data[0]=`0x01` 表示请求进入 CAN 控制模式 |
| `0x401` | 运动模式选择 | data[0]=`0x02` (MOTION_MODE_FORWARD，最佳全向模式) |
| `0x402` | 运动控制（最常用）| Vx/Vy/Vz，大端 × 1000，限幅 ±2.0 |

#### 接收帧（底盘 → STM32）

| CAN ID | 用途 | 周期 |
|--------|------|------|
| `0x100` | 系统状态 | 1000ms |
| `0x101` | 四轮速度 | 30ms |
| `0x102` | 四轮角度 | 30ms |
| `0x103` | 故障定位 | 1000ms |
| `0x104` | 运动状态反馈 Vx/Vy/Vz | 20ms |

#### ⚠️ 极重要防坑点

**【坑1】夺权使能码 vs 反馈状态码完全不一样！**
- 发送 `0x400` 时，data[0] 填 **`0x01`**（枚举 `CTRL_MODE_CAN`）
- 读取 `0x100` 反馈 ctrl_mode 字段判断是否进入 CAN 模式，要等于 **`0x02`**（宏 `CTRL_MODE_FB_CAN`）
- **两者绝对不能混用！** 历史上曾因混用导致系统永远认为底盘没在 CAN 模式下，自愈死循环。

**【坑2】0x402 发包频率不能超过 50Hz！**
- 历史测试：以 10ms(100Hz) 高频发送 `0x402`，原厂底盘 MCU 邮箱被冲爆，报 `CANERR`，底盘拒收。
- 当前锁定：**`vTaskDelay(20)` = 50Hz**，稳定无误。

**【坑3】电池电流是标准大端序！**
- 原厂文档曾声称电流 Byte[6-7] 为"特殊小端"，经真车静止状态（-560mA）实测验证：
- **正确解码**: `battery_current = (int16_t)((data[6] << 8) | data[7])` ← 标准大端！
- 小端解码会得到离谱的 -14337mA，切勿修改回小端！

---

## 3. FreeRTOS 任务架构

### 任务清单（共 4 个）

| 任务名 | 优先级 | 栈大小 | 周期 | 职责 |
|--------|--------|--------|------|------|
| `chassis_ctrl_task` | 2 | 512 Words | 20ms | 自愈控制+速度下发 |
| `chassis_rx_task` | 3（最高） | 512 Words | 200ms超时 | CAN 数据接收解码 |
| `led_task` | 1 | 128 Words | 500ms | 心跳闪灯 |
| `start_task` | 1 | 128 Words | 一次性 | 创建其他任务后自删 |

### chassis_ctrl_task 详解（核心任务）

```
每 20ms 循环一次:
  1. 检查 chassis_is_online() 和 ctrl_mode 是否 == CTRL_MODE_FB_CAN(0x02)
  2. 若离线或模式错误:
     - 强制 target_vx/vy/vz = 0
     - 每 500ms 发一次 0x400(夺权) + 0x401(运动模式)
     - LCD 显示红色 OFFLINE 或品红色 Reconnecting
     - continue 跳过本轮控制
  3. 若在线且模式正确:
     - LCD 显示绿色 CAN ONLINE
     - __disable_irq() 临界区读取 Jetson 指令（防半帧撕裂）
     - 发送 0x402 运动控制帧
```

### chassis_rx_task 详解

```
死循环:
  xQueueReceive(g_can_rx_queue, &rx_data, pdMS_TO_TICKS(200))
  if 收到数据:
    chassis_process_feedback() 解码存入 g_chassis_state
    0x100 到了: 打印串口仪表盘 + 刷新 LCD
  无论是否收到数据:
    每 200ms 强制刷新 LCD 在线状态（防假在线残影）
```

---

## 4. 重要安全机制

### 4.1 竞态临界区保护
- `chassis_ctrl_task` 读取 `g_jetson_cmd` 时，用 `__disable_irq()/__enable_irq()` 包裹
- 原因：USART3 中断会在任意时刻写入 g_jetson_cmd，若不保护会发生"半帧撕裂"（Vx 读旧值，Vy 读新值）

### 4.2 printf 互斥保护
- 定义了 `safe_printf()` 函数，内部使用 FreeRTOS 互斥锁
- 宏 `#define printf(...) safe_printf(...)` 全局替换，多任务并发打印安全

### 4.3 栈溢出检测
- `configCHECK_FOR_STACK_OVERFLOW = 2`（水印法，最严格）
- 溢出时触发 `vApplicationStackOverflowHook()`：LCD 爆红 `!!! STACK OVERFLOW !!!` + LED0 闪烁死锁

### 4.4 速度限幅防飞车
- `encode_motion_ctrl()` 中对 Vx/Vy/Vz 强制限幅 ±2.0 m/s（rad/s）
- 转换倍率固定为 **×1000**（Vx/Vy/Vz 统一，含角速度 Vz）

---

## 5. 已踩坑汇总（历史防雷区）

| 坑 | 错误现象 | 根因 | 正确做法 |
|----|----------|------|----------|
| ctrl_mode 混用 | 永远 WAIT FOR SYNC | 发送 0x01，反馈比较也用 0x01，但实际反馈是 0x02 | 发送用枚举，比较用 CTRL_MODE_FB_CAN(0x02) |
| 电流小端解码 | 显示 -143370mA | 误信原厂文档小端说明 | 大端：(data[6]<<8)&#124;data[7]，实测-560mA ✅ |
| CAN 100Hz 超速发包 | CANERR 报错 | 底盘 MCU 邮箱冲爆 | vTaskDelay(20) 限 50Hz |
| 断电 LCD 不更新 | LCD 残留绿色假在线 | xQueueReceive 用 portMAX_DELAY 永久阻塞 | 改为 pdMS_TO_TICKS(200) 超时 |
| 半帧撕裂风险 | 偶发抽搐抖动 | USART3 中断与主任务无保护并发 | __disable_irq() 临界区 |
| freertos_demo 链接冲突 | 链接器重复定义报错风险 | 两个.c文件有同名全局变量 | freertos_demo.c 全局变量加 static |

---

## 6. 下一步计划（NEXT ACTIONS）

### 任务一：Jetson ROS2 控制联调（核心目标）
- **控制源**: 在 Jetson Nano 上运行 ROS2 标准键盘控制节点 `teleop_twist_keyboard`。
- **转发逻辑**: 开发一个极简的 ROS2 节点，订阅 `/cmd_vel` 话题，将接收到的 `geometry_msgs::Twist` 消息（线速度 x/y, 角速度 z）按照 10 字节自定义协议（A5 5A...EE）打包，通过串口发送给 STM32。
- **目标**: 实现通过笔记本键盘远程遥控真车平稳移动。

### 任务二：里程计上报（Odometry）
- 利用 0x104 反馈的 Vx/Vy/Vz（20ms 周期）进行积分
- 将累计算好的 X/Y/θ 通过 USART3 上报给 Jetson，作为里程计数据源

### 任务三：状态监控上报
- 将 0x100 中的 SOC、电压、电流、故障码低频上报给 Jetson
- 在 RViz 或监控节点显示电池状态和故障预警
