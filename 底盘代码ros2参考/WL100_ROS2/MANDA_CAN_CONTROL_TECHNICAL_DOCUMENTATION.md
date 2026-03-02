# MANDA CAN Control 技术文档

## 目录
1. [项目概述](#项目概述)
2. [系统架构](#系统架构)
3. [消息定义](#消息定义)
4. [API文档](#api文档)
5. [函数详细分析](#函数详细分析)
6. [CAN协议规范](#can协议规范)
7. [编译与部署](#编译与部署)
8. [代码优化说明](#代码优化说明)
9. [错误处理](#错误处理)
10. [故障排除](#故障排除)

---

## 项目概述

### 项目简介
MANDA CAN Control 是一个基于ROS（Robot Operating System）的CAN总线控制系统，用于四轮转向车辆的实时控制。该系统通过CAN总线与车辆底盘进行通信，实现速度控制、转向控制、运动控制等功能。

### 主要功能
- **实时CAN通信**：与车辆底盘进行双向CAN通信
- **多轮控制**：支持四轮独立的速度和转向控制
- **运动控制**：提供线性和角速度控制接口
- **状态监控**：实时监控电池状态、系统状态、故障信息
- **模式切换**：支持多种控制模式和运动模式

### 技术栈
- **ROS版本**：ROS2 Foxy (ament_cmake)
- **编程语言**：C++14
- **通信协议**：CAN总线 (SocketCAN)
- **操作系统**：Linux (支持CAN接口)

---

## 系统架构

### 主要变化说明
相比之前的版本，当前代码有以下重要变化：

1. **接口类型变化**：
   - 运动模式控制从话题订阅改为服务调用
   - 移除了控制模式话题订阅功能
   - 新增数字输出控制服务

2. **性能优化**：
   - CAN数据解析使用memcpy替代位运算，提高性能
   - 线程管理使用detach()替代join()，避免阻塞

3. **功能增强**：
   - 添加了新的CAN ID 0x405用于数字输出控制
   - 修复了电池电流解析的字节序问题
   - 运动反馈消息增加了mode_type字段

### 整体架构图
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   ROS Nodes     │    │  CAN Control    │    │   Vehicle       │
│                 │    │                 │    │   Chassis       │
│ ┌─────────────┐ │    │ ┌─────────────┐ │◄──►│ ┌─────────────┐ │
│ │ Publishers  │ │    │ │ CAN Socket  │ │    │ │ CAN Bus     │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │                 │
│ │Subscribers  │ │    │ │ CAN Thread  │ │    │                 │
│ └─────────────┘ │    │ └─────────────┘ │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 模块组成
1. **manda_can_control**：核心控制节点
2. **manda_can_control**：消息定义包

### 数据流
```
外部控制命令 → ROS话题 → CAN控制节点 → CAN总线 → 车辆底盘
车辆状态数据 ← CAN总线 ← CAN控制节点 ← ROS话题 ← 外部监控
```

---

## 消息定义

### 反馈消息 (Feedback Messages)

#### 1. SpeedFb.msg - 速度反馈
```msg
float32 lf_speed    # 左前轮速度 (m/s)
float32 lr_speed    # 左后轮速度 (m/s)
float32 rf_speed    # 右前轮速度 (m/s)
float32 rr_speed    # 右后轮速度 (m/s)
```

#### 2. SteerFb.msg - 转向反馈
```msg
float32 lf_steer_angle    # 左前轮转向角 (度)
float32 lr_steer_angle    # 左后轮转向角 (度)
float32 rf_steer_angle    # 右前轮转向角 (度)
float32 rr_steer_angle    # 右后轮转向角 (度)
```

#### 3. BatteryFb.msg - 电池反馈
```msg
uint8 battery_soc        # 电池电量百分比 (0-100)
uint16 battery_voltage   # 电池电压 (V * 100)
int16 battery_current    # 电池电流 (A * 100)
```

#### 4. SystemstateFb.msg - 系统状态反馈
```msg
uint8 system_mode           # 系统模式
uint8 control_mode          # 控制模式
uint8 emergency_mode        # 紧急模式状态
uint8 obstacle_mode         # 障碍物模式状态
uint8 proximity_switch_mode # 接近开关模式状态
uint8 secure_edge_mode      # 安全触边模式状态
```

#### 5. MotionFb.msg - 运动反馈
```msg
float32 linear_x     # X方向线速度 (m/s)
float32 linear_y     # Y方向线速度 (m/s)
float32 angular_z    # Z方向角速度 (rad/s)
uint8   mode_type    # 模式类型
uint8   mode_switch  # 模式切换状态
```

#### 6. FaultFb.msg - 故障反馈
```msg
uint8 fault_code              # 故障代码
uint8 lf_motor_fault          # 左前电机故障
uint8 lr_motor_fault          # 左后电机故障
uint8 rf_motor_fault          # 右前电机故障
uint8 rr_motor_fault          # 右后电机故障
uint8 lf_steer_fault          # 左前转向故障
uint8 lr_steer_fault          # 左后转向故障
uint8 rf_steer_fault          # 右前转向故障
uint8 rr_steer_fault          # 右后转向故障
uint8 lf_motor_disconnect     # 左前电机断开
uint8 lr_motor_disconnect     # 左后电机断开
uint8 rf_motor_disconnect     # 右前电机断开
uint8 rr_motor_disconnect     # 右后电机断开
uint8 lf_steer_disconnect     # 左前转向断开
uint8 lr_steer_disconnect     # 左后转向断开
uint8 rf_steer_disconnect     # 右前转向断开
uint8 rr_steer_disconnect     # 右后转向断开
```

### 控制消息 (Control Messages)

### 服务定义 (Service Definitions)

#### 1. MotionMode.srv - 运动模式设置服务
```srv
char cmd_ctl    # 运动模式控制命令
---
char cmd_ack    # 命令确认响应
```

#### 2. DoCtrl.srv - 数字输出控制服务
```srv
char cmd_ctl    # 数字输出控制命令
---
char cmd_ack    # 命令确认响应
```

#### 1. SpeedCtrl.msg - 速度控制
```msg
float32 lf_speed    # 左前轮目标速度 (m/s)
float32 lr_speed    # 左后轮目标速度 (m/s)
float32 rf_speed    # 右前轮目标速度 (m/s)
float32 rr_speed    # 右后轮目标速度 (m/s)
```

#### 2. SteerCtrl.msg - 转向控制
```msg
float32 lf_steer_angle    # 左前轮目标转向角 (度)
float32 lr_steer_angle    # 左后轮目标转向角 (度)
float32 rf_steer_angle    # 右前轮目标转向角 (度)
float32 rr_steer_angle    # 右后轮目标转向角 (度)
```

#### 3. MotionCtrl.msg - 运动控制
```msg
float32 linear_x     # X方向目标线速度 (m/s)
float32 linear_y     # Y方向目标线速度 (m/s)
float32 angular_z    # Z方向目标角速度 (rad/s)
```

#### 4. ControlModeCtrl.msg - 控制模式
```msg
uint8 mode    # 控制模式值
```

#### 5. MotionModeCtrl.msg - 运动模式
```msg
uint8 mode    # 运动模式值
```

---

## API文档

### 话题 (Topics)

#### 发布话题 (Publishers)
| 话题名 | 消息类型 | 描述 | 频率 |
|--------|----------|------|------|
| `/speed_fb` | `manda_can_control::msg::SpeedFb` | 速度反馈 | 实时 |
| `/steer_fb` | `manda_can_control::msg::SteerFb` | 转向反馈 | 实时 |
| `/battery_fb` | `manda_can_control::msg::BatteryFb` | 电池反馈 | 实时 |
| `/system_state_fb` | `manda_can_control::msg::SystemstateFb` | 系统状态反馈 | 实时 |
| `/motion_fb` | `manda_can_control::msg::MotionFb` | 运动反馈 | 实时 |
| `/fault_fb` | `manda_can_control::msg::FaultFb` | 故障反馈 | 实时 |

#### 订阅话题 (Subscribers)
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/motion_control` | `manda_can_control::msg::MotionCtrl` | 运动控制命令 |
| `/speed_ctrl` | `manda_can_control::msg::SpeedCtrl` | 速度控制命令 |
| `/steer_ctrl` | `manda_can_control::msg::SteerCtrl` | 转向控制命令 |

#### 服务 (Services)
| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/motion_mode` | `manda_can_control::srv::MotionMode` | 运动模式设置服务 |
| `/do_ctrl` | `manda_can_control::srv::DoCtrl` | 数字输出控制服务 |

**服务使用方法**：

- **运动模式设置**：通过 `ros2 service call /motion_mode manda_can_control/srv/MotionMode "{cmd_ctl: 1}"` 调用

  运动模式功能类型如下

  | 0x01 | 反向模式                             |
  | ---- | :----------------------------------- |
  | 0x02 | 正向模式                             |
  | 0x04 | 自旋模式                             |
  | 0x08 | 用户自主控制模式（分别控制每个电机） |
  | 0x10 | 驻车模式                             |

  

- **数字输出控制**：通过 `ros2 service call /do_ctrl manda_can_control/srv/DoCtrl "{cmd_ctl: 1}"` 调用

- 服务返回 `cmd_ack` 字段表示命令执行状态

---

## 函数详细分析

### 类定义：MandaCanControl

#### 构造函数
```cpp
MandaCanControl::MandaCanControl():nh_("/")
```
**功能**：初始化CAN控制类
**参数**：无
**返回值**：无
**说明**：

- 创建ROS节点句柄，命名空间为"/"
- 初始化成员变量

#### 析构函数
```cpp
MandaCanControl::~MandaCanControl()
```
**功能**：清理资源
**参数**：无
**返回值**：无
**说明**：

- 释放CAN socket资源
- 停止接收线程

#### 主要运行函数
```cpp
void MandaCanControl::Run()
```
**功能**：启动CAN控制系统
**参数**：无
**返回值**：无
**执行流程**：

1. 调用 `InitRosNode()` 初始化ROS节点
2. 调用 `InitCanSocket()` 初始化CAN socket
3. 如果初始化失败，关闭ROS并返回
4. 创建CAN接收线程并使用detach()模式运行

**错误处理**：

- 如果ROS节点或CAN socket初始化失败，会调用 `ros::shutdown()` 关闭系统

**线程管理**：
- 使用 `std::thread` 创建CAN接收线程
- 采用 `detach()` 模式，避免主线程阻塞
- 线程在后台运行，主线程可以继续执行其他任务

### 初始化函数

#### ROS节点初始化
```cpp
int MandaCanControl::InitRosNode()
```
**功能**：初始化ROS发布者和订阅者
**参数**：无
**返回值**：
- `0`：成功
- 负值：失败

**详细实现**：
1. **发布者初始化**：
   - `wheel_fb_pub_`：发布速度反馈，话题名"speed_fb"，队列大小5
   - `steer_fb_pub_`：发布转向反馈，话题名"steer_fb"，队列大小5
   - `battery_fb_pub_`：发布电池反馈，话题名"battery_fb"，队列大小5
   - `system_state_fb_pub_`：发布系统状态反馈，话题名"system_state_fb"，队列大小5
   - `motion_fb_pb_`：发布运动反馈，话题名"motion_fb"，队列大小5

2. **订阅者初始化**：
   - `ctrl_cmd_sub_`：订阅运动控制，话题名"motion_control"，队列大小3
   - `ctrl_speed_sub_`：订阅速度控制，话题名"speed_ctrl"，队列大小3
   - `ctrl_steer_sub_`：订阅转向控制，话题名"steer_ctrl"，队列大小3

3. **服务初始化**：
   - `srv_motion_mode_`：运动模式设置服务，服务名"motion_mode"
   - `srv_do_sub_`：数字输出控制服务，服务名"do_ctrl"

#### CAN Socket初始化
```cpp
int MandaCanControl::InitCanSocket()
```
**功能**：初始化CAN总线socket连接
**参数**：无
**返回值**：
- `0`：成功
- `-1`：socket创建失败
- `-2`：CAN FD设置失败
- `-3`：CAN接口名称错误
- `-4`：接口索引映射失败
- `-5`：socket绑定失败

**详细实现**：
1. **创建CAN Socket**：
   ```cpp
   canSocketFd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)
   ```

2. **启用CAN FD支持**：
   
   ```cpp
   ::setsockopt(canSocketFd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &nEnable, sizeof(nEnable))
   ```
   
3. **配置CAN接口**：
   - 设置接口名称为"can0"
   - 获取接口索引
   - 配置socket地址

4. **绑定Socket**：
   ```cpp
   ::bind(canSocketFd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_))
   ```

### CAN通信函数

#### CAN接收线程
```cpp
void MandaCanControl::CanRecieveThread(void* ptr)
```
**功能**：CAN数据接收线程
**参数**：

- `ptr`：指向MandaCanControl实例的指针
**返回值**：无

**详细实现**：
1. **初始化文件描述符集合**：
   
   ```cpp
   fd_set fds;
   struct timeval timeout;
   ```
   
2. **主循环**：
   - 使用 `select()` 监听CAN socket
   - 检查是否有数据到达
   - 读取CAN帧数据

3. **数据处理**：
   - 根据帧长度判断是标准CAN帧还是CAN FD帧
   - 调用 `DecodeCanFrameData()` 解析数据

#### CAN帧数据解析
```cpp
void MandaCanControl::DecodeCanFrameData(const struct can_frame* frame)
```
**功能**：解析CAN帧数据并发布相应的ROS消息
**参数**：
- `frame`：指向CAN帧结构的指针
**返回值**：无

**支持的CAN ID和解析逻辑**：

1. **0x100 - 系统状态和电池信息**：
   ```cpp
   // 系统状态解析
   msg.system_mode = frame->data[0];
   msg.control_mode = frame->data[1];
   
   // 状态位解析
   int bits[4];
   for(int i = 0; i < 4; i++) {
       bits[i] = (frame->data[2] >> i) & 1;
   }
   msg.emergency_mode = bits[0];
   msg.obstacle_mode = bits[1];
   msg.proximity_switch_mode = bits[2];
   msg.secure_edge_mode = bits[3];
   
   // 电池信息解析
   msg_battery.battery_soc = frame->data[3];
               // 电池电压解析
            msg_battery.battery_voltage = (frame->data[4] << 8 | frame->data[5]);
            // 电池电流解析（修复字节序问题）
            uint8_t buf[2] = {frame->data[7], frame->data[6]};
            int16_t battery_current;
            memcpy(&battery_current, buf, 2);
            
            msg_battery.battery_current = battery_current;
   ```

2. **0x101 - 速度反馈**：
   
   ```cpp
   // 使用memcpy优化数据解析
   uint8_t buf[8] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4],frame->data[7],frame->data[6]};
   int16_t lf_speed, rf_speed, lr_speed, rr_speed;
   memcpy(&lf_speed, buf, 2);
   msg.lf_speed = (float)(lf_speed / 1000.0);
   memcpy(&lr_speed, buf + 2, 2);
   msg.lr_speed = (float)(lr_speed / 1000.0);
   memcpy(&rf_speed, buf + 4, 2);
   msg.rf_speed = (float)(rf_speed / 1000.0);
   memcpy(&rr_speed, buf + 6, 2);
   msg.rr_speed = (float)(rr_speed / 1000.0);
   ```
   
3. **0x102 - 转向反馈**：
   ```cpp
   // 使用memcpy优化数据解析
   uint8_t buf[8] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4],frame->data[7],frame->data[6]};
   int16_t lf_steer_angle, lr_steer_angle, rf_steer_angle, rr_steer_angle;
   memcpy(&lf_steer_angle, buf, 2);
   msg.lf_steer_angle = float(lf_steer_angle) / 100.0;
   memcpy(&lr_steer_angle, buf + 2, 2);
   msg.lr_steer_angle = float(lr_steer_angle) / 100.0;
   memcpy(&rf_steer_angle, buf + 4, 2);
   msg.rf_steer_angle = float(rf_steer_angle) / 100.0;
   memcpy(&rr_steer_angle, buf + 6, 2);
   msg.rr_steer_angle = float(rr_steer_angle) / 100.0;
   ```

4. **0x103 - 故障反馈**：
   ```cpp
   // 故障代码
   msg.fault_code = frame->data[0];
   
   // 电机故障位解析
   for(int i = 0; i < 4; i++) {
       bits_motor_alarm[i] = (frame->data[1] >> i) & 1;
   }
   
   // 转向故障位解析
   for(int i = 0; i < 4; i++) {
       bits_steer_alarm[i] = (frame->data[2] >> i) & 1;
   }
   
   // 连接状态位解析
   for(int i = 0; i < 4; i++) {
       bits_motor_connect[i] = (frame->data[3] >> i) & 1;
   }
   for(int i = 0; i < 4; i++) {
       bits_steer_connect[i] = (frame->data[4] >> i) & 1;
   }
   ```

5. **0x104 - 运动反馈**：
   ```cpp
   // 使用memcpy优化数据解析
   int16_t linear_x, linear_y, angular_z;
   uint8_t buf[6] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4]};
   memcpy(&linear_x, buf, 2);
   msg.linear_x = float(linear_x) / 1000.0;
   memcpy(&linear_y, buf + 2, 2);
   msg.linear_y = float(linear_y) / 1000.0;
   memcpy(&angular_z, buf + 4, 2);
   msg.angular_z = float(angular_z) / 1000.0;
   msg.mode_type = frame->data[6];    // 新增：模式类型
   msg.mode_switch = frame->data[7];
   ```

#### CAN帧发送
```cpp
int MandaCanControl::SendCanFrame(canid_t id, uint8_t *data, uint32_t frame_num)
```
**功能**：发送CAN帧数据
**参数**：

- `id`：CAN帧ID
- `data`：数据指针
- `frame_num`：帧数量
**返回值**：
- `0`：成功
- `-1`：发送失败

**详细实现**：
1. **初始化CAN帧结构**：
   
   ```cpp
   struct can_frame frame;
   memset(&frame, 0, sizeof(frame));
   frame.can_id = id;
   frame.can_dlc = 8;
   ```
   
2. **发送数据**：
   ```cpp
   for (uint32_t i = 0; i < frame_num; i++) {
       for (uint32_t j = 0; j < frame.can_dlc; j++)
           frame.data[j] = data[(i * frame.can_dlc) + j];
       if (write(canSocketFd_, &frame, sizeof(frame)) != sizeof(frame)) {
           ROS_INFO("Can socket write error");
           return -1;
       }
   }
   ```

### 回调函数

#### 运动模式服务回调
```cpp
void MotionModeCallback(
    const std::shared_ptr<manda_can_control::srv::MotionMode::Request> req,
    std::shared_ptr<manda_can_control::srv::MotionMode::Response> res)
```
**功能**：处理运动模式设置服务请求
**参数**：

- `req`：运动模式请求
- `res`：运动模式响应
**返回值**：`true`表示服务处理成功
**实现**：
```cpp
uint8_t data = (uint8_t)req.cmd_ctl;
uint8_t nData[1] = {data};
SendCanFrame(0x401, nData, 1);
res.cmd_ack = CMD_ACK_FINISH;
return true;
```

#### 运动控制回调
```cpp
void MotionControlCallback(const manda_can_control::msg::MotionCtrl::SharedPtr motion)
```
**功能**：处理运动控制命令
**参数**：

- `motion`：运动控制消息
**返回值**：无
**实现**：
```cpp
uint8_t nData[8];
int16_t linear_x = int16_t(motion.linear_x * 1000);
int16_t linear_y = int16_t(motion.linear_y * 1000);
int16_t angular_z = int16_t(motion.angular_z * 1000);

nData[0] = (linear_x >> 8) & 0xff;
nData[1] = linear_x & 0xff;
nData[2] = (linear_y >> 8) & 0xff;
nData[3] = linear_y & 0xff;
nData[4] = (angular_z >> 8) & 0xff;
nData[5] = angular_z & 0xff;
nData[6] = 0;
nData[7] = 0;
SendCanFrame(0x402, nData, 1);
```

#### 速度控制回调
```cpp
void MandaCanControl::SpeedControlCallback(const manda_can_control::msg::SpeedCtrl::SharedPtr speed)
```
**功能**：处理速度控制命令
**参数**：

- `speed`：速度控制消息
**返回值**：无
**实现**：
```cpp
uint8_t nData[8];
int16_t lf_speed = int16_t(speed.lf_speed * 1000);
int16_t lr_speed = int16_t(speed.lr_speed * 1000);
int16_t rf_speed = int16_t(speed.rf_speed * 1000);
int16_t rr_speed = int16_t(speed.rr_speed * 1000);

nData[0] = (lf_speed >> 8) & 0xff;
nData[1] = lf_speed & 0xff;
nData[2] = (lr_speed >> 8) & 0xff;
nData[3] = lr_speed & 0xff;
nData[4] = (rf_speed >> 8) & 0xff;
nData[5] = rf_speed & 0xff;
nData[6] = (rr_speed >> 8) & 0xff;
nData[7] = rr_speed & 0xff;
SendCanFrame(0x403, nData, 1);
```

#### 转向控制回调
```cpp
void MandaCanControl::SteerControlCallback(const manda_can_control::msg::SteerCtrl::SharedPtr angle)
```
**功能**：处理转向控制命令
**参数**：
- `angle`：转向控制消息
**返回值**：无
**实现**：
```cpp
uint8_t nData[8];
int16_t lf_steer_angle = int16_t(angle.lf_steer_angle * 100);
int16_t lr_steer_angle = int16_t(angle.lr_steer_angle * 100);
int16_t rf_steer_angle = int16_t(angle.rf_steer_angle * 100);
int16_t rr_steer_angle = int16_t(angle.rr_steer_angle * 100);

nData[0] = (lf_steer_angle >> 8) & 0xff;
nData[1] = lf_steer_angle & 0xff;
nData[2] = (lr_steer_angle >> 8) & 0xff;
nData[3] = lr_steer_angle & 0xff;
nData[4] = (rf_steer_angle >> 8) & 0xff;
nData[5] = rf_steer_angle & 0xff;
nData[6] = (rr_steer_angle >> 8) & 0xff;
nData[7] = rr_steer_angle & 0xff;
SendCanFrame(0x404, nData, 1);
```

#### 数字输出控制服务回调
```cpp
bool MandaCanControl::DoCallback(const std::shared_ptr<manda_can_control::srv::DoCtrl::Request> req,
    std::shared_ptr<manda_can_control::srv::DoCtrl::Response> res)
```
**功能**：处理数字输出控制服务请求
**参数**：
- `req`：数字输出控制请求
- `res`：数字输出控制响应
**返回值**：`true`表示服务处理成功
**实现**：
```cpp
uint8_t data = (uint8_t)req.cmd_ctl;
uint8_t nData[8] = {data, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
SendCanFrame(0x405, nData, 1);
res.cmd_ack = CMD_ACK_FINISH;
return true;
```

### 主函数
```cpp
int main(int argc, char **argv)
```
**功能**：程序入口点
**参数**：
- `argc`：命令行参数数量
- `argv`：命令行参数数组
**返回值**：
- `0`：正常退出
**实现**：
```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<MandaCanControl>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
```

---

## CAN协议规范

### CAN帧格式
- **标准帧长度**：8字节
- **CAN ID范围**：0x100 - 0x405
- **波特率**：500kbps（默认）

### CAN ID分配

#### 接收ID (0x100-0x104)
| ID | 功能 | 数据长度 | 描述 |
|----|------|----------|------|
| 0x100 | 系统状态+电池 | 8字节 | 系统模式、控制模式、状态位、电池信息 |
| 0x101 | 速度反馈 | 8字节 | 四轮速度反馈 |
| 0x102 | 转向反馈 | 8字节 | 四轮转向角度反馈 |
| 0x103 | 故障反馈 | 8字节 | 故障代码、电机故障、转向故障、连接状态 |
| 0x104 | 运动反馈 | 8字节 | 线速度、角速度、模式切换完成标志 |

#### 发送ID (0x400-0x405)
| ID | 功能 | 数据长度 | 描述 |
|----|------|----------|------|
| 0x400 | 控制模式 | 1字节 | 设置控制模式 |
| 0x401 | 运动模式 | 1字节 | 设置运动模式 |
| 0x402 | 运动控制 | 8字节 | 线速度、角速度控制 |
| 0x403 | 速度控制 | 8字节 | 四轮速度控制 |
| 0x404 | 转向控制 | 8字节 | 四轮转向角度控制 |
| 0x405 | 数字输出控制 | 8字节 | 数字输出控制命令 |

### 数据格式规范

#### 速度数据格式
- **单位**：m/s
- **精度**：0.001 m/s
- **编码**：16位有符号整数，除以1000

#### 转向角度格式
- **单位**：度
- **精度**：0.01度
- **编码**：16位有符号整数，除以100

#### 电池电压格式
- **单位**：V
- **精度**：0.01V
- **编码**：16位无符号整数，除以100

#### 电池电流格式
- **单位**：A
- **精度**：0.01A
- **编码**：16位有符号整数，除以100

---

## 编译与部署

### 依赖项
本包为 ROS2 Foxy 版本，主要依赖：

- `rclcpp`
- `ament_cmake`
- `rosidl_default_generators`

### 编译步骤
1. **创建工作空间**：
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **复制项目文件**：
   
   ```bash
   cp -r manda_can_control ./
   ```
   
3. **编译项目**：
   
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```
   
4. **设置环境**：
   ```bash
   source install/setup.bash
   ```

### 运行步骤
1. **配置CAN接口**：
   
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0
   ```
   
2. **运行节点**：
   
   ```bash
   ros2 run manda_can_control manda_can_control_node
   ```


### 测试命令
```bash
# 发布速度控制命令
ros2 topic pub /speed_ctrl manda_can_control/msg/SpeedCtrl "{lf_speed: 1.0, lr_speed: 1.0, rf_speed: 1.0, rr_speed: 1.0}" -1

# 循环发布速度控制命令
ros2 topic pub /speed_ctrl manda_can_control/msg/SpeedCtrl "{lf_speed: 1.0, lr_speed: 1.0, rf_speed: 1.0, rr_speed: 1.0}" -r 10

# 发布转向控制命令
ros2 topic pub /steer_ctrl manda_can_control/msg/SteerCtrl "{lf_steer_angle: 0.0, lr_steer_angle: 0.0, rf_steer_angle: 0.0, rr_steer_angle: 0.0}" -1

#循环发布转向控制命令
ros2 topic pub /steer_ctrl manda_can_control/msg/SteerCtrl "{lf_steer_angle: 0.0, lr_steer_angle: 0.0, rf_steer_angle: 0.0, rr_steer_angle: 0.0}" -r 10

# 发布运动控制命令
ros2 topic pub /motion_control manda_can_control/msg/MotionCtrl "{linear_x: 1.0, linear_y: 0.0, angular_z: 0.0}" -1

# 循环发布运动控制命令
ros2 topic pub /motion_control manda_can_control/msg/MotionCtrl "{linear_x: 1.0, linear_y: 0.0, angular_z: 0.0}" -r 10

# 调用运动模式设置服务
ros2 service call /motion_mode manda_can_control/srv/MotionMode "{cmd_ctl: 1}"

# 调用数字输出控制服务
ros2 service call /do_ctrl manda_can_control/srv/DoCtrl "{cmd_ctl: 1}"

# 监听速度反馈
ros2 topic echo /speed_fb

# 监听系统状态
ros2 topic echo /system_state_fb

# 监听运动反馈
ros2 topic echo /motion_fb
```


## 错误处理

### 常见错误及解决方案
**错误信息**：`CAN socket open error`
**原因**：权限不足或CAN接口未配置
**解决方案**：

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

#### 2. CAN接口不存在
**错误信息**：`If_nametoindex CAN name error`
**原因**：CAN接口名称错误或接口未创建
**解决方案**：

```bash
# 检查CAN接口
ip link show

# 创建CAN接口
sudo modprobe can
sudo modprobe can_raw
```

#### 3. 权限错误
**错误信息**：`Bind CAN socket error`
**原因**：用户权限不足
**解决方案**：

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启
```

#### 4. ROS话题连接失败
**错误信息**：话题无数据
**原因**：话题名称不匹配或节点未启动
**解决方案**：

```bash
# 检查话题列表
ros2 topic list

# 检查话题信息
ros2 topic info /speed_fb
```

### 调试技巧
1. **启用ROS调试信息**：
   
   ```bash
   export ROS_LOG_LEVEL=DEBUG
   ```
   
2. **使用CAN工具调试**：
   
   ```bash
   # 安装can-utils
   sudo apt-get install can-utils
   
   # 监听CAN数据
   candump can0
   
   # 发送CAN数据
   cansend can0 100#1234567890ABCDEF
   ```
   
3. **检查系统日志**：
   
   ```bash
   dmesg | grep can
   ```



---

## 故障排除

### 常见问题

#### Q1: 节点启动后无法接收CAN数据
**A1**: 检查以下几点：
1. CAN接口是否正确配置
2. 硬件连接是否正常
3. 波特率是否匹配
4. 权限是否足够

#### Q2: 发送控制命令后车辆无响应
**A2**: 检查以下几点：
1. CAN ID是否正确
2. 数据格式是否符合协议
3. 车辆是否处于正确模式
4. 安全开关是否激活

#### Q3: 系统运行一段时间后崩溃
**A3**: 检查以下几点：
1. 内存泄漏
2. 线程死锁
3. 资源耗尽
4. 异常处理

#### Q4: 数据延迟过高
**A4**: 优化建议：
1. 减少数据处理时间
2. 优化线程调度
3. 使用更高效的算法
4. 调整缓冲区大小

### 调试工具
1. **rqt_graph**：查看节点连接关系
2. **rqt_plot**：实时绘制数据曲线
3. **ros2 topic hz**：检查话题频率
4. **rqt_console**：查看日志信息

### 性能监控
```bash
# 监控CPU使用率
top -p $(pgrep manda_can_control_node)

# 监控内存使用
ps aux | grep manda_can_control_node

# 监控CAN接口状态
cat /proc/net/can/stats
```

---

## 版本历史

### v1.1.1 (当前版本)

- 更改ROS2

### v1.1.0 

- 重构了运动模式控制，从话题订阅改为服务调用
- 移除了控制模式话题订阅功能
- 添加了数字输出控制服务
- 优化了CAN数据解析，使用memcpy提高性能
- 修复了电池电流解析的字节序问题
- 添加了新的CAN ID 0x405用于数字输出控制
- 改进了线程管理，使用detach()替代join()

### v1.0.0 (初始版本)
- 初始版本发布
- 支持基本的CAN通信功能
- 实现四轮控制和状态监控
- 提供ROS接口

---

## 联系信息

**维护者**：fengql  
**邮箱**：jianhouxifeng1222@gmail.com  
**许可证**：TODO

---

*本文档最后更新时间：2026年1月4日*