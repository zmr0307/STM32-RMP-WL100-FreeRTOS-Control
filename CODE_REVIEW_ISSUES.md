# 代码审核问题清单 (2026-03-26 更新)

> 审核范围：FreeRTOS-can 项目全部核心源代码（~2500行）  
> 审核结论：架构分层清晰，安全机制完善。以下为已修复和待处理的问题。

---

## ✅ 已修复 (历次审核中已处理)

### P0-1. `configASSERT` 在中断中触发会死锁 → ✅ 已修复
- **修复**: 替换为纯硬件死循环（`__disable_irq()` + LED0 闪烁），不依赖信号量

### P0-2. `chassis_ctrl_task` 临界区用 `__disable_irq()` → ✅ 已修复
- **修复**: 改为 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`

### P0-3. Jetson 断线后速度不归零（飞车风险）→ ✅ 已修复
- **修复**: `jetson_usart.c` 解码成功时记录 `HAL_GetTick()` 时间戳，`chassis_ctrl_task` 每轮检测 500ms 超时强制归零

### P1-3. `HardFault_Handler` 无诊断信息 → ✅ 已修复
- **修复**: LED1 快速闪烁(~200ms)，与心跳 LED0(500ms) 区分

### P1-4. `USART3_IRQHandler` ORE 清除 → ✅ 已修复
- **修复**: 在 `HAL_UART_IRQHandler` 前手动清除 ORE 标志

### P1-5. 全局变量未加 `static` → ✅ 已修复
- **修复**: `chassis_test.c` 中 6 个全局变量加 `static` 修饰

### P1-6. `MemManage/BusFault/UsageFault` 无诊断 → ✅ 已修复
- **修复**: LED0+LED1 同步快闪(~100ms)，与 HardFault 的 LED1 独闪区分

### P1-7. 0x104 丢弃运动模式字段 → ✅ 已修复
- **修复**: `chassis_protocol.h` 新增 `motion_mode`/`mode_switch`，`chassis_driver.c` 补全 data[6]/data[7] 解码

### P2-1. `safe_printf` 调度器启动前调用 → ✅ 已修复
- **修复**: 加 `xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED` 判断

### P2-2. `AI_CONTEXT.md` HSE 值写错 → ✅ 已修复
- **修复**: 25MHz → 8MHz

---

## ⏳ P2 - 后续扩展时处理

### 1. `jetson_protocol_decode()` 在 ISR 中执行

- **文件**: `jetson_usart.c`，`USART3_IRQHandler` → `jetson_protocol_decode()`
- **现状**: 100字节缓冲区滑窗搜帧，耗时 ~10µs，当前完全可接受
- **隐患**: 若未来加大缓冲区或增加帧类型，ISR 耗时增长可能导致 CAN 丢帧
- **后续方案**: 重构为「ISR 只搬运原始数据入队列，任务中解析」

### 2. `g_chassis_state` 跨任务读写无同步保护

- **文件**: `chassis_driver.c` 中的 `g_chassis_state`
- **现状**: `chassis_rx_task`(prio=3) 写入，`chassis_ctrl_task`(prio=2) 读取，无互斥
- **隐患**: 当前读取的是 `uint8_t` 单字节字段（ARM原子操作），不会出问题。但若未来需要多字段联合判断可能读到不一致状态
- **后续方案**: 扩展时给读写操作加 `taskENTER_CRITICAL()` 保护

### 3. 缺少 IWDG 硬件看门狗

- **隐患**: 若系统死锁（非 HardFault），无法自动恢复
- **后续方案**: 在 `led_task` 中喂狗，设置 2~4 秒超时

### 4. CAN Bus-Off 无恢复机制

- **现状**: `AutoBusOff = DISABLE`，Bus-Off 后需手动恢复
- **后续方案**: 设置 `AutoBusOff = ENABLE` 或在 Error 回调中重新初始化

### 5. `can_send_msg()` 未来扩展风险

- **现状**: 当前仅 `chassis_ctrl_task` 调用，无竞态
- **隐患**: 若未来新增 CAN 发送任务，全局 `g_canx_txheader` 存在竞态
- **后续方案**: 添加互斥锁或改用局部变量

---

## ✅ 已确认正确的设计

以下为审核中**确认无问题**的关键设计决策，记录在此防止误改：

| 设计 | 确认结论 |
|------|---------|
| `start_task` 中 `vTaskDelete` 在 `taskENTER/EXIT_CRITICAL` 之间 | ✅ 正确，PendSV 在临界区退出后才触发上下文切换 |
| 电流大端解码 `(data[6]<<8)\|data[7]` | ✅ 实车验证正确（-560mA），非小端 |
| 0x402 发包 50Hz（`vTaskDelay(20)`） | ✅ 100Hz 曾导致 CANERR，50Hz 稳定 |
| CAN 发送超时改为 `0x7FFFF` | ✅ 修复正点原子 BUG，覆盖帧传输时间 |
| `safe_printf` 互斥锁宏替换 | ✅ 多任务并发打印安全 |
| DMA CIRCULAR + IDLE 中断接收引擎 | ✅ 架构合理，热插拔测试通过 |
| 速度限幅 ±2.0 m/s | ✅ 防飞车安全机制 |
| Jetson 断线 500ms 超时归零 | ✅ 上电初始安全（last_valid_tick=0 立即满足超时）|

