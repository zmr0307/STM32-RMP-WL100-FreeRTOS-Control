/**
 ****************************************************************************************************
 * @file        chassis_test.c
 * @brief       Chassis driver test program (loopback mode)
 * @note        Test platform: Alientek Explorer F407 board
 *              Test content:
 *              1. Encoding test: Send control commands, verify CAN data encoding
 *              2. Decoding test: Simulate chassis feedback, verify data decoding
 *              3. Integration test: Complete send-receive flow
 ****************************************************************************************************
 */

#include "chassis_test.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/CHASSIS/chassis_driver.h"
#include "./BSP/JETSON/jetson_usart.h"

/* 引用 JETSON 驱动中的 ORE 错误计数器 */
extern uint32_t g_uart3_ore_cnt;

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/******************************************************************************************************/
/*                                    IWDG 硬件看门狗                                                  */
/******************************************************************************************************/

static IWDG_HandleTypeDef g_iwdg_handle;  /* IWDG 句柄 */

/**
 * @brief       初始化独立看门狗 (4秒超时)
 * @note        LSI ≈ 32KHz, 预分频=64, 重装载=2000
 *              超时 = 64 × 2000 / 32000 = 4秒
 *              调试时自动冻结，防止断点触发复位
 */
static void iwdg_init(void)
{
    g_iwdg_handle.Instance = IWDG;
    g_iwdg_handle.Init.Prescaler = IWDG_PRESCALER_64;  /* 32KHz/64 = 500Hz */
    g_iwdg_handle.Init.Reload = 2000;                  /* 2000/500 = 4秒 */
    HAL_IWDG_Init(&g_iwdg_handle);

    __HAL_DBGMCU_FREEZE_IWDG();  /* ST-Link 调试时冻结看门狗 */
}

/******************************************************************************************************/
/*                                    FreeRTOS Tasks                                                   */
/******************************************************************************************************/

/* ============ Start Task ============ */
#define START_TASK_PRIO     1
#define START_STK_SIZE      128
static TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

/* ============ Chassis Control Test Task ============ */
#define CHASSIS_CTRL_TASK_PRIO    2
#define CHASSIS_CTRL_STK_SIZE     512
static TaskHandle_t Chassis_Ctrl_Task_Handler;
void chassis_ctrl_task(void *pvParameters);

/* ============ Chassis Feedback Processing Task ============ */
#define CHASSIS_RX_TASK_PRIO      3
#define CHASSIS_RX_STK_SIZE       512
static TaskHandle_t Chassis_RX_Task_Handler;
void chassis_rx_task(void *pvParameters);

/* ============ LED Blink Task ============ */
#define LED_TASK_PRIO       1
#define LED_STK_SIZE        128
static TaskHandle_t LED_Task_Handler;
void led_task(void *pvParameters);

/* ============ CAN接收消息队列 ============ */
#define CAN_RX_QUEUE_LEN    10
static QueueHandle_t g_can_rx_queue;

/* ============ Printf 互斥锁 ============ */
#include <stdarg.h>
#include <stdio.h>
#include "semphr.h"

static SemaphoreHandle_t g_printf_mutex = NULL;

void safe_printf(const char *format, ...)
{
    /* P2: 调度器未启动时跳过信号量操作，避免未定义行为 */
    uint8_t need_lock = (g_printf_mutex != NULL &&
                         xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
    if (need_lock)
    {
        xSemaphoreTake(g_printf_mutex, portMAX_DELAY);
    }
    
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    if (need_lock)
    {
        xSemaphoreGive(g_printf_mutex);
    }
}
#define printf(...) safe_printf(__VA_ARGS__)

/******************************************************************************************************/
/*                              FreeRTOS 栈溢出报警钩子                                               */
/******************************************************************************************************/

/**
 * @brief  FreeRTOS 栈溢出检测回调（configCHECK_FOR_STACK_OVERFLOW = 2 时生效）
 * @note   任何任务栈溢出时，系统会自动调用此函数。LED 疯狂闪烁 + LCD 红字报警。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* LCD 显示崩溃信息 */
    lcd_show_string(10, 280, 240, 16, 16, "!!! STACK OVERFLOW !!!", RED);
    
    /* LED0 疯狂闪烁示警，方便物理观察 */
    while (1)
    {
        LED0_TOGGLE();
        for (volatile uint32_t i = 0; i < 500000; i++);  /* 粗略延时，此时调度器已死 */
    }
}

/******************************************************************************************************/
/*                                    主函数                                                          */
/******************************************************************************************************/

/**
 * @brief  底盘驱动测试主函数
 */
void chassis_test_demo(void)
{
    /* 创建 printf 的防抢占互斥锁 */
    g_printf_mutex = xSemaphoreCreateMutex();

    /* LCD 显示标题 */
    lcd_show_string(10, 10, 220, 32, 32, "STM32", RED);
    lcd_show_string(10, 47, 220, 24, 24, "Chassis Test", RED);
    lcd_show_string(10, 76, 220, 16, 16, "ATOM@ALIENTEK", RED);
    lcd_show_string(10, 100, 220, 16, 16, "CAN NORMAL Mode", BLUE);

    printf("\r\n========================================\r\n");
    printf("Chassis Driver Test Program Start\r\n");
    printf("========================================\r\n");

    /* ① 初始化底盘驱动（0 = 开启正常 CAN 模式连接实车，不再回环） */
    if (chassis_driver_init(0) == CHASSIS_OK)
    {
        printf("[Init] Chassis driver init success (NORMAL mode)\r\n");
    }
    else
    {
        printf("[Init] Chassis driver init failed!\r\n");
        while (1);
    }

    /* 🔌 初始化高速接收引擎 (115200波特率, 接Jetson/电脑USB) */
    jetson_usart_init(115200);
    printf("[Init] JETSON USART3 DMA-IDLE Engine Started.\r\n");

    /* ② 创建CAN接收消息队列 */
    g_can_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_RxData_t));
    if (g_can_rx_queue == NULL)
    {
        printf("[Init] Create CAN RX queue failed!\r\n");
        while (1);
    }
    printf("[Init] CAN RX queue created, capacity=%d\r\n", CAN_RX_QUEUE_LEN);

    /* ③ 设置队列句柄给CAN驱动（中断会往队列发数据） */
    can_set_rx_queue(g_can_rx_queue);

    /* ④ 初始化 IWDG 硬件看门狗 (4秒超时, 由 led_task 喂狗) */
    iwdg_init();
    printf("[Init] IWDG Watchdog Started (4s timeout)\r\n");

    /* ⑤ 创建启动任务 */
    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&StartTask_Handler);

    /* ⑥ 启动调度器 */
    vTaskStartScheduler();
}

/******************************************************************************************************/
/*                                    任务实现                                                      */
/******************************************************************************************************/

/**
 * @brief  启动任务：创建所有工作任务然后自杀
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    /* 创建 底盘控制测试任务 */
    xTaskCreate((TaskFunction_t )chassis_ctrl_task,
                (const char*    )"chassis_ctrl_task",
                (uint16_t       )CHASSIS_CTRL_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CHASSIS_CTRL_TASK_PRIO,
                (TaskHandle_t*  )&Chassis_Ctrl_Task_Handler);

    /* 创建 底盘反馈处理任务 */
    xTaskCreate((TaskFunction_t )chassis_rx_task,
                (const char*    )"chassis_rx_task",
                (uint16_t       )CHASSIS_RX_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CHASSIS_RX_TASK_PRIO,
                (TaskHandle_t*  )&Chassis_RX_Task_Handler);

    /* 创建 LED 闪烁任务 */
    xTaskCreate((TaskFunction_t )led_task,
                (const char*    )"led_task",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_TASK_PRIO,
                (TaskHandle_t*  )&LED_Task_Handler);

    vTaskDelete(StartTask_Handler);     /* 删除自己 */
    taskEXIT_CRITICAL();
}

/**
 * @brief  底盘安全全向微操任务 (从 Jetson 映射)
 * @note   利用底座内置 0x402 全向平滑算法，实时拉取串口指令。
 */
void chassis_ctrl_task(void *pvParameters)
{
    chassis_err_e ret;
    chassis_motion_ctrl_t motion_cmd;
    jetson_ctrl_cmd_t *jetson_data;

    /* 初始锁定发送速度为 0 */
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_vz = 0.0f;

    /* 故障安全拦截变量 */
    uint32_t fault_start_tick = 0;  /* 驱动器故障开始时间戳 */
    uint8_t  fault_parked = 0;     /* 是否已切入驻车模式 */

    printf("\r\n>>> Chassis JETSON-TELEOP Ready <<<\r\n");

    vTaskDelay(1000);  

    /* ========== Test3: OMNI RealTime Loop (10ms高速周期) + 自愈重连系统 ========== */
    printf("\r\n[Test3] Starting Realtime Serial Loop with Self-Healing...\r\n");

    while (1)
    {
        /* 缓存底盘状态快照, 避免同一轮循环内多次读取被抢占更新导致不一致 */
        uint8_t cached_online = chassis_is_online();
        uint8_t cached_ctrl_mode = chassis_get_state()->system_status.ctrl_mode;

        /* ---【系统级自愈保护：心跳与模式监测】---
         * 不断嗅探车身原厂底座反馈回来的 0x100 状态里的控制位 和 在线心跳
         * 如果底盘刚上电或者因为一些干扰掉出了 CAN 控制模式，立刻强杀所有的速度下发，切入重连挽救流程！
         *
         * 重要：底盘 0x100 反馈帧中 ctrl_mode 的含义：
         *   0x00 = 待机模式
         *   0x01 = 遥控器模式  
         *   0x02 = CAN指令控制模式 (我们需要达到的目标状态)
         * 注意: 发送 0x400 使能时用的 CTRL_MODE_CAN=0x01是控制命令的参数，
         *           与反馈帧中返回的状态码含义不同！不能直接混用！
         */
        if (cached_online == 0 || cached_ctrl_mode != CTRL_MODE_FB_CAN)
        {
            /* 1. 强制锁死目标车速为 0 */
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;

            if (cached_online == 0)
            {
                lcd_show_string(10, 130, 240, 16, 16, "Status: OFFLINE...     ", RED);
                lcd_show_string(10, 150, 240, 16, 16, "Mode: OFFLINE        ", RED);
            }
            else if (cached_ctrl_mode == CTRL_MODE_FB_REMOTE)
            {
                lcd_show_string(10, 130, 240, 16, 16, "Status: CAN ONLINE   ", GREEN);
                lcd_show_string(10, 150, 240, 16, 16, "Mode: REMOTE CTRL   ", YELLOW);
            }
            else
            {
                lcd_show_string(10, 130, 240, 16, 16, "Status: Reconnecting...", MAGENTA);
                lcd_show_string(10, 150, 240, 16, 16, "Mode: STANDBY        ", RED);
            }

            /* 2. 根据模式决定是否抢权 */
            if (cached_ctrl_mode == CTRL_MODE_FB_REMOTE)
            {
                /* 遥控器控制中：不发 0x400 抢权，等遥控器松手后自动接管 */
            }
            else
            {
                /* 离线/待机：发送 CAN 夺权使能 + 运动模式 */
                ret = chassis_send_ctrl_enable(CTRL_MODE_CAN);
                if (ret == CHASSIS_OK)
                {
                    chassis_send_motion_mode(MOTION_MODE_FORWARD);
                }
            }

            /* 既然在抢修通讯期间，就不要高频发包耗电了，休息一会等车子反应过来再战 */
            vTaskDelay(500);
            
            /* 抢修期跳过后续所有打控制指令的操作，直接下一把 */
            continue; 
        }
        else
        {
            /* 车子已经乖乖在手底下听话了，我们显示一点安心的绿色 */
            lcd_show_string(10, 130, 240, 16, 16, "Status: CAN ONLINE   ", GREEN);
            lcd_show_string(10, 150, 240, 16, 16, "Mode: JETSON CMD     ", BLUE);
        }

        /* ---【常规业务：提取最新串口高速缓存】--- */
        jetson_data = get_jetson_cmd_ptr();
        
        /* 临界区保护：防止 USART3 中断在读取途中写入新值导致半帧撕裂 */
        taskENTER_CRITICAL();
        if (jetson_data->frame_valid)
        {
            /* 有合法新帧到达！取出大端序收到的 1000 倍率整型，除以 1000f 变回实际浮点数交接 */
            target_vx = (float)(jetson_data->target_vx) / 1000.0f;
            target_vy = (float)(jetson_data->target_vy) / 1000.0f;
            target_vz = (float)(jetson_data->target_vz) / 1000.0f;
            
            /* 清除标志位，等下一包 */
            jetson_data->frame_valid = 0;
        }
        taskEXIT_CRITICAL();

        /* P0: Jetson 通信超时保护 - 500ms无新帧则强制归零，防断线飞车 */
        if (HAL_GetTick() - jetson_data->last_valid_tick > 500)
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;
        }

        /* ❗ 故障/急停安全拦截（优先级高于一切运动指令）*/
        if (chassis_is_estop())
        {
            /* 急停：硬件已断电，仅零速度 + 报警 */
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;
            lcd_show_string(10, 170, 240, 16, 16, "Ctrl: E-STOP!!! ", RED);
            fault_start_tick = 0;
            fault_parked = 0;
        }
        else if (chassis_has_fault())
        {
            /* 驱动器故障：零速度 → 3秒后切驻车 */
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;

            if (fault_start_tick == 0)
            {
                fault_start_tick = HAL_GetTick();  /* 记录故障开始时刻 */
            }

            if (!fault_parked && (HAL_GetTick() - fault_start_tick > 3000))
            {
                /* 故障持续超过3秒，切驻车模式（车轮内八锁死） */
                chassis_send_motion_mode(MOTION_MODE_PARK);
                fault_parked = 1;
                lcd_show_string(10, 170, 240, 16, 16, "Ctrl: PARKED!!! ", RED);
            }
            else if (!fault_parked)
            {
                lcd_show_string(10, 170, 240, 16, 16, "Ctrl: FAULT!!!  ", RED);
            }
        }
        else
        {
            /* 无故障：故障恢复处理 + 正常LCD显示 */
            if (fault_parked)
            {
                /* 从驻车恢复：重新切回FORWARD模式 */
                chassis_send_motion_mode(MOTION_MODE_FORWARD);
                fault_parked = 0;
            }
            fault_start_tick = 0;

            if (target_vx != 0.0f || target_vy != 0.0f || target_vz != 0.0f)
            {
                lcd_show_string(10, 170, 240, 16, 16, "Ctrl: JETSON CMD", MAGENTA);
            }
            else
            {
                /* 收到全0指令 (刹车) */
                lcd_show_string(10, 170, 240, 16, 16, "Ctrl: STOP CMD  ", BLUE);
            }
        }

        /* 灌递给平滑处理黑盒 (软限幅在函数内部会自动生效) */
        motion_cmd.vx = target_vx;
        motion_cmd.vy = target_vy;
        motion_cmd.vz = target_vz;

        chassis_err_e err = chassis_send_motion_ctrl(&motion_cmd);

        if (err != CHASSIS_OK)
        {
            lcd_show_string(10, 170, 240, 16, 16, "Ctrl: CANERR    ", RED);
        }

        /* 降频发包以维持底盘连贯动作响应，50Hz防爆箱 (20ms级较优) */
        vTaskDelay(20);  
    }
}

/**
 * @brief  底盘反馈处理任务
 * @note   从队列读取CAN数据，调用底盘驱动解码函数
 */
void chassis_rx_task(void *pvParameters)
{
    CAN_RxData_t rx_data;
    BaseType_t ret;
    chassis_state_t *state;

    printf("\r\n>>> Chassis Feedback Processing Task Start\r\n");

    while (1)
    {
        /* 从队列等待接收数据（最多等200ms，超时也要刷新在线状态） */
        ret = xQueueReceive(g_can_rx_queue, &rx_data, pdMS_TO_TICKS(200));

        if (ret == pdTRUE)
        {
            /* 为避免刷屏干扰仪表盘，屏蔽接收到的原始十六进制数据 printf */

            /* 调用底盘驱动处理函数（解码暂存至全局 state） */
            chassis_process_feedback(rx_data.id, rx_data.data, rx_data.len);

            /* 获取底盘状态 */
            state = chassis_get_state();

            /* 根据ID打印解码后的数据 */
            switch (rx_data.id)
            {
                case CAN_ID_SYSTEM_STATUS:  /* 借用 0x100 每 1000ms才触发一次的特性，实现整洁看板打印与LCD刷屏 */
                    {
                        char lcd_buf[50];
                        
                        /* 1. 串口打印 */
                        printf("\r\n--- CHASSIS STATUS DASHBOARD ---\r\n");
                        printf("Voltage : %5dmV | Current: %5dmA | SOC: %3d%%\r\n",
                                state->system_status.battery_voltage * 10,
                                state->system_status.battery_current * 10,
                                state->system_status.soc);
                        printf("State   : %d       | Periph : 0x%02X   | Mode: %d\r\n",
                                state->system_status.system_state,
                                state->system_status.peripheral_state,
                                state->system_status.ctrl_mode);
                        printf("Speed(mm/s): W1=%d, W2=%d, W3=%d, W4=%d\r\n",
                                state->wheel_speed.wheel1_speed,
                                state->wheel_speed.wheel2_speed,
                                state->wheel_speed.wheel3_speed,
                                state->wheel_speed.wheel4_speed);
                        printf("Angle(deg) : FL=%.2f, RL=%.2f, FR=%.2f, RR=%.2f\r\n",
                                state->wheel_angle.wheel1_angle * 0.01f,
                                state->wheel_angle.wheel2_angle * 0.01f,
                                state->wheel_angle.wheel3_angle * 0.01f,
                                state->wheel_angle.wheel4_angle * 0.01f);
                        printf("--------------------------------\r\n");
                        
                        /* 2. LCD 屏幕显示刷新 (坐标 10, 210 往下排列) */
                        snprintf(lcd_buf, sizeof(lcd_buf), "BAT:%.1fV %02d%% %5dmA", 
                                state->system_status.battery_voltage * 10 / 1000.0f, 
                                state->system_status.soc,
                                state->system_status.battery_current * 10);
                        lcd_show_string(10, 210, 240, 16, 16, lcd_buf, BLUE);
                        
                        snprintf(lcd_buf, sizeof(lcd_buf), "Spd: %4d %4d %4d %4d", 
                                state->wheel_speed.wheel1_speed, state->wheel_speed.wheel2_speed,
                                state->wheel_speed.wheel3_speed, state->wheel_speed.wheel4_speed);
                        lcd_show_string(10, 230, 240, 16, 16, lcd_buf, MAGENTA);
                        
                        snprintf(lcd_buf, sizeof(lcd_buf), "Ang: %3d  %3d  %3d  %3d", 
                                (int)(state->wheel_angle.wheel1_angle * 0.01f), (int)(state->wheel_angle.wheel2_angle * 0.01f),
                                (int)(state->wheel_angle.wheel3_angle * 0.01f), (int)(state->wheel_angle.wheel4_angle * 0.01f));
                        lcd_show_string(10, 250, 240, 16, 16, lcd_buf, MAGENTA);

                        /* ORE 错误自动恢复计数 (正常运行应为0，偶发不为0说明有电气干扰) */
                        snprintf(lcd_buf, sizeof(lcd_buf), "UART3 ORE-ERR: %lu", g_uart3_ore_cnt);
                        lcd_show_string(10, 270, 240, 16, 16, lcd_buf, 
                                        g_uart3_ore_cnt > 0 ? RED : GREEN);
                    }
                    break;

                case CAN_ID_WHEEL_SPEED:    /* 0x101 轮速 (30ms级, 屏蔽单行打印以防刷屏) */
                    break;

                case CAN_ID_WHEEL_ANGLE:    /* 0x102 轮角 (30ms级, 屏蔽单行打印以防刷屏) */
                    break;

                case CAN_ID_FAULT_FEEDBACK:  /* 0x103 故障包 (1000ms级) */
                    if (state->fault_status.fault_flag != 0)
                    {
                        char lcd_buf[50];
                        printf("[FAULT] walk=0x%02X steer=0x%02X comm_w=0x%02X comm_s=0x%02X\r\n",
                                state->fault_status.walk_fault,
                                state->fault_status.steer_fault,
                                state->fault_status.walk_comm_fault,
                                state->fault_status.steer_comm_fault);
                        snprintf(lcd_buf, sizeof(lcd_buf), "FAULT: W%02X S%02X",
                                 state->fault_status.walk_fault,
                                 state->fault_status.steer_fault);
                        lcd_show_string(10, 290, 240, 16, 16, lcd_buf, RED);
                    }
                    else
                    {
                        lcd_show_string(10, 290, 240, 16, 16, "FAULT: NONE         ", GREEN);
                    }
                    break;

                case CAN_ID_MOTION_FEEDBACK: /* 0x104 运动反馈 (20ms级, 触发里程计上报给Jetson) */
                    jetson_report_odom(state->motion_feedback.vx_fb,
                                       state->motion_feedback.vy_fb,
                                       state->motion_feedback.vz_fb);
                    break;

                case CAN_ID_CTRL_ENABLE:     
                case CAN_ID_MOTION_MODE:     
                case CAN_ID_MOTION_CTRL:     
                    /* 屏蔽回环或其它无用打印 */
                    break;

                default:
                    /* printf("[RX] Unknown ID: 0x%03X\r\n", rx_data.id); */
                    break;
            }
        }

        /* 【移到 if 外面】无论有没有收到新包，每200ms都刷新一次在线状态 */
        if (chassis_is_online())
        {
            lcd_show_string(10, 190, 220, 16, 16, "Status: ONLINE ", GREEN);
        }
        else
        {
            lcd_show_string(10, 190, 220, 16, 16, "Status: OFFLINE", RED);
        }
    }
}

/**
 * @brief  LED 闪烁任务 + IWDG 喂狗
 * @note   每500ms翻转一次LED，指示系统运行
 *         同时刷新看门狗，如果本任务被高优先级任务饿死，4秒后硬件复位
 */
void led_task(void *pvParameters)
{
    while (1)
    {
        LED0_TOGGLE();
        HAL_IWDG_Refresh(&g_iwdg_handle);  /* 喂狗 */
        vTaskDelay(500);
    }
}
