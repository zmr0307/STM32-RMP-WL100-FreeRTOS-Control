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

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/******************************************************************************************************/
/*                                    FreeRTOS Tasks                                                   */
/******************************************************************************************************/

/* ============ Start Task ============ */
#define START_TASK_PRIO     1
#define START_STK_SIZE      128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

/* ============ Chassis Control Test Task ============ */
#define CHASSIS_CTRL_TASK_PRIO    2
#define CHASSIS_CTRL_STK_SIZE     512
TaskHandle_t Chassis_Ctrl_Task_Handler;
void chassis_ctrl_task(void *pvParameters);

/* ============ Chassis Feedback Processing Task ============ */
#define CHASSIS_RX_TASK_PRIO      3
#define CHASSIS_RX_STK_SIZE       512
TaskHandle_t Chassis_RX_Task_Handler;
void chassis_rx_task(void *pvParameters);

/* ============ LED Blink Task ============ */
#define LED_TASK_PRIO       1
#define LED_STK_SIZE        128
TaskHandle_t LED_Task_Handler;
void led_task(void *pvParameters);

/* ============ CAN接收消息队列 ============ */
#define CAN_RX_QUEUE_LEN    10
QueueHandle_t g_can_rx_queue;

/* ============ Printf 互斥锁 ============ */
#include <stdarg.h>
#include <stdio.h>
#include "semphr.h"

SemaphoreHandle_t g_printf_mutex = NULL;

void safe_printf(const char *format, ...)
{
    if (g_printf_mutex != NULL)
    {
        xSemaphoreTake(g_printf_mutex, portMAX_DELAY);
    }
    
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    if (g_printf_mutex != NULL)
    {
        xSemaphoreGive(g_printf_mutex);
    }
}
#define printf(...) safe_printf(__VA_ARGS__)

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
    lcd_show_string(10, 100, 220, 16, 16, "LoopBack Mode", BLUE);

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

    /* ④ 创建启动任务 */
    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&StartTask_Handler);

    /* ⑤ 启动调度器 */
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
 * @brief  底盘安全全向首飞微操测试任务
 * @note   利用底座内置 0x402 全向平滑算法，测试机器人的运动控制：
 *         1. 切入自旋模式 0x04
 *         2. 状态交替：停稳 5秒 -> 逆时针自转 2秒 -> 永久安全停车
 *         不会改变 LCD 刷屏任务。
 */
void chassis_ctrl_task(void *pvParameters)
{
    chassis_err_e ret;
    chassis_motion_ctrl_t motion_cmd;

    uint8_t state_step = 0;   /* 0: 静止等待, 1: 侧向微滑 */
    uint16_t timer_100ms = 0; /* 100ms计次定时器 */

    printf("\r\n>>> Chassis OMNI-DIRECTIONAL Safe Test Task Start <<<\r\n");

    vTaskDelay(1000);  /* Wait 1s for system stable */

    /* ========== Test1: Send control enable ========== */
    ret = chassis_send_ctrl_enable(CTRL_MODE_CAN);
    if (ret == CHASSIS_OK)
    {
        lcd_show_string(10, 130, 220, 16, 16, "Enable: OK  ", GREEN);
    }
    else
    {
        lcd_show_string(10, 130, 220, 16, 16, "Enable: FAIL", RED);
    }

    vTaskDelay(500);

    /* ========== Test2: Send SPIN Mode (自旋模式，交由原厂内部做平滑处理) ========== */
    printf("\r\n[Test2] Requesting SPIN Control Mode (0x04)...\r\n");
    /* 发送自带的原地打转模式 */
    ret = chassis_send_motion_mode(MOTION_MODE_SPIN); 
    if (ret == CHASSIS_OK)
    {
        /* 借用以前LCD字符占位免得LCD闪烁 */
        lcd_show_string(10, 150, 220, 16, 16, "Mode: SPIN  ", BLUE); 
    }

    vTaskDelay(500);

    /* ========== Test3: OMNI Safe Loop (100ms周期循环) ========== */
    printf("\r\n[Test3] Starting SPIN Test (Stop 5s, CCW 2s, Stop Forever)\r\n");

    while (1)
    {
        float target_vx = 0.0f;
        float target_vy = 0.0f;
        float target_vz = 0.0f;

        /* 简易安全状态机 */
        if (state_step == 0)
        {
            /* 状态 0：绝对静止期 (5秒 = 50 * 100ms) */
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;
            
            if (timer_100ms == 0) printf("\r\n[SAFE STOP] Wheels aligning to 0, holding for 5s...");
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: STOP  ", BLUE);

            timer_100ms++;
            if (timer_100ms >= 50) 
            {
                state_step = 1;
                timer_100ms = 0;
            }
        }
        else if (state_step == 1)
        {
            /* 状态 1：原地陀螺缓慢自转期 (2秒 = 20 * 100ms, 逆时针恢复) */
            target_vx = 0.0f;  
            target_vy = 0.0f; 
            target_vz = 0.2f; /* 自转反向恢复：逆时针旋转，+0.2 弧度/秒 */
            
            if (timer_100ms == 0) printf("\r\n[SPIN] Spinning CCW (Positive Z) at +0.2 rad/s for 2s...");
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: CCW   ", MAGENTA);

            timer_100ms++;
            if (timer_100ms >= 20) 
            {
                /* 执行完2秒恢复自传后，斩断循环，进入永久停机状态 */
                state_step = 2; 
                timer_100ms = 0;
            }
        }
        else if (state_step == 2)
        {
            /* 状态 2：永久静止期 (测试结束保活) */
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;
            
            if (timer_100ms == 0) printf("\r\n[TEST DONE] Test sequence finished. Holding 0 speed forever.");
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: FINISH", BLUE);
        }

        /* 1. 不再调用手写的矩阵！直接将 Vz 塞给官方 0x402 骨架 */
        motion_cmd.vx = target_vx;
        motion_cmd.vy = target_vy;
        motion_cmd.vz = target_vz;

        /* 3. 使用底盘内部的全向平滑避磨损算法处理 */
        chassis_err_e err = chassis_send_motion_ctrl(&motion_cmd);

        if (err != CHASSIS_OK)
        {
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: CANERR", RED);
        }

        /* 必须维持在 100ms 的精确发包心跳 */
        vTaskDelay(100);  
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
        /* 从队列等待接收数据（永久等待） */
        ret = xQueueReceive(g_can_rx_queue, &rx_data, portMAX_DELAY);

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
                        sprintf(lcd_buf, "BAT:%.1fV %02d%% %5dmA", 
                                state->system_status.battery_voltage * 10 / 1000.0f, 
                                state->system_status.soc,
                                state->system_status.battery_current * 10);
                        lcd_show_string(10, 210, 240, 16, 16, lcd_buf, BLUE);
                        
                        sprintf(lcd_buf, "Spd: %4d %4d %4d %4d", 
                                state->wheel_speed.wheel1_speed, state->wheel_speed.wheel2_speed,
                                state->wheel_speed.wheel3_speed, state->wheel_speed.wheel4_speed);
                        lcd_show_string(10, 230, 240, 16, 16, lcd_buf, MAGENTA);
                        
                        sprintf(lcd_buf, "Ang: %3d  %3d  %3d  %3d", 
                                (int)(state->wheel_angle.wheel1_angle * 0.01f), (int)(state->wheel_angle.wheel2_angle * 0.01f),
                                (int)(state->wheel_angle.wheel3_angle * 0.01f), (int)(state->wheel_angle.wheel4_angle * 0.01f));
                        lcd_show_string(10, 250, 240, 16, 16, lcd_buf, MAGENTA);
                    }
                    break;

                case CAN_ID_WHEEL_SPEED:    /* 0x101 轮速 (30ms级, 屏蔽单行打印以防刷屏) */
                    break;

                case CAN_ID_WHEEL_ANGLE:    /* 0x102 轮角 (30ms级, 屏蔽单行打印以防刷屏) */
                    break;

                case CAN_ID_FAULT_FEEDBACK:  /* 0x103 故障包 (1000ms级, 屏蔽以防干扰仪表盘) */
                    break;

                case CAN_ID_MOTION_FEEDBACK: /* 0x104 运动反馈 (20ms级, 屏蔽单行打印以防刷屏) */
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

            /* 检查底盘在线状态 */
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
}

/**
 * @brief  LED 闪烁任务
 * @note   每500ms翻转一次LED，指示系统运行
 */
void led_task(void *pvParameters)
{
    while (1)
    {
        LED0_TOGGLE();
        vTaskDelay(500);
    }
}
