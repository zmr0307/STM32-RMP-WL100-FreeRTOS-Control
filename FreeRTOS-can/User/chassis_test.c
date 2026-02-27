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
#include "./BSP/CAN/can.h"
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

    /* ① 初始化底盘驱动（回环测试模式） */
    if (chassis_driver_init(1) == CHASSIS_OK)
    {
        printf("[Init] Chassis driver init success (loopback mode)\r\n");
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
 * @brief  底盘控制测试任务
 * @note   测试流程：
 *         1. 发送控制使能（0x400）
 *         2. 发送运动模式（0x401）
 *         3. 发送运动控制（0x402）
 *         4. 循环发送不同速度指令
 */
void chassis_ctrl_task(void *pvParameters)
{
    chassis_motion_ctrl_t ctrl;
    chassis_err_e ret;
    uint8_t test_step = 0;

    printf("\r\n>>> Chassis Control Test Task Start\r\n");

    vTaskDelay(1000);  /* Wait 1s for system stable */

    /* ========== Test1: Send control enable ========== */
    printf("\r\n[Test1] Send control enable message (0x400)\r\n");
    ret = chassis_send_ctrl_enable(CTRL_MODE_CAN);
    if (ret == CHASSIS_OK)
    {
        printf("[Test1] OK Control enable sent\r\n");
        lcd_show_string(10, 130, 220, 16, 16, "Enable: OK  ", GREEN);
    }
    else
    {
        printf("[Test1] FAIL Control enable send failed\r\n");
        lcd_show_string(10, 130, 220, 16, 16, "Enable: FAIL", RED);
    }

    vTaskDelay(500);

    /* ========== Test2: Send motion mode ========== */
    printf("\r\n[Test2] Send motion mode message (0x401)\r\n");
    ret = chassis_send_motion_mode(MOTION_MODE_FORWARD);
    if (ret == CHASSIS_OK)
    {
        printf("[Test2] OK Motion mode sent (Ackermann forward)\r\n");
        lcd_show_string(10, 150, 220, 16, 16, "Mode: OK    ", GREEN);
    }
    else
    {
        printf("[Test2] FAIL Motion mode send failed\r\n");
        lcd_show_string(10, 150, 220, 16, 16, "Mode: FAIL  ", RED);
    }

    vTaskDelay(500);

    /* ========== Test3: Loop send motion control commands ========== */
    printf("\r\n[Test3] Start loop sending motion control commands (0x402)\r\n");

    while (1)
    {
        switch (test_step)
        {
            case 0:  /* Forward 0.5m/s */
                ctrl.vx = 0.5f;
                ctrl.vy = 0.0f;
                ctrl.vz = 0.0f;
                printf("\r\n[Ctrl] Forward: vx=0.5m/s, vy=0, vz=0\r\n");
                break;

            case 1:  /* Backward -0.3m/s */
                ctrl.vx = -0.3f;
                ctrl.vy = 0.0f;
                ctrl.vz = 0.0f;
                printf("\r\n[Ctrl] Backward: vx=-0.3m/s, vy=0, vz=0\r\n");
                break;

            case 2:  /* Turn left 0.5rad/s */
                ctrl.vx = 0.0f;
                ctrl.vy = 0.0f;
                ctrl.vz = 0.5f;
                printf("\r\n[Ctrl] Turn left: vx=0, vy=0, vz=0.5rad/s\r\n");
                break;

            case 3:  /* Turn right -0.5rad/s */
                ctrl.vx = 0.0f;
                ctrl.vy = 0.0f;
                ctrl.vz = -0.5f;
                printf("\r\n[Ctrl] Turn right: vx=0, vy=0, vz=-0.5rad/s\r\n");
                break;

            case 4:  /* Stop */
                ctrl.vx = 0.0f;
                ctrl.vy = 0.0f;
                ctrl.vz = 0.0f;
                printf("\r\n[Ctrl] Stop: vx=0, vy=0, vz=0\r\n");
                break;
        }

        /* 发送控制指令 */
        ret = chassis_send_motion_ctrl(&ctrl);
        if (ret == CHASSIS_OK)
        {
            printf("[Ctrl] OK Motion control sent\r\n");
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: OK    ", BLUE);
        }
        else
        {
            printf("[Ctrl] FAIL Motion control send failed\r\n");
            lcd_show_string(10, 170, 220, 16, 16, "Ctrl: FAIL  ", RED);
        }

        /* 切换到下一个测试步骤 */
        test_step++;
        if (test_step > 4)
        {
            test_step = 0;
        }

        vTaskDelay(3000);  /* 每3秒发送一次 */
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
            /* 打印接收到的原始CAN数据 */
            printf("[RX] ID=0x%03X, Len=%d, Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    rx_data.id,
                    rx_data.len,
                    rx_data.data[0], rx_data.data[1], rx_data.data[2], rx_data.data[3],
                    rx_data.data[4], rx_data.data[5], rx_data.data[6], rx_data.data[7]);

            /* 调用底盘驱动处理函数（解码） */
            chassis_process_feedback(rx_data.id, rx_data.data, rx_data.len);

            /* 获取底盘状态 */
            state = chassis_get_state();

            /* 根据ID打印解码后的数据 */
            switch (rx_data.id)
            {
                case CAN_ID_SYSTEM_STATUS:  /* 0x100 系统状态 */
                    printf("[Decode] System status: Voltage=%dmV, Current=%dmA, SOC=%d%%, State=%d\r\n",
                            state->system_status.battery_voltage * 10,  /* 10mV -> mV */
                            state->system_status.battery_current * 10,  /* 10mA -> mA */
                            state->system_status.soc,
                            state->system_status.system_state);
                    break;

                case CAN_ID_WHEEL_SPEED:    /* 0x101 轮速 */
                    printf("[Decode] Wheel speed: W1=%d, W2=%d, W3=%d, W4=%d\r\n",
                            state->wheel_speed.wheel1_speed,
                            state->wheel_speed.wheel2_speed,
                            state->wheel_speed.wheel3_speed,
                            state->wheel_speed.wheel4_speed);
                    break;

                case CAN_ID_WHEEL_ANGLE:    /* 0x102 轮角 */
                    printf("[Decode] Wheel angle: W1=%d, W2=%d, W3=%d, W4=%d\r\n",
                            state->wheel_angle.wheel1_angle,
                            state->wheel_angle.wheel2_angle,
                            state->wheel_angle.wheel3_angle,
                            state->wheel_angle.wheel4_angle);
                    break;

                case CAN_ID_MOTION_FEEDBACK: /* 0x104 运动反馈 */
                    printf("[Decode] Motion feedback: vx=%d, vy=%d, vz=%d\r\n",
                            state->motion_feedback.vx_fb,
                            state->motion_feedback.vy_fb,
                            state->motion_feedback.vz_fb);
                    break;

                case CAN_ID_CTRL_ENABLE:     /* 0x400 控制使能（回环收到自己发的） */
                    printf("[Loopback] Received control enable message (self-sent)\r\n");
                    break;

                case CAN_ID_MOTION_MODE:     /* 0x401 运动模式（回环收到自己发的） */
                    printf("[Loopback] Received motion mode message (self-sent)\r\n");
                    break;

                case CAN_ID_MOTION_CTRL:     /* 0x402 Motion control (loopback received self-sent) */
                    printf("[Loopback] Received motion control message (self-sent)\r\n");

                    /* Simulate chassis feedback in loopback mode */
                    {
                        CAN_RxData_t feedback;
                        feedback.id = CAN_ID_SYSTEM_STATUS;  /* 0x100 System status */
                        feedback.len = 8;
                        /* 构造模拟的0x100反馈包 */
                        feedback.data[0] = 0x00;  /* Byte[0]: System state (0: normal) */
                        feedback.data[1] = 0x01;  /* Byte[1]: Ctrl mode (1: CAN code) */
                        feedback.data[2] = 0x00;  /* Byte[2]: Peripheral (0: clear) */
                        feedback.data[3] = 80;    /* Byte[3]: SOC 80% */
                        
                        /* Voltage: 48V = 48000mV = 4800 * 10mV, 0x12C0 */
                        feedback.data[4] = 0x12;  /* Byte[4]: Voltage high byte */
                        feedback.data[5] = 0xC0;  /* Byte[5]: Voltage low byte */
                        
                        /* Current: 放电3A = 3000mA = 300 * 10mA, 0x012C */
                        feedback.data[6] = 0x01;  /* Byte[6]: Current high byte */
                        feedback.data[7] = 0x2C;  /* Byte[7]: Current low byte */

                        /* Send simulated feedback to queue */
                        xQueueSend(g_can_rx_queue, &feedback, 0);
                    }
                    break;

                default:
                    printf("[RX] Unknown ID: 0x%03X\r\n", rx_data.id);
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
