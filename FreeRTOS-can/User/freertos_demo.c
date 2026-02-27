/**
 ****************************************************************************************************
 * @file        freertos_demo.c
 * @brief       FreeRTOS + CAN 通讯功能（中断+队列模式）
 * @note        适配正点原子 探索者F407 开发板
 *              功能：RTOS下的CAN收发任务
 *              - task1: 每2秒发送一帧CAN数据
 *              - task2: 从队列读取CAN接收数据并打印
 *              - CAN接收中断: 收到数据后入队列
 ****************************************************************************************************
 */

#include "freertos_demo.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/CAN/can.h"

/* FreeRTOS 头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/******************************************************************************************************/
/*                                    数据结构定义                                                      */
/******************************************************************************************************/

/* 注意：CAN_RxData_t 已经在 can.h 中定义，这里不需要重复定义 */

/******************************************************************************************************/
/*                                    FreeRTOS 配置                                                     */
/******************************************************************************************************/

/* ============ 启动任务 ============ */
#define START_TASK_PRIO     1
#define START_STK_SIZE      128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

/* ============ CAN发送任务 ============ */
#define CAN_TX_TASK_PRIO    2               /* 任务优先级较低 */
#define CAN_TX_STK_SIZE     256
TaskHandle_t CAN_TX_Task_Handler;
void can_tx_task(void *pvParameters);

/* ============ CAN接收处理任务 ============ */
#define CAN_RX_TASK_PRIO    3               /* 任务优先级较高，数据更重要*/
#define CAN_RX_STK_SIZE     256
TaskHandle_t CAN_RX_Task_Handler;
void can_rx_task(void *pvParameters);

/* ============ LED闪烁任务 ============ */
#define LED_TASK_PRIO       1               /* 任务优先级最低 */
#define LED_STK_SIZE        128
TaskHandle_t LED_Task_Handler;
void led_task(void *pvParameters);

/* ============ CAN接收消息队列 ============ */
#define CAN_RX_QUEUE_LEN    10              /* 队列长度，最多缓存10条CAN消息 */
QueueHandle_t g_can_rx_queue;               /* 队列句柄 */

/******************************************************************************************************/
/*                                    核心函数                                                          */
/******************************************************************************************************/

/**
 * @brief  FreeRTOS + CAN 核心函数
 */
void freertos_demo(void)
{
    /* LCD 显示标题 */
    lcd_show_string(10, 10, 220, 32, 32, "STM32", RED);
    lcd_show_string(10, 47, 220, 24, 24, "CAN + RTOS", RED);
    lcd_show_string(10, 76, 220, 16, 16, "ATOM@ALIENTEK", RED);
    lcd_show_string(10, 100, 220, 16, 16, "LoopBack Mode", BLUE);

    /* ① 初始化CAN（回环模式，500Kbps */
    can_init(CAN_SJW_1TQ, CAN_BS2_6TQ, CAN_BS1_7TQ, 6, CAN_MODE_LOOPBACK);
    printf("CAN 初始化完成：回环模式，500Kbps。\r\n");

    /* ② 创建CAN接收消息队列 */
    g_can_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_RxData_t));
    if (g_can_rx_queue == NULL)
    {
        printf("创建CAN接收队列失败！\r\n");
        while (1);  /* 死循环，停止运行 */
    }
    printf("CAN 接收队列创建成功，容量：%d条\r\n", CAN_RX_QUEUE_LEN);

    /* ③ 将队列句柄传递给CAN驱动（用于中断中入队） */
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
 * @brief  启动任务：创建所有工作任务，然后自杀
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    /* 创建 CAN 发送任务 */
    xTaskCreate((TaskFunction_t )can_tx_task,
                (const char*    )"can_tx_task",
                (uint16_t       )CAN_TX_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CAN_TX_TASK_PRIO,
                (TaskHandle_t*  )&CAN_TX_Task_Handler);

    /* 创建 CAN 接收处理任务 */
    xTaskCreate((TaskFunction_t )can_rx_task,
                (const char*    )"can_rx_task",
                (uint16_t       )CAN_RX_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CAN_RX_TASK_PRIO,
                (TaskHandle_t*  )&CAN_RX_Task_Handler);

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
 * @brief  CAN 发送任务
 * @note   每2秒发送一帧测试数据：ID=0x12，8字节的递增数据）
 *         （在回环模式下，发送的数据会被自己的接收中断捕获，形成发送到任务级的指令）
 */
void can_tx_task(void *pvParameters)
{
    uint8_t tx_buf[8];
    uint8_t cnt = 0;
    uint8_t res;

    printf(">>> CAN 发送任务已启动\r\n");

    while (1)
    {
        /* 构造测试数据：cnt, cnt+1, ... cnt+7 */
        for (uint8_t i = 0; i < 8; i++)
        {
            tx_buf[i] = cnt + i;
        }


        /* 发送 CAN 消息：ID=0x12，8字节 */
        res = can_send_msg(0x12, tx_buf, 8);

        if (res == 0)
        {
            printf("[TX] ID=0x12, Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                    tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);
            lcd_show_string(10, 130, 220, 16, 16, "TX: OK  ", GREEN);
        }
        else
        {
            printf("[TX] 发送失败！\r\n");
            lcd_show_string(10, 130, 220, 16, 16, "TX: FAIL", RED);
        }

        cnt++;
        vTaskDelay(2000);   /* 每2秒发一次，让出CPU给其他任务*/
    }
}

/**
 * @brief  CAN 接收处理任务
 * @note   从队列中等待接收数据，收到数据就打印
 *         （在回环模式下，发送的数据会被任务级返回到任务级/队列状态）
 */
void can_rx_task(void *pvParameters)
{
    CAN_RxData_t rx_data;
    BaseType_t ret;

    printf(">>> CAN 接收任务已启动（中断+队列模式）\r\n");

    while (1)
    {
        /* 从队列中阻塞等待接收数据（portMAX_DELAY = 永久等待） */
        ret = xQueueReceive(g_can_rx_queue, &rx_data, portMAX_DELAY);

        if (ret == pdTRUE)
        {
            printf("[RX] ID=0x%02X, Len=%d, Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    rx_data.id,
                    rx_data.len,
                    rx_data.data[0], rx_data.data[1], rx_data.data[2], rx_data.data[3],
                    rx_data.data[4], rx_data.data[5], rx_data.data[6], rx_data.data[7]);

            /* 在LCD上显示收到数据 */
            lcd_show_string(10, 150, 220, 16, 16, "RX: OK  ", BLUE);

            /* ========================================
             * 这里添加你的数据处理逻辑：
             * 例如：解析底盘返回的电机转速、电机计数等
             *
             * rx_data.id       - 接收到的CAN ID
             * rx_data.data[0~7] - 接收到的8字节数据
             * rx_data.len      - 数据长度
             * ======================================== */
        }
    }
}

/**
 * @brief  LED 闪烁任务
 * @note   每500ms翻转一次LED，显示系统运行
 */
void led_task(void *pvParameters)
{
    while (1)
    {
        LED0_TOGGLE();
        vTaskDelay(500);
    }
}
