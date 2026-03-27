/**
 ****************************************************************************************************
 * @file        jetson_usart.h
 * @author      RMP-WL100 通信中间件
 * @version     V1.0
 * @date        2026-03-04
 * @brief       面向 Jetson ROS2 控制的高速串口(USART3_DMA)底层驱动 (PB10/PB11)
 ****************************************************************************************************
 */

#ifndef __JETSON_USART_H
#define __JETSON_USART_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 通信协议与数据结构 */

#define JETSON_FRAME_LEN    10      /* 固定帧长: 帧头2 + VX2 + VY2 + VZ2 + 校验1 + 帧尾1 */
#define JETSON_RX_BUF_SIZE  100     /* DMA 接收缓冲区大小 */

/**
 * @brief ROS2 下发的目标速度结构体
 * @note  协议单位放大 1000 倍，即 1000 = 1.0 m/s 或 1.0 rad/s
 */
typedef struct
{
    int16_t target_vx;      /* 收到的 X 方向目标速度 */
    int16_t target_vy;      /* 收到的 Y 方向目标速度 */
    int16_t target_vz;      /* 收到的 Z 方向目标角速度 */
    uint8_t frame_valid;
    uint32_t last_valid_tick;   /* 最近一次收到合法帧的 HAL_GetTick() 时间戳 */
} jetson_ctrl_cmd_t;

/******************************************************************************************/
/* 上报协议定义 (STM32 -> Jetson，与下发协议 A5 5A 完全区分) */

#define JETSON_REPORT_HEADER1       0xAA    /* 上报帧头第1字节 */
#define JETSON_REPORT_HEADER2       0x55    /* 上报帧头第2字节 */
#define JETSON_REPORT_TAIL          0xEE    /* 上报帧尾 */
#define JETSON_REPORT_FRAME_LEN     11      /* 上报帧总长: 帧头2 + 类型1 + 数据6 + 校验1 + 帧尾1 */

/* 上报消息类型 */
#define JETSON_REPORT_TYPE_ODOM     0x01    /* 里程计速度上报 */
#define JETSON_REPORT_TYPE_BATTERY  0x02    /* 电池状态上报(预留) */

/******************************************************************************************/

/* 外部接口 */
extern UART_HandleTypeDef g_uart3_handle;         /* USART3 UART 句柄 */
extern DMA_HandleTypeDef g_dma_uart3_rx_handle;   /* USART3 RX DMA 句柄 */

void jetson_usart_init(uint32_t baudrate);        /* 初始化串口 */
void jetson_process_staged_rx(void);              /* 任务上下文处理暂存接收数据 */
jetson_ctrl_cmd_t* get_jetson_cmd_ptr(void);      /* 获取速度指针 */
uint32_t jetson_get_stage_overwrite_count(void);  /* 获取暂存区覆盖计数 */
void jetson_report_odom(int16_t vx, int16_t vy, int16_t vz);  /* 里程计速度上报 */

#endif
