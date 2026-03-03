/**
 ****************************************************************************************************
 * @file        jetson_usart.h
 * @author      RMP-WL100 通信中间件
 * @version     V1.0
 * @date        2026-03-04
 * @brief       针对 Jetson ROS2 控制的高速串口(USART3_DMA)底层驱动 (基于PB10, PB11)
 ****************************************************************************************************
 */

#ifndef __JETSON_USART_H
#define __JETSON_USART_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 通信协议宏与数据结构 */

#define JETSON_FRAME_LEN    10      /* 固定帧协议长度: 帧头2 + VX段2 + VY段2 + VZ段2 + 校验1 + 帧尾1 = 10 字节 */
#define JETSON_RX_BUF_SIZE  100     /* DMA缓存区必须足够大,防止高速轰炸造成溢出 */

/**
 * @brief ROS2 解析出的速度容器结构体
 * @note  单位要求：1000倍率整型。即 1000 = 1.0 m/s 或 1.0 rad/s
 */
typedef struct
{
    int16_t target_vx;      /* 收到的 X向目标线速度 (相对放大 1000 倍) */
    int16_t target_vy;      /* 收到的 Y向目标线速度 */
    int16_t target_vz;      /* 收到的 Z向目标角速度 */
    uint8_t frame_valid;    /* 有效新帧标记 (外部主循环提取后必须清零) */
} jetson_ctrl_cmd_t;


/* 外部接口 */
extern UART_HandleTypeDef g_uart3_handle;         /* USART3 UART句柄 */
extern DMA_HandleTypeDef g_dma_uart3_rx_handle;   /* USART3 接收DMA句柄 */

void jetson_usart_init(uint32_t baudrate);        /* 初始化函数 */
jetson_ctrl_cmd_t* get_jetson_cmd_ptr(void);      /* 获取速度容器指针 */

#endif
