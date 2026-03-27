/**
 ****************************************************************************************************
 * @file        can.h
 * @author      (ALIENTEK)
 * @version     V1.1
 * @date        2023-06-06
 * @brief       CAN 驱动头文件
 * @license     Copyright (c) 2020-2032, ALIENTEK
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: 正点原子 探索者 F407 开发板
 * 在线视频: www.yuanzige.com
 * 技术论坛: www.openedv.com
 * 公司网址: www.alientek.com
 * 购买地址: openedv.taobao.com
 *
 * 版本说明:
 * V1.0 2021-10-25 初始版本
 * V1.1 2023-06-06 新增 can_send_msg 接口
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* CAN   */

#define CAN_RX_GPIO_PORT                GPIOA
#define CAN_RX_GPIO_PIN                 GPIO_PIN_11
#define CAN_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA */

#define CAN_TX_GPIO_PORT                GPIOA
#define CAN_TX_GPIO_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA */

/******************************************************************************************/

/* 使能 CAN RX0 中断接收 */
#define CAN_RX0_INT_ENABLE              1

/* CAN 接收数据结构体 */
typedef struct
{
    uint32_t id;        /* ID */
    uint8_t  data[8];   /* 数据区 */
    uint8_t  len;       /* DLC */
} CAN_RxData_t;

/* CAN 接口 */
#if !CAN_RX0_INT_ENABLE
uint8_t can_receive_msg(uint32_t id, uint8_t *buf);                                     /* CAN 轮询接收 */
#endif
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len);                           /* CAN */
uint8_t can_init(uint32_t tsjw,uint32_t tbs2,uint32_t tbs1,uint16_t brp,uint32_t mode); /* CAN */
void can_set_rx_queue(void *queue_handle);                                               /* 设置接收队列 */
uint32_t can_get_rx_drop_count(void);                                                    /* 获取CAN接收队列丢包计数 */

#endif
