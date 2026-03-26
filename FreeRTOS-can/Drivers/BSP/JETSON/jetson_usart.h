/**
 ****************************************************************************************************
 * @file        jetson_usart.h
 * @author      RMP-WL100 ͨ���м��
 * @version     V1.0
 * @date        2026-03-04
 * @brief       ��� Jetson ROS2 ���Ƶĸ��ٴ���(USART3_DMA)�ײ����� (����PB10, PB11)
 ****************************************************************************************************
 */

#ifndef __JETSON_USART_H
#define __JETSON_USART_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* ͨ��Э��������ݽṹ */

#define JETSON_FRAME_LEN    10      /* �̶�֡Э�鳤��: ֡ͷ2 + VX��2 + VY��2 + VZ��2 + У��1 + ֡β1 = 10 �ֽ� */
#define JETSON_RX_BUF_SIZE  100     /* DMA�����������㹻��,��ֹ���ٺ�ը������ */

/**
 * @brief ROS2 ���������ٶ������ṹ��
 * @note  ��λҪ��1000�������͡��� 1000 = 1.0 m/s �� 1.0 rad/s
 */
typedef struct
{
    int16_t target_vx;      /* �յ��� X��Ŀ�����ٶ� (��ԷŴ� 1000 ��) */
    int16_t target_vy;      /* �յ��� Y��Ŀ�����ٶ� */
    int16_t target_vz;      /* �յ��� Z��Ŀ����ٶ� */
    uint8_t frame_valid;
    uint32_t last_valid_tick;   /* ����յ��Ϸ�֡�� HAL_GetTick() ʱ��� */
} jetson_ctrl_cmd_t;


/******************************************************************************************/
/* �ϱ�Э��궨�� (STM32 -> Jetson, ���·�Э�� A5 5A ��ȫ����) */

#define JETSON_REPORT_HEADER1       0xAA    /* �ϱ�֡ͷ��1�ֽ� */
#define JETSON_REPORT_HEADER2       0x55    /* �ϱ�֡ͷ��2�ֽ� */
#define JETSON_REPORT_TAIL          0xEE    /* �ϱ�֡β */
#define JETSON_REPORT_FRAME_LEN     11      /* �ϱ�֡�ܳ���: ֡ͷ2 + ����1 + ����6 + У��1 + ֡β1 */

/* �ϱ���Ϣ���� */
#define JETSON_REPORT_TYPE_ODOM     0x01    /* ��̼��ٶ��ϱ� */
#define JETSON_REPORT_TYPE_BATTERY  0x02    /* ���״̬�ϱ�(Ԥ��) */

/******************************************************************************************/

/* �ⲿ�ӿ� */
extern UART_HandleTypeDef g_uart3_handle;         /* USART3 UART��� */
extern DMA_HandleTypeDef g_dma_uart3_rx_handle;   /* USART3 ����DMA��� */

void jetson_usart_init(uint32_t baudrate);        /* ��ʼ������ */
jetson_ctrl_cmd_t* get_jetson_cmd_ptr(void);      /* ��ȡ�ٶ�����ָ�� */
void jetson_report_odom(int16_t vx, int16_t vy, int16_t vz);  /* ��̼��ٶ��ϱ� */

#endif
