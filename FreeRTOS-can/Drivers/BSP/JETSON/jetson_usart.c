/**
 ****************************************************************************************************
 * @file        jetson_usart.c
 * @author      RMP-WL100 通信中间件
 * @version     V1.0
 * @date        2026-03-04
 * @brief       针对 Jetson ROS2 控制的高速串口(USART3_DMA)底层驱动
 * @note        硬件连接: PB10=TX (接Jetson的RX), PB11=RX (接Jetson的TX)
 *              架构机制: 串口空闲中断(IDLE) + DMA 接收 + 任务上下文解析
 *              协议格式: [0xA5, 0x5A, VX_H, VX_L, VY_H, VY_L, VZ_H, VZ_L, XorCheck, 0xEE]
 ****************************************************************************************************
 */

#include "jetson_usart.h"
#include <string.h>

/******************************************************************************************/
/* 核心底层句柄与缓存阵列 */

UART_HandleTypeDef g_uart3_handle;       /* UART3 句柄 */
DMA_HandleTypeDef g_dma_uart3_rx_handle; /* DMA 句柄 */

/* DMA 直接写入的内存池 */
static uint8_t g_jetson_rx_buf[JETSON_RX_BUF_SIZE] __attribute__((aligned(32)));
static uint8_t g_jetson_stage_buf[JETSON_RX_BUF_SIZE];

/* 解析完成的控制指令和暂存区状态
 * 单槽邮箱语义: ISR 始终保留最近一次 IDLE 截获的数据块。
 * 若任务还未消费上一块数据，新的数据会覆盖旧数据，并累计 overwrite_count。
 * 该设计适用于连续速度控制，只保证“最新值有效”，不保证“逐帧可靠消费”。
 */
static jetson_ctrl_cmd_t g_jetson_cmd = {0};
static volatile uint16_t g_jetson_stage_len = 0;
static volatile uint8_t g_jetson_stage_pending = 0;
static volatile uint32_t g_jetson_stage_overwrite_count = 0;

/* ORE/FE/NE 等串口硬件错误累计次数 (供调试和LCD显示) */
uint32_t g_uart3_ore_cnt = 0;

/******************************************************************************************/

/**
 * @brief       暴露容器的安全指针供任务读取
 */
jetson_ctrl_cmd_t* get_jetson_cmd_ptr(void)
{
    return &g_jetson_cmd;
}

/**
 * @brief       获取 Jetson 暂存区覆盖计数
 */
uint32_t jetson_get_stage_overwrite_count(void)
{
    return g_jetson_stage_overwrite_count;
}

/**
 * @brief       协议校验与剥离解析子函数
 * @param       buf: 数据头指针
 * @param       len: 捕获到这一串收到的总长度
 */
static void jetson_protocol_decode(uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (len < JETSON_FRAME_LEN)
    {
        return;
    }

    for (i = 0; i <= (len - JETSON_FRAME_LEN); i++)
    {
        if (buf[i] == 0xA5 && buf[i + 1] == 0x5A)
        {
            if (buf[i + 9] == 0xEE)
            {
                uint8_t cal_check = 0;

                for (uint8_t j = 0; j < 8; j++)
                {
                    cal_check ^= buf[i + j];
                }

                if (cal_check == buf[i + 8])
                {
                    g_jetson_cmd.target_vx = (int16_t)((buf[i + 2] << 8) | buf[i + 3]);
                    g_jetson_cmd.target_vy = (int16_t)((buf[i + 4] << 8) | buf[i + 5]);
                    g_jetson_cmd.target_vz = (int16_t)((buf[i + 6] << 8) | buf[i + 7]);
                    g_jetson_cmd.frame_valid = 1;
                    g_jetson_cmd.last_valid_tick = HAL_GetTick();

                    i += (JETSON_FRAME_LEN - 1);
                }
            }
        }
    }
}

/**
 * @brief       在任务上下文处理暂存区中的串口数据
 * @note        每次只提取当前邮箱中的最新一块数据；若 ISR 期间发生覆盖，旧块不会逐帧补处理
 */
void jetson_process_staged_rx(void)
{
    uint8_t local_buf[JETSON_RX_BUF_SIZE];
    uint16_t local_len = 0;

    __disable_irq();
    if (g_jetson_stage_pending != 0)
    {
        local_len = g_jetson_stage_len;
        if (local_len > JETSON_RX_BUF_SIZE)
        {
            local_len = JETSON_RX_BUF_SIZE;
        }

        memcpy(local_buf, g_jetson_stage_buf, local_len);
        g_jetson_stage_pending = 0;
        g_jetson_stage_len = 0;
    }
    __enable_irq();

    if (local_len > 0)
    {
        jetson_protocol_decode(local_buf, local_len);
    }
}

/**
 * @brief       USART3 初始化并开启 DMA 空闲接收模式
 * @param       baudrate: 波特率 (推荐 115200)
 */
void jetson_usart_init(uint32_t baudrate)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;

    gpio_init_struct.Pin = GPIO_PIN_10;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    g_uart3_handle.Instance = USART3;
    g_uart3_handle.Init.BaudRate = baudrate;
    g_uart3_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart3_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart3_handle.Init.Parity = UART_PARITY_NONE;
    g_uart3_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart3_handle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart3_handle);

    __HAL_RCC_DMA1_CLK_ENABLE();

    g_dma_uart3_rx_handle.Instance = DMA1_Stream1;
    g_dma_uart3_rx_handle.Init.Channel = DMA_CHANNEL_4;
    g_dma_uart3_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_dma_uart3_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_uart3_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_uart3_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_uart3_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_dma_uart3_rx_handle.Init.Mode = DMA_CIRCULAR;
    g_dma_uart3_rx_handle.Init.Priority = DMA_PRIORITY_HIGH;
    g_dma_uart3_rx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_dma_uart3_rx_handle);

    __HAL_LINKDMA(&g_uart3_handle, hdmarx, g_dma_uart3_rx_handle);

    __HAL_UART_ENABLE_IT(&g_uart3_handle, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&g_uart3_handle, g_jetson_rx_buf, JETSON_RX_BUF_SIZE);

    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * @brief       USART3 中断服务函数
 */
void USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&g_uart3_handle, UART_FLAG_IDLE) != RESET)
    {
        uint16_t current_rx_len;

        __HAL_UART_CLEAR_IDLEFLAG(&g_uart3_handle);

        current_rx_len = JETSON_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(g_uart3_handle.hdmarx);
        if (current_rx_len > JETSON_RX_BUF_SIZE)
        {
            current_rx_len = JETSON_RX_BUF_SIZE;
        }

        if (current_rx_len > 0)
        {
            /* 先拷贝到任务侧暂存邮箱，再重启 DMA，避免 DMA 复用接收缓冲区时覆盖当前数据 */
            memcpy(g_jetson_stage_buf, g_jetson_rx_buf, current_rx_len);
        }

        HAL_UART_AbortReceive(&g_uart3_handle);
        HAL_UART_Receive_DMA(&g_uart3_handle, g_jetson_rx_buf, JETSON_RX_BUF_SIZE);

        if (current_rx_len > 0)
        {
            if (g_jetson_stage_pending != 0)
            {
                /* 单槽邮箱已被占用: 统计一次覆盖，随后用最新数据替换旧数据 */
                g_jetson_stage_overwrite_count++;
            }

            g_jetson_stage_len = current_rx_len;
            g_jetson_stage_pending = 1;
        }
    }

    __HAL_UART_CLEAR_OREFLAG(&g_uart3_handle);
    HAL_UART_IRQHandler(&g_uart3_handle);
}

/******************************************************************************************/
/* 里程计上报函数 (STM32 -> Jetson) */

/**
 * @brief       向 Jetson 上报底盘里程计速度 (Vx/Vy/Vz)
 * @param       vx: X方向真实线速度 (单位: 0.001 m/s, 即底盘 0x104 原始值)
 * @param       vy: Y方向真实线速度 (单位: 0.001 m/s)
 * @param       vz: Z方向真实角速度 (单位: 0.001 rad/s)
 * @note        帧格式: [AA][55][0x01][Vx_H][Vx_L][Vy_H][Vy_L][Vz_H][Vz_L][XOR][EE]
 *              共 11 字节, 使用阻塞式发送, @115200 约需 1ms, 不影响系统实时性
 *              本函数仅操作 USART3 的 TX 通道, 与 DMA RX 接收引擎完全独立互不干扰
 */
void jetson_report_odom(int16_t vx, int16_t vy, int16_t vz)
{
    uint8_t tx_buf[JETSON_REPORT_FRAME_LEN];
    uint8_t xor_check = 0;

    tx_buf[0] = JETSON_REPORT_HEADER1;
    tx_buf[1] = JETSON_REPORT_HEADER2;
    tx_buf[2] = JETSON_REPORT_TYPE_ODOM;

    tx_buf[3] = (uint8_t)((vx >> 8) & 0xFF);
    tx_buf[4] = (uint8_t)(vx & 0xFF);
    tx_buf[5] = (uint8_t)((vy >> 8) & 0xFF);
    tx_buf[6] = (uint8_t)(vy & 0xFF);
    tx_buf[7] = (uint8_t)((vz >> 8) & 0xFF);
    tx_buf[8] = (uint8_t)(vz & 0xFF);

    for (uint8_t i = 0; i < 9; i++)
    {
        xor_check ^= tx_buf[i];
    }
    tx_buf[9] = xor_check;
    tx_buf[10] = JETSON_REPORT_TAIL;

    __HAL_UART_DISABLE_IT(&g_uart3_handle, UART_IT_IDLE);
    HAL_UART_Transmit(&g_uart3_handle, tx_buf, JETSON_REPORT_FRAME_LEN, 10);
    __HAL_UART_ENABLE_IT(&g_uart3_handle, UART_IT_IDLE);
}

/**
 * @brief       UART 错误恢复回调
 * @note        无论什么错误，都清标志并强制重启 DMA 接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        g_uart3_ore_cnt++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_DMA(huart, g_jetson_rx_buf, JETSON_RX_BUF_SIZE);
    }
}
