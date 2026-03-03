/**
 ****************************************************************************************************
 * @file        jetson_usart.c
 * @author      RMP-WL100 通信中间件
 * @version     V1.0
 * @date        2026-03-04
 * @brief       针对 Jetson ROS2 控制的高速串口(USART3_DMA)底层驱动
 * @note        硬件连接: PB10=TX (接Jetson的RX), PB11=RX (接Jetson的TX) \n
 *              架构机制: 串口空闲中断 (IDLE) + DMA 循环接收引擎防丢包 \n
 *              协议格式: [0xA5, 0x5A, VX_H, VX_L, VY_H, VY_L, VZ_H, VZ_L, XorCheck, 0xEE]
 ****************************************************************************************************
 */

#include "jetson_usart.h"
#include <string.h>

/******************************************************************************************/
/* 核心底层句柄与缓存阵列 */

UART_HandleTypeDef g_uart3_handle;       /* UART3 句柄 */
DMA_HandleTypeDef g_dma_uart3_rx_handle; /* DMA 句柄 */

/* DMA直接写入的内存泳池 (全局不可被覆盖) */
static uint8_t g_jetson_rx_buf[JETSON_RX_BUF_SIZE] __attribute__((aligned(32))); 

/* 存放解析完成的高净值速度参数容器 */
static jetson_ctrl_cmd_t g_jetson_cmd = {0, 0, 0, 0}; 

/******************************************************************************************/

/**
 * @brief       暴露容器的安全指针供主函数拉取
 */
jetson_ctrl_cmd_t* get_jetson_cmd_ptr(void)
{
    return &g_jetson_cmd;
}

/**
 * @brief       协议校验与剥离解析子函数
 * @param       buf: 数据头指针
 * @param       len: 捕获到这一串收到的总长度
 */
static void jetson_protocol_decode(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    
    /* 防御:连一包基本长度都不满足,直接掐掉 */
    if (len < JETSON_FRAME_LEN) return; 

    /* 在这堆不确定长的数据流里，滑窗寻找合法的帧头 [A5 5A] */
    for (i = 0; i <= (len - JETSON_FRAME_LEN); i++)
    {
        /* --- 第1步: 帧头匹配 --- */
        if (buf[i] == 0xA5 && buf[i+1] == 0x5A)
        {
            /* --- 第2步: 帧尾匹配 (加速过滤) --- */
            if (buf[i + 9] == 0xEE)
            {
                /* --- 第3步: 核心异或 Check 运算 --- */
                uint8_t cal_check = 0;
                for (uint8_t j = 0; j < 8; j++)
                {
                    cal_check ^= buf[i + j];
                }

                /* --- 第4步: 校验核对无误，开始开箱大端序解码 --- */
                if (cal_check == buf[i + 8])
                {
                    /* ROS传来的16位整型（高位在前） */
                    g_jetson_cmd.target_vx = (int16_t)((buf[i+2] << 8) | buf[i+3]);
                    g_jetson_cmd.target_vy = (int16_t)((buf[i+4] << 8) | buf[i+5]);
                    g_jetson_cmd.target_vz = (int16_t)((buf[i+6] << 8) | buf[i+7]);
                    
                    /* 立起新数据标记，通知 FreeRTOS 主任务有热腾腾的数据送达 */
                    g_jetson_cmd.frame_valid = 1;

                    /* 已经剥了一帧完整的就算成功，防粘连可跳跃本次解析指针 */
                    i += (JETSON_FRAME_LEN - 1); 
                }
            }
        }
    }
}

/**
 * @brief       USART3 初始化并开启 DMA 空闲接收模式
 * @param       baudrate: 波特率 (推荐 115200)
 */
void jetson_usart_init(uint32_t baudrate)
{
    /* 1. UART3与外设引脚底层配置 */
    /* 注：HAL库内部会去调用 HAL_UART_MspInit 开启时钟与 GPIO (在sys.c或此文件中复写均可,这里为封装独立我们内联接管) */
    
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;
    /* PB10 = TX */
    gpio_init_struct.Pin = GPIO_PIN_10;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    /* PB11 = RX */
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP; /* 推荐复用推挽拉高，抗悬空乱码 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* 2. UART3 核心参数 */
    g_uart3_handle.Instance = USART3;
    g_uart3_handle.Init.BaudRate = baudrate;
    g_uart3_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart3_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart3_handle.Init.Parity = UART_PARITY_NONE;
    g_uart3_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart3_handle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart3_handle);

    /* 3. DMA RX 接收流配置 (F407: USART3_RX 挂载在 DMA1_Stream1_Channel4) */
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    g_dma_uart3_rx_handle.Instance = DMA1_Stream1;
    g_dma_uart3_rx_handle.Init.Channel = DMA_CHANNEL_4;
    g_dma_uart3_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_dma_uart3_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_uart3_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_uart3_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_uart3_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    /* 设置为纯正循环接收，像传输带一样永远接收不必每次死停重开 */
    g_dma_uart3_rx_handle.Init.Mode = DMA_CIRCULAR; 
    g_dma_uart3_rx_handle.Init.Priority = DMA_PRIORITY_HIGH;
    g_dma_uart3_rx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    HAL_DMA_Init(&g_dma_uart3_rx_handle);

    /* 将 DMA 句柄链接到 UART 底层结构身上 */
    __HAL_LINKDMA(&g_uart3_handle, hdmarx, g_dma_uart3_rx_handle);

    /* 4. 开启空闲中断（关键灵魂）以及让 DMA 开始干活 */
    __HAL_UART_ENABLE_IT(&g_uart3_handle, UART_IT_IDLE); 
    
    /* 让硬件开启 DMA 循环死等，任何进来的比特都丢到 g_jetson_rx_buf 里 */
    HAL_UART_Receive_DMA(&g_uart3_handle, g_jetson_rx_buf, JETSON_RX_BUF_SIZE);

    /* 5. 放行中断控制器 NVIC 回调响应 */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0); /* 避开了高抢占，暂时给主线逻辑让路 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * @brief       中断服务硬核钩子 (当一连串指令全进DMA突然哑火停顿时触发)
 */
void USART3_IRQHandler(void)
{
    /* 查询是否确实是空闲断档引发的 IDLE Interrupt */
    if (__HAL_UART_GET_FLAG(&g_uart3_handle, UART_FLAG_IDLE) != RESET)
    {
        uint16_t current_rx_len = 0;

        /* 清除 IDLE 标志位铁律：先读 SR 状态寄存器，再读 DR 数据寄存器！*/
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart3_handle);

        /* 【精华：此时 DMA 还在循环圈里跑，我们停下引擎计算这次一共搬了多长？】 */
        /* 获取 DMA 目前未填满的剩余空间数，相减可知本次包长 */
        current_rx_len = JETSON_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(g_uart3_handle.hdmarx);

        /* 提取有效数据并丢给状态机解析！ */
        if (current_rx_len > 0)
        {
            jetson_protocol_decode(g_jetson_rx_buf, current_rx_len);
        }

        /* 这次消化完了，如果遇到极其复杂的工业要求这里需要重启 DMA。
         * 由于设置了 DMA_CIRCULAR 循环模式，外加它是数组覆写，
         * 我们为了图极端稳定性：每次 IDLE 干脆复位干回0霸道重写数组。 */
        HAL_UART_AbortReceive(&g_uart3_handle);
        HAL_UART_Receive_DMA(&g_uart3_handle, g_jetson_rx_buf, JETSON_RX_BUF_SIZE);
    }
    
    /* 其他默认错误（溢出等）依然交回HAL库内部清理 */
    HAL_UART_IRQHandler(&g_uart3_handle);
}
