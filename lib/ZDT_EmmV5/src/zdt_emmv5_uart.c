/**
 * @file    zdt_emmv5_uart.c
 * @brief   ZDT EmmV5 UART接口实现
 * @author  Ryan
 * @date    2026-03-26
 */

#include "zdt_emmv5_uart.h"
#include "zdt_emmv5_config.h"
#include <string.h>

/* ============================================
 * 私有变量
 * ============================================ */
static ZDT_UART_HandleTypeDef *g_uart_handle = NULL;
static ZDT_RingBuffer_t g_rx_buffer;

/* ============================================
 * 函数实现
 * ============================================ */

/**
 * @brief  初始化UART和环形缓冲区
 */
void ZDT_UART_Init(ZDT_UART_HandleTypeDef *huart)
{
    g_uart_handle = huart;
    memset(&g_rx_buffer, 0, sizeof(g_rx_buffer));

    /* 启动接收中断 */
    HAL_UART_Receive_IT(g_uart_handle, (uint8_t*)&g_rx_buffer.buffer[g_rx_buffer.write_idx], 1);
}

/**
 * @brief  发送命令帧
 */
void ZDT_UART_SendCmd(uint8_t *data, uint16_t len)
{
    if (g_uart_handle == NULL) {
        return;
    }
    HAL_UART_Transmit(g_uart_handle, data, len, ZDT_UART_TIMEOUT);
}

/**
 * @brief  发送单个字节
 */
void ZDT_UART_SendByte(uint8_t data)
{
    if (g_uart_handle == NULL) {
        return;
    }
    HAL_UART_Transmit(g_uart_handle, &data, 1, ZDT_UART_TIMEOUT);
}

/**
 * @brief  从环形缓冲区读取一字节
 */
int ZDT_UART_ReadByte(uint8_t *data)
{
    uint16_t read_idx = g_rx_buffer.read_idx;
    uint16_t write_idx = g_rx_buffer.write_idx;

    /* 缓冲区为空 */
    if (read_idx == write_idx) {
        return -1;
    }

    *data = g_rx_buffer.buffer[read_idx];
    g_rx_buffer.read_idx = (read_idx + 1) % ZDT_RINGBUFFER_SIZE;

    return 0;
}

/**
 * @brief  获取环形缓冲区数据长度
 */
uint16_t ZDT_UART_GetDataLen(void)
{
    int16_t len = (int16_t)(g_rx_buffer.write_idx - g_rx_buffer.read_idx);
    if (len < 0) {
        len += ZDT_RINGBUFFER_SIZE;
    }
    return (uint16_t)len;
}

/**
 * @brief  清空环形缓冲区
 */
void ZDT_UART_ClearBuffer(void)
{
    g_rx_buffer.write_idx = 0;
    g_rx_buffer.read_idx = 0;
}

/**
 * @brief  UART接收中断处理函数
 * @note   在USART_IRQHandler中调用
 */
void ZDT_UART_RxCpltCallback(void)
{
    if (g_uart_handle == NULL) {
        return;
    }

    /* 移动写指针 */
    g_rx_buffer.write_idx = (g_rx_buffer.write_idx + 1) % ZDT_RINGBUFFER_SIZE;

    /* 防止缓冲区溢出 */
    if (g_rx_buffer.write_idx == g_rx_buffer.read_idx) {
        /* 缓冲区满，移动读指针丢弃最老的数据 */
        g_rx_buffer.read_idx = (g_rx_buffer.read_idx + 1) % ZDT_RINGBUFFER_SIZE;
    }

    /* 继续接收 */
    HAL_UART_Receive_IT(g_uart_handle, (uint8_t*)&g_rx_buffer.buffer[g_rx_buffer.write_idx], 1);
}

/**
 * @brief  获取接收句柄
 */
ZDT_UART_HandleTypeDef* ZDT_UART_GetHandle(void)
{
    return g_uart_handle;
}
