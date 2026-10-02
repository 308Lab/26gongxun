/**
 * @file    zdt_emmv5_uart.h
 * @brief   ZDT EmmV5 UART接口抽象层
 * @author  Ryan
 * @date    2026-03-26
 */

#ifndef __ZDT_EMMV5_UART_H
#define __ZDT_EMMV5_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zdt_emmv5_config.h"
#include "stm32f1xx_hal.h"

/* ============================================
 * UART句柄typedef
 * ============================================ */
typedef UART_HandleTypeDef ZDT_UART_HandleTypeDef;

/* ============================================
 * Ring Buffer结构体
 * ============================================ */
typedef struct {
    uint8_t buffer[ZDT_RINGBUFFER_SIZE];
    volatile uint16_t write_idx;  /* 写指针 */
    volatile uint16_t read_idx;   /* 读指针 */
} ZDT_RingBuffer_t;

/* ============================================
 * 函数声明
 * ============================================ */

/**
 * @brief  初始化UART和环形缓冲区
 * @param  huart: HAL UART句柄指针
 * @retval None
 */
void ZDT_UART_Init(ZDT_UART_HandleTypeDef *huart);

/**
 * @brief  发送命令帧
 * @param  data: 命令数据
 * @param  len:  数据长度
 * @retval None
 */
void ZDT_UART_SendCmd(uint8_t *data, uint16_t len);

/**
 * @brief  发送单个字节
 * @param  data: 字节数据
 * @retval None
 */
void ZDT_UART_SendByte(uint8_t data);

/**
 * @brief  从环形缓冲区读取一字节
 * @param  data: 读取数据存放位置
 * @retval 0=成功, -1=缓冲区为空
 */
int ZDT_UART_ReadByte(uint8_t *data);

/**
 * @brief  获取环形缓冲区数据长度
 * @retval 数据长度
 */
uint16_t ZDT_UART_GetDataLen(void);

/**
 * @brief  清空环形缓冲区
 * @retval None
 */
void ZDT_UART_ClearBuffer(void);

/**
 * @brief  UART接收中断处理函数
 * @retval None
 */
void ZDT_UART_RxCpltCallback(void);

/**
 * @brief  获取接收句柄
 * @retval UART句柄指针
 */
ZDT_UART_HandleTypeDef* ZDT_UART_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_EMMV5_UART_H */
