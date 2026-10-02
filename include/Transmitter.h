/**
 * ==================== ====================            ==================== ====================
 *                                          Host通信发送
 * ==================== ====================            ==================== ====================
 *
 * 必须与 Host.c / Host.h 一齐使用。
 *
 * 功能使用：
 *   1. Transmit_Frame(TransmitFrame_t *frame);  按协议发送自定义指令
 *   2. Transmit_Ack(uint8_t ack_cmd);           按协议答复某个指令
 *   3. Transmit_Heartbeat(void);                按协议发送心跳包
 *
 * 移植为 STM32 HAL 版，使用 USART3（huart3）。
 *
 * @author ZQU_JG3085_25Cheng | 移植：鸢 (OpenClaw)
 */

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 协议常量 ==================== */

#define TX_MAX_DATA_LEN      32
#define TX_FRAME_MAX_LEN     (1 + 1 + 1 + TX_MAX_DATA_LEN + 1)

/* ==================== 指令帧结构体 ==================== */

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[TX_MAX_DATA_LEN];
} TransmitFrame_t;

/* ==================== 函数声明 ==================== */

void Transmit_Raw(uint8_t *data, uint16_t len);
void Transmit_Frame(TransmitFrame_t *frame);
void Transmit_Ack(uint8_t ack_cmd);
void Transmit_Heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITTER_H */
