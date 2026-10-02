/**
 * ==================== ====================            ==================== ====================
 *                                          Host通信解析
 * ==================== ====================            ==================== ====================
 *
 * 必须与 Host.c / Host.h 一齐使用。
 *
 * 功能使用：主循环调用 CommunicateToHost()，解析完成后记得清除 parsed_flag。
 *
 * @author ZQU_JG3085_25Cheng | 移植：鸢 (OpenClaw)
 */

#ifndef __COMMPARSER_H
#define __COMMPARSER_H

#include "main.h"
#include "Host.h"

/* ==================== 变量声明 ==================== */

extern uint8_t comm_error_flag;   // 通信错误标志位
extern uint8_t parsed_flag;       // 解析完成标志位

/* ==================== 函数声明 ==================== */

void CMD_Parse(serial_frame_t *frame);
void Data_Parse(serial_frame_t *frame);
void CommParse(serial_frame_t *frame);
uint8_t CommStatus(void);
void CommunicateToHost(void);

#endif /* __COMMPARSER_H */
