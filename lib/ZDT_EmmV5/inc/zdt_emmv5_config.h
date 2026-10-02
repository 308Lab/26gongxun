/**
 * @file    zdt_emmv5_config.h
 * @brief   ZDT EmmV5 驱动库配置文件
 * @author  Ryan
 * @date    2026-03-26
 */

#ifndef __ZDT_EMMV5_CONFIG_H
#define __ZDT_EMMV5_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================
 * 串口配置
 * ============================================ */
#define ZDT_UART_BAUDRATE         115200  /* 波特率 */
#define ZDT_UART_TIMEOUT          1000    /* 超时时间 (ms) */

/* ============================================
 * 环形缓冲区配置
 * ============================================ */
#define ZDT_RINGBUFFER_SIZE       128     /* 缓冲区大小 */

/* ============================================
 * 命令帧配置
 * ============================================ */
#define ZDT_FRAME_END             0x6B    /* 帧结束符 */
#define ZDT_MAX_CMD_SIZE          20      /* 最大命令帧长度 */
#define ZDT_MAX_RESP_SIZE         64      /* 最大响应帧长度 */

/* ============================================
 * 使能/禁用开关
 * ============================================ */
#define ZDT_ENABLE                1
#define ZDT_DISABLE               0

/* ============================================
 * 方向定义
 * ============================================ */
#define ZDT_DIR_CW                0       /* 顺时针 */
#define ZDT_DIR_CCW               1       /* 逆时针 */

/* ============================================
 * 相对/绝对标志
 * ============================================ */
#define ZDT_RELATIVE              0       /* 相对运动 */
#define ZDT_ABSOLUTE             1       /* 绝对运动 */

/* ============================================
 * 控制模式
 * ============================================ */
#define ZDT_CTRL_OFF              0       /* 关闭 */
#define ZDT_CTRL_OPEN_LOOP        1       /* 开环 */
#define ZDT_CTRL_CLOSED_LOOP      2       /* 闭环 */
#define ZDT_CTRL_PULSE_DIR        3       /* 脉冲方向模式 */

/* ============================================
 * 归零模式
 * ============================================ */
#define ZDT_ORIGIN_SINGLE         0       /* 单圈回零 */
#define ZDT_ORIGIN_MULTI          1       /* 多圈回零 */
#define ZDT_ORIGIN_SENSOR         2       /* 原点传感器 */
#define ZDT_ORIGIN_SENSOR_LIMIT   3       /* 原点传感器+限位 */

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_EMMV5_CONFIG_H */
