#ifndef __GYRO_JY61P_H
#define __GYRO_JY61P_H

#include "main.h"

/* ============================================================
 * JY61P 六轴姿态传感器 · 串口驱动（STM32 HAL 版）
 *
 * 协议：帧头 0x55 + TYPE + 4*短数据(低8+高8) + 校验和，共 11 字节
 *   TYPE 0x50=时间 0x51=加速度 0x52=角速度 0x53=角度 0x54=磁场
 * 角度解算：angle = (short)((H<<8)|L) / 32768.0 * 180.0  （度，±180）
 *
 * 用法：
 *   1) 调用 Gyro_Init() 初始化（内部可直接启动接收）
 *   2) 在 HAL_UART_RxCpltCallback 里调用 Gyro_FeedByte(byte) 喂字节
 *   3) 需要数据时 Gyro_GetAngles() 读 roll/pitch/yaw
 * ============================================================ */

/* 帧头 */
#define JY61P_HEADER  0x55

/* TYPE 定义 */
#define JY61P_TYPE_ACC   0x51  /* 加速度 */
#define JY61P_TYPE_GYRO  0x52  /* 角速度 */
#define JY61P_TYPE_ANGLE 0x53  /* 角度 */

/* 帧长（固定 11 字节） */
#define JY61P_FRAME_LEN  11

/* 角度数据（单位：度） */
typedef struct
{
    float roll;    /* 横滚角 X，±180° */
    float pitch;   /* 俯仰角 Y，±180° */
    float yaw;     /* 偏航角 Z，±180°（转弯用这个） */
} Gyro_Angles_t;

/* 完整数据（可选扩展：加速度/角速度） */
typedef struct
{
    Gyro_Angles_t angle;
} Gyro_Data_t;

/* ==================== 接口 ==================== */

/* 初始化（传入已 MX_ 初始化的串口句柄，并启动中断接收） */
void Gyro_Init(UART_HandleTypeDef *huart);

/* 从串口中断回调喂一个字节进来，内部做状态机解析 */
void Gyro_FeedByte(uint8_t byte);

/* 串口接收完成回调入口：喂字节 + 续接收（单字节中断必须续上，否则断流） */
void Gyro_ReceiveHandler(UART_HandleTypeDef *huart);

/* 读取最新角度 */
Gyro_Angles_t Gyro_GetAngles(void);

/* 判断最近一帧角度数据是否新鲜（可用于超时检测） */
uint8_t Gyro_IsDataFresh(void);

#endif /* __GYRO_JY61P_H */
