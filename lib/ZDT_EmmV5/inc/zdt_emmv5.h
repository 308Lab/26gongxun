/**
 * @file    zdt_emmv5.h
 * @brief   ZDT EmmV5 步进电机驱动库主头文件
 * @author  Ryan
 * @date    2026-03-26
 */

#ifndef __ZDT_EMMV5_H
#define __ZDT_EMMV5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zdt_emmv5_config.h"
#include "zdt_emmv5_uart.h"
#include "stm32f1xx_hal.h"

/* ============================================
 * 系统参数枚举
 * ============================================ */
typedef enum {
    ZDT_PARAM_VER = 0,      /* 固件版本 */
    ZDT_PARAM_RL,           /* 旋转方向 */
    ZDT_PARAM_PID,          /* PID参数 */
    ZDT_PARAM_VBUS,         /* 母线电压 */
    ZDT_PARAM_CPHA,         /* 相位 */
    ZDT_PARAM_ENCL,         /* 编码器值 */
    ZDT_PARAM_TPOS,         /* 目标位置 */
    ZDT_PARAM_VEL,          /* 实时转速 */
    ZDT_PARAM_CPOS,         /* 实时位置 */
    ZDT_PARAM_PERR,         /* 位置误差 */
    ZDT_PARAM_FLAG,         /* 状态标志 */
    ZDT_PARAM_CONF,         /* 配置参数 */
    ZDT_PARAM_STATE,        /* 系统状态 */
    ZDT_PARAM_ORG           /* 原点状态 */
} ZDT_SysParams_t;

/* ============================================
 * 控制句柄
 * ============================================ */
typedef struct {
    uint8_t addr;                       /* 驱动器地址 */
    ZDT_UART_HandleTypeDef *huart;      /* UART句柄 */
    volatile uint8_t rx_complete;       /* 接收完成标志 */
    uint8_t rx_data[ZDT_MAX_RESP_SIZE]; /* 接收数据缓存 */
    uint16_t rx_len;                    /* 接收数据长度 */
} ZDT_HandleTypeDef;

/* ============================================
 * 函数声明
 * ============================================ */

/**
 * @brief  初始化驱动库
 * @param  handle: 控制句柄
 * @param  huart: UART句柄
 * @retval None
 */
void ZDT_EmmV5_Init(ZDT_HandleTypeDef *handle, ZDT_UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  当前位置置零
 * @param  addr: 驱动器地址
 * @retval None
 */
void ZDT_EmmV5_Reset_CurPos_To_Zero(uint8_t addr);

/**
 * @brief  清除堵转故障
 * @param  addr: 驱动器地址
 * @retval None
 */
void ZDT_EmmV5_Reset_Clog_Pro(uint8_t addr);

/**
 * @brief  读取系统参数
 * @param  addr: 驱动器地址
 * @param  param: 系统参数类型
 * @retval None
 */
void ZDT_EmmV5_Read_Sys_Params(uint8_t addr, ZDT_SysParams_t param);

/**
 * @brief  修改控制模式
 * @param  addr: 驱动器地址
 * @param  store: 是否存储 (false/true)
 * @param  mode: 控制模式 (0=关闭, 1=开环, 2=闭环, 3=脉冲方向)
 * @retval None
 */
void ZDT_EmmV5_Modify_Ctrl_Mode(uint8_t addr, uint8_t store, uint8_t mode);

/**
 * @brief  使能控制
 * @param  addr: 驱动器地址
 * @param  enable: 使能状态 (false/true)
 * @param  sync: 同步标志 (false/true)
 * @retval None
 */
void ZDT_EmmV5_En_Control(uint8_t addr, uint8_t enable, uint8_t sync);

/**
 * @brief  速度模式控制
 * @param  addr: 驱动器地址
 * @param  dir: 方向 (0=CW, 1=CCW)
 * @param  vel: 速度 (RPM)
 * @param  acc: 加速度 (0-255)
 * @param  sync: 同步标志
 * @retval None
 */
void ZDT_EmmV5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t sync);

/**
 * @brief  位置模式控制
 * @param  addr: 驱动器地址
 * @param  dir: 方向 (0=CW, 1=CCW)
 * @param  vel: 速度 (RPM)
 * @param  acc: 加速度 (0-255)
 * @param  pulses: 脉冲数
 * @param  relative: 相对/绝对标志 (false=相对, true=绝对)
 * @param  sync: 同步标志
 * @retval None
 */
void ZDT_EmmV5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                           uint32_t pulses, uint8_t relative, uint8_t sync);

/**
 * @brief  立即停止
 * @param  addr: 驱动器地址
 * @param  sync: 同步标志
 * @retval None
 */
void ZDT_EmmV5_Stop_Now(uint8_t addr, uint8_t sync);

/**
 * @brief  同步运动
 * @param  addr: 驱动器地址
 * @retval None
 */
void ZDT_EmmV5_Synchronous_motion(uint8_t addr);

/**
 * @brief  设置原点
 * @param  addr: 驱动器地址
 * @param  store: 是否存储
 * @retval None
 */
void ZDT_EmmV5_Origin_Set_O(uint8_t addr, uint8_t store);

/**
 * @brief  修改归零参数
 * @param  addr: 驱动器地址
 * @param  store: 是否存储
 * @param  mode: 归零模式
 * @param  dir: 归零方向
 * @param  vel: 归零速度 (RPM)
 * @param  timeout: 超时时间 (ms)
 * @param  sensor_vel: 传感器探测速度 (RPM)
 * @param  sensor_max: 传感器探测最大距离
 * @param  sensor_timeout: 传感器探测超时 (ms)
 * @param  limit_enable: 限位开关使能
 * @retval None
 */
void ZDT_EmmV5_Origin_Modify_Params(uint8_t addr, uint8_t store, uint8_t mode,
                                    uint8_t dir, uint16_t vel, uint32_t timeout,
                                    uint16_t sensor_vel, uint16_t sensor_max,
                                    uint16_t sensor_timeout, uint8_t limit_enable);

/**
 * @brief  触发归零
 * @param  addr: 驱动器地址
 * @param  mode: 归零模式
 * @param  sync: 同步标志
 * @retval None
 */
void ZDT_EmmV5_Origin_Trigger_Return(uint8_t addr, uint8_t mode, uint8_t sync);

/**
 * @brief  强制中断归零
 * @param  addr: 驱动器地址
 * @retval None
 */
void ZDT_EmmV5_Origin_Interrupt(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_EMMV5_H */
