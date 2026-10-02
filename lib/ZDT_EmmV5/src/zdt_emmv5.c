/**
 * @file    zdt_emmv5.c
 * @brief   ZDT EmmV5 步进电机驱动实现
 * @author  Ryan
 * @date    2026-03-26
 */

#include "zdt_emmv5.h"
#include "zdt_emmv5_uart.h"
#include "zdt_emmv5_config.h"

/* ============================================
 * 私有变量
 * ============================================ */
static ZDT_HandleTypeDef *g_handle = NULL;

/* ============================================
 * 函数实现
 * ============================================ */

/**
 * @brief  初始化驱动库
 */
void ZDT_EmmV5_Init(ZDT_HandleTypeDef *handle, ZDT_UART_HandleTypeDef *huart, uint8_t addr)
{
    g_handle = handle;
    g_handle->huart = huart;
    g_handle->addr = addr;
    g_handle->rx_complete = 0;
    g_handle->rx_len = 0;

    /* 初始化UART和环形缓冲区 */
    ZDT_UART_Init(huart);
}

/**
 * @brief  当前位置置零
 */
void ZDT_EmmV5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0A, 0x6D, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 4);
}

/**
 * @brief  清除堵转故障
 */
void ZDT_EmmV5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0E, 0x52, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 4);
}

/**
 * @brief  读取系统参数
 */
void ZDT_EmmV5_Read_Sys_Params(uint8_t addr, ZDT_SysParams_t param)
{
    uint8_t cmd[4] = {0};
    uint8_t i = 0;

    cmd[i++] = addr;

    switch (param) {
        case ZDT_PARAM_VER:    cmd[i++] = 0x1F; break;
        case ZDT_PARAM_RL:     cmd[i++] = 0x20; break;
        case ZDT_PARAM_PID:    cmd[i++] = 0x21; break;
        case ZDT_PARAM_VBUS:   cmd[i++] = 0x24; break;
        case ZDT_PARAM_CPHA:   cmd[i++] = 0x27; break;
        case ZDT_PARAM_ENCL:   cmd[i++] = 0x31; break;
        case ZDT_PARAM_TPOS:   cmd[i++] = 0x33; break;
        case ZDT_PARAM_VEL:    cmd[i++] = 0x35; break;
        case ZDT_PARAM_CPOS:   cmd[i++] = 0x37; break;
        case ZDT_PARAM_FLAG:  cmd[i++] = 0x3A; break;
        case ZDT_PARAM_ORG:    cmd[i++] = 0x3B; break;
        case ZDT_PARAM_CONF:   cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
        case ZDT_PARAM_STATE:  cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
        default: return;
    }

    cmd[i++] = ZDT_FRAME_END;
    ZDT_UART_SendCmd(cmd, i);
}

/**
 * @brief  修改控制模式
 */
void ZDT_EmmV5_Modify_Ctrl_Mode(uint8_t addr, uint8_t store, uint8_t mode)
{
    uint8_t cmd[6] = {addr, 0x46, 0x69, store, mode, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 6);
}

/**
 * @brief  使能控制
 */
void ZDT_EmmV5_En_Control(uint8_t addr, uint8_t enable, uint8_t sync)
{
    uint8_t cmd[6] = {addr, 0xF3, 0xAB, enable, sync, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 6);
}

/**
 * @brief  速度模式控制
 */
void ZDT_EmmV5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t sync)
{
    uint8_t cmd[8] = {
        addr, 0xF6, dir,
        (uint8_t)(vel >> 8), (uint8_t)(vel & 0xFF),
        acc, sync, ZDT_FRAME_END
    };
    ZDT_UART_SendCmd(cmd, 8);
}

/**
 * @brief  位置模式控制
 */
void ZDT_EmmV5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                           uint32_t pulses, uint8_t relative, uint8_t sync)
{
    uint8_t cmd[13] = {
        addr, 0xFD, dir,
        (uint8_t)(vel >> 8), (uint8_t)(vel & 0xFF),
        acc,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)(pulses & 0xFF),
        relative, sync, ZDT_FRAME_END
    };
    ZDT_UART_SendCmd(cmd, 13);
}

/**
 * @brief  立即停止
 */
void ZDT_EmmV5_Stop_Now(uint8_t addr, uint8_t sync)
{
    uint8_t cmd[5] = {addr, 0xFE, 0x98, sync, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 5);
}

/**
 * @brief  同步运动
 */
void ZDT_EmmV5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0xFF, 0x66, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 4);
}

/**
 * @brief  设置原点
 */
void ZDT_EmmV5_Origin_Set_O(uint8_t addr, uint8_t store)
{
    uint8_t cmd[5] = {addr, 0x93, 0x88, store, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 5);
}

/**
 * @brief  修改归零参数
 */
void ZDT_EmmV5_Origin_Modify_Params(uint8_t addr, uint8_t store, uint8_t mode,
                                   uint8_t dir, uint16_t vel, uint32_t timeout,
                                   uint16_t sensor_vel, uint16_t sensor_max,
                                   uint16_t sensor_timeout, uint8_t limit_enable)
{
    uint8_t cmd[20] = {
        addr, 0x4C, 0xAE, store, mode, dir,
        (uint8_t)(vel >> 8), (uint8_t)(vel & 0xFF),
        (uint8_t)(timeout >> 24), (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8), (uint8_t)(timeout & 0xFF),
        (uint8_t)(sensor_vel >> 8), (uint8_t)(sensor_vel & 0xFF),
        (uint8_t)(sensor_max >> 8), (uint8_t)(sensor_max & 0xFF),
        (uint8_t)(sensor_timeout >> 8), (uint8_t)(sensor_timeout & 0xFF),
        limit_enable, ZDT_FRAME_END
    };
    ZDT_UART_SendCmd(cmd, 20);
}

/**
 * @brief  触发归零
 */
void ZDT_EmmV5_Origin_Trigger_Return(uint8_t addr, uint8_t mode, uint8_t sync)
{
    uint8_t cmd[5] = {addr, 0x9A, mode, sync, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 5);
}

/**
 * @brief  强制中断归零
 */
void ZDT_EmmV5_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x9C, 0x48, ZDT_FRAME_END};
    ZDT_UART_SendCmd(cmd, 4);
}
