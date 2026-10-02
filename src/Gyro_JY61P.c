#include "Gyro_JY61P.h"
#include <string.h>

/* ==================== 内部状态 ==================== */

static UART_HandleTypeDef *gyro_uart = NULL;   /* 绑定的串口 */

static uint8_t  rx_byte = 0;                    /* 单字节接收缓冲 */

/* 状态机缓冲 */
static uint8_t  frame_buf[JY61P_FRAME_LEN];     /* 帧缓冲 */
static uint8_t  frame_idx = 0;                  /* 当前已收到字节数 */

/* 解析结果 */
static Gyro_Angles_t angles = {0, 0, 0};        /* 最新角度 */
static uint32_t last_update_tick = 0;           /* 最近一次有效帧的时刻 */

/* 内部函数前向声明 */
static void Gyro_ParseFrame(void);

/* ==================== 初始化 ==================== */

void Gyro_Init(UART_HandleTypeDef *huart)
{
    gyro_uart = huart;
    frame_idx = 0;
    memset(frame_buf, 0, sizeof(frame_buf));

    /* 启动中断接收：一次收 1 字节 */
    if (gyro_uart != NULL)
    {
        HAL_UART_Receive_IT(gyro_uart, &rx_byte, 1);
    }
}

/* ==================== 喂字节（状态机） ==================== */

void Gyro_FeedByte(uint8_t byte)
{
    /* 状态机：等待帧头 0x55 */
    if (frame_idx == 0)
    {
        if (byte != JY61P_HEADER)
        {
            return;               /* 非帧头，丢弃，重新等 */
        }
        frame_buf[frame_idx++] = byte;
        return;
    }

    /* 正常收帧 */
    frame_buf[frame_idx++] = byte;

    /* 收满一帧，解析 */
    if (frame_idx >= JY61P_FRAME_LEN)
    {
        Gyro_ParseFrame();
        frame_idx = 0;            /* 复位，等下一帧 */
    }
}

/* ==================== 帧解析 ==================== */

static void Gyro_ParseFrame(void)
{
    uint8_t  type;
    uint8_t  sum_calc;
    uint8_t  sum_recv;
    uint8_t  i;
    int16_t  val1, val2, val3;

    /* 基本校验：帧头 */
    if (frame_buf[0] != JY61P_HEADER)
    {
        return;
    }

    /* 校验和：SUM = 0x55 + TYPE + 前 8 个数据字节，取低 8 位 */
    sum_calc = 0;
    for (i = 0; i < (JY61P_FRAME_LEN - 1); i++)
    {
        sum_calc += frame_buf[i];
    }
    sum_calc &= 0xFF;
    sum_recv = frame_buf[JY61P_FRAME_LEN - 1];

    if (sum_calc != sum_recv)
    {
        return;                    /* 校验失败，丢弃 */
    }

    type = frame_buf[1];

    /* 角度帧 0x53：Roll / Pitch / Yaw / 版本 */
    if (type == JY61P_TYPE_ANGLE)
    {
        /* 高字节先转有符号，再移位，保证负数正确 */
        val1 = (int16_t)(((int16_t)frame_buf[3] << 8) | frame_buf[2]);  /* Roll */
        val2 = (int16_t)(((int16_t)frame_buf[5] << 8) | frame_buf[4]);  /* Pitch */
        val3 = (int16_t)(((int16_t)frame_buf[7] << 8) | frame_buf[6]);  /* Yaw */

        angles.roll  = (float)val1 / 32768.0f * 180.0f;
        angles.pitch = (float)val2 / 32768.0f * 180.0f;
        angles.yaw   = (float)val3 / 32768.0f * 180.0f;

        last_update_tick = HAL_GetTick();
    }
    /* 其它类型暂不处理（需要可自行扩展） */
}

/* ==================== 对外接口 ==================== */

/*
 * 串口接收完成回调专用入口：
 *   单字节中断收满一字节后必须重新 HAL_UART_Receive_IT 续上，
 *   否则只收 1 字节就断流。rx_byte 是 static，封装在此处对外。
 * 用法：在 HAL_UART_RxCpltCallback 里，判断是本陀螺仪串口后调用本函数。
 */
void Gyro_ReceiveHandler(UART_HandleTypeDef *huart)
{
    Gyro_FeedByte(rx_byte);                    /* 喂这一字节进状态机 */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);   /* 续上，等下一字节 */
}

Gyro_Angles_t Gyro_GetAngles(void)
{
    return angles;
}

uint8_t Gyro_IsDataFresh(void)
{
    /* 超过 200ms 无新数据视为不新鲜 */
    if ((HAL_GetTick() - last_update_tick) < 200)
    {
        return 1;
    }
    return 0;
}
