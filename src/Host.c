#include "Host.h"

serial_frame_t  rx_frame;                // 帧缓冲区
serial_state_t  rx_state = SERIAL_IDLE;  // 状态机，初始为 IDLE
uint8_t         rx_data_cnt = 0;         // 数据体计数器

/* 初始化串口：使能 USART3 逐字节接收中断 */
void Host_Init(void)
{
    /* 启动逐字节接收：每收到 1 字节进 HAL_UART_RxCpltCallback，
       在其中续接下一次 Receive_IT 并喂入状态机 */
    HAL_UART_Receive_IT(&huart3, &rx_byte_cache, 1);
}

/* 计算XOR */
uint8_t serial_calc_xor(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t xor_val = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/* 检验收到的帧 XOR 是否正确 */
uint8_t serial_check_xor(serial_frame_t *frame)
{
    uint8_t calc = serial_calc_xor(frame->cmd, frame->len, frame->data);
    return (calc == frame->xor_byte);
}

/* 串口接收状态机 */
void serial_receive_byte(uint8_t byte, serial_frame_t *frame,
                         serial_state_t *state, uint8_t *data_cnt)
{
    switch (*state)
    {
        case SERIAL_IDLE:
            if (byte == SERIAL_HEADER) {
                *state = SERIAL_CMD;
            }
            break;

        case SERIAL_CMD:
            frame->cmd = byte;
            *state = SERIAL_LEN;
            break;

        case SERIAL_LEN:
            frame->len = byte;
            *data_cnt = 0;

            if (byte == 0)
            {
                *state = SERIAL_XOR;
            }
            else if (byte <= SERIAL_MAX_DATA_LEN)
            {
                *state = SERIAL_DATA;
            }
            else
            {
                *state = SERIAL_IDLE;
            }
            break;

        case SERIAL_DATA:
            frame->data[*data_cnt] = byte;
            (*data_cnt)++;
            if (*data_cnt >= frame->len)
            {
                *state = SERIAL_XOR;
            }
            break;

        case SERIAL_XOR:
            frame->xor_byte = byte;

            if (serial_check_xor(frame))
            {
                *state = SERIAL_EXECUTE;
            }
            else
            {
                *state = SERIAL_IDLE;
            }
            break;

        case SERIAL_EXECUTE:
            /* 主循环处理完帧后会设回 IDLE；期间意外收到的字节忽略 */
            break;
    }
}
