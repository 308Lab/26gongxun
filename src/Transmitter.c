#include "Transmitter.h"
#include "Host.h"

/* 计算 XOR 值 */
static uint8_t calc_xor(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t xor_val = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/* 发送底层：阻塞发送整段数据（USART3） */
void Transmit_Raw(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, data, len, 100);
}

/* 协议发送：按 0xA5|CMD|LEN|DATA|XOR 封帧并发送 */
void Transmit_Frame(TransmitFrame_t *frame)
{
    uint8_t xor_val = calc_xor(frame->cmd, frame->len, frame->data);
    uint8_t buf[TX_FRAME_MAX_LEN];
    uint8_t idx = 0;

    buf[idx++] = SERIAL_HEADER;   // 0xA5
    buf[idx++] = frame->cmd;
    buf[idx++] = frame->len;

    for (uint8_t i = 0; i < frame->len; i++) {
        buf[idx++] = frame->data[i];
    }

    buf[idx++] = xor_val;

    Transmit_Raw(buf, idx);
}

/* 快捷指令·答复 */
void Transmit_Ack(uint8_t ack_cmd)
{
    TransmitFrame_t frame;
    frame.cmd = 0xFF;
    frame.len = 1;
    frame.data[0] = ack_cmd;
    Transmit_Frame(&frame);
}

/* 快捷指令·心跳包 */
void Transmit_Heartbeat(void)
{
    TransmitFrame_t frame;
    frame.cmd = 0xFE;
    frame.len = 0;
    Transmit_Frame(&frame);
}
