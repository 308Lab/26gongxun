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

/* ==================== 视觉协议 v1.4 语义化发送 ==================== */

/* 请求扫码（0x10，LEN=0） */
void Transmit_ReqScan(void)
{
    TransmitFrame_t frame;
    frame.cmd = HOST_CMD_REQ_SCAN;
    frame.len = 0;
    Transmit_Frame(&frame);
}

/* 观察三物体（0x20/0x30/0x40，LEN=0） */
void Transmit_ObsThree(uint8_t cmd)
{
    TransmitFrame_t frame;
    frame.cmd = cmd;
    frame.len = 0;
    Transmit_Frame(&frame);
}

/* 开始追踪（0x22/0x25/0x32/0x42，LEN=2，DATA=控制周期 Hz int16 大端） */
void Transmit_TrackBegin(uint8_t cmd, uint16_t rate_hz)
{
    TransmitFrame_t frame;
    frame.cmd = cmd;
    frame.len = VISION_TRACK_RATE_LEN;   /* 2 字节 */
    frame.data[0] = (uint8_t)((rate_hz >> 8) & 0xFF);  /* 高八位 */
    frame.data[1] = (uint8_t)(rate_hz & 0xFF);         /* 低八位 */
    Transmit_Frame(&frame);
}

/* 停止追踪（0x24/0x27/0x34/0x44，LEN=0） */
void Transmit_TrackStop(uint8_t cmd)
{
    TransmitFrame_t frame;
    frame.cmd = cmd;
    frame.len = 0;
    Transmit_Frame(&frame);
}

/* 末端位姿上报（0x14，LEN=6，xyz int16 大端，单位 mm） */
void Transmit_PoseReport(int16_t x_mm, int16_t y_mm, int16_t z_mm)
{
    TransmitFrame_t frame;
    frame.cmd = HOST_CMD_POSE_REPORT;
    frame.len = VISION_XYZ_LEN;   /* 6 字节 */
    frame.data[0] = (uint8_t)((x_mm >> 8) & 0xFF);
    frame.data[1] = (uint8_t)(x_mm & 0xFF);
    frame.data[2] = (uint8_t)((y_mm >> 8) & 0xFF);
    frame.data[3] = (uint8_t)(y_mm & 0xFF);
    frame.data[4] = (uint8_t)((z_mm >> 8) & 0xFF);
    frame.data[5] = (uint8_t)(z_mm & 0xFF);
    Transmit_Frame(&frame);
}
