#include "Host.h"
#include "Transmitter.h"

serial_frame_t  rx_frame;                // 帧缓冲区
serial_state_t  rx_state = SERIAL_IDLE;  // 状态机，初始为 IDLE
uint8_t         rx_data_cnt = 0;         // 数据体计数器

/* ==================== 心跳主动播发（协议 §3.1） ==================== */

/* 心跳周期：200ms（5Hz）。
 * 协议 §3.1 定 MCU 为心跳主动方；周期必须【明显小于】接收方的失联判定阈值
 * （视觉侧 link_timeout = 500ms），给两侧各留 2~3 帧余量，漏一帧不会误判失联。 */
#define HOST_HEARTBEAT_PERIOD_MS   200

static uint32_t s_heartbeat_tick = 0;

/* 初始化串口：使能 USART3 逐字节接收中断 */
void Host_Init(void)
{
    /* 启动逐字节接收：每收到 1 字节进 HAL_UART_RxCpltCallback，
       在其中续接下一次 Receive_IT 并喂入状态机 */
    HAL_UART_Receive_IT(&huart3, &rx_byte_cache, 1);
    s_heartbeat_tick = HAL_GetTick();
}

/* 周期驱动：主动播发心跳。放主循环里每 tick 调一次（协议 §3.1 主动方 = MCU）。
 *
 * 为什么必须有一方主动发：双方都只在「收到才回」的话，链路上永远不会出现心跳帧，
 * 对端的失联判定就永远停在「从未收到过任何帧」，看门狗形同虚设。 */
void Host_Tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_heartbeat_tick) >= HOST_HEARTBEAT_PERIOD_MS)
    {
        s_heartbeat_tick = now;
        Transmit_Heartbeat();
    }
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
                /* LEN 超限 = 这一帧不合法。同样的道理：这个字节可能就是下一帧的帧头
                 * （0xA5 > 32，必然落进这里），别把它一并丢掉。 */
                *state = (byte == SERIAL_HEADER) ? SERIAL_CMD : SERIAL_IDLE;
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
                /* 校验失败：这一帧作废。但当前这个字节本身可能是【下一帧的帧头】
                 * —— 数据里出现 0xA5、而它又恰好是 XOR 位时就会这样。直接回 IDLE
                 * 会把它吞掉，于是下一帧从 CMD 开始解析、整条错位下去。
                 * 所以按"它是不是帧头"决定回到 CMD 还是 IDLE。 */
                *state = (byte == SERIAL_HEADER) ? SERIAL_CMD : SERIAL_IDLE;
            }
            break;

        case SERIAL_EXECUTE:
            /* 主循环处理完帧后会设回 IDLE；期间意外收到的字节忽略 */
            break;
    }
}
