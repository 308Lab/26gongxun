#include "CommParser.h"

uint8_t comm_error_flag;    // 通信错误标志位
uint8_t parsed_flag;        // 解析完成标志位

/* ==================== 指令解析函数 ==================== */
/**
 * 在该函数中解析具体指令，并置对应标志位 / 派发到业务层。
 * 由上位机下发的运动/急停/查询指令在此消费；业务动作按底盘语义接入。
 */
void CMD_Parse(serial_frame_t *frame)
{
    switch (frame->cmd)
    {
        case HOST_CMD_GO:        /* 直行 */
            /* TODO: 接入底盘直行指令（例如 g_cmd 下发或 PathRunner 触发） */
            break;

        case HOST_CMD_TURN:      /* 转弯：DATA[0]=角度, DATA[1]=方向 */
            /* TODO: 接入底盘转弯指令 */
            break;

        case HOST_CMD_STOP:      /* 急停 */
            /* TODO: 调用 Stop() */
            break;

        case HOST_CMD_STATUS_Q:  /* 状态查询：回复状态包 */
            /* TODO: Transmit 状态回复（0x81） */
            break;

        default:
            comm_error_flag = 1;   // 未识别指令
            break;
    }
}

/* 数据解析函数 */
void Data_Parse(serial_frame_t *frame)
{
    /* 按需对 frame->data 做长度/字段二次解析 */
    (void)frame;
}

void CommParse(serial_frame_t *frame)
{
    CMD_Parse(frame);
    if (comm_error_flag)
    {
        return;
    }
    Data_Parse(frame);
}

/* 判断通信是否接收完毕：0=没有，1=有 */
uint8_t CommStatus(void)
{
    return (rx_state == SERIAL_EXECUTE);
}

/* ==================== 主函数接入： 上下位机通信协议解析 ==================== */

/**
 * @brief 上下位机通信解析，主循环唯一调用
 * @note 需要在主循环手动清除 parsed_flag
 */
void CommunicateToHost(void)
{
    if (CommStatus())
    {
        comm_error_flag = 0;
        CommParse(&rx_frame);
        rx_state = SERIAL_IDLE;
        if (!comm_error_flag)
        {
            parsed_flag = 1;
        }
    }
}
