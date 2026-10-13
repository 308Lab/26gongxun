#include "CommParser.h"
#include "Transmitter.h"
#include "Vision.h"
#include "path_runner.h"
#include "Car_control.h"

uint8_t comm_error_flag;    // 通信错误标志位
uint8_t parsed_flag;        // 解析完成标志位

/* ==================== 应答与动作辅助 ==================== */

/* 心跳（协议 §3.1，2026-10-13 修订）：**收到不回**。
 *
 * 主动方是 MCU —— Host_Tick() 每 200ms 主动发一帧 0xFE；视觉收到会回一帧。
 * MCU 收到的 0xFE 必然是视觉对自己刚才那帧的**回声**，再回就无限对打
 * （实测：心跳帧瞬间淹没所有任务帧，0x21 连一次都挤不进去）。
 *
 * 注意：心跳【不刷新业务超时】—— Vision 层的 last_recv_tick 只由业务回包
 * （0x21/0x23/0x26/0x31/0x33/0x41/0x43）刷新。否则视觉业务线程卡死但心跳还在回时，
 * §2.5 的兜底永远不会触发。 */
static void HandleHeartbeat(void)
{
    /* 有意留空：见上。 */
}

/* ==================== 指令解析函数 ==================== */
/**
 * 在该函数中解析具体指令，并置对应标志位 / 派发到业务层。
 *
 * 协议 v1.4 落地：
 *   - 视觉 → MCU 的带语义指令：解析后回 ACK（§2.1 单向应答）。
 *   - 收到 ACK(0xFF)、心跳(0xFE) 不回 ACK。
 *   - 未识别指令 → comm_error_flag=1（由上层的 CommunicateToHost 判断）。
 */
void CMD_Parse(serial_frame_t *frame)
{
    switch (frame->cmd)
    {
        /* ================= 底盘控制（旧上位机语义） ================= */
        case HOST_CMD_GO:        /* 直行 */
            /* TODO: 接入底盘直行（PathRunner / RunTask 触发） */
            break;

        case HOST_CMD_TURN:      /* 转弯：DATA[0]=角度, DATA[1]=方向 */
            /* TODO: 接入底盘转弯 */
            break;

        case HOST_CMD_STOP:      /* 急停 */
            /* TODO: 调用底盘急停 Stop()（Car_control） */
            break;

        case HOST_CMD_STATUS_Q:  /* 状态查询：回复状态包 0x81 */
            /* TODO: 组织 4B 状态载荷并 Transmit 0x81 */
            break;

        /* ================= 全局指令（视觉语义） ================= */
        case HOST_CMD_SCAN_DONE:      /* 0x11 扫码完成（协议 §3.2：LEN=3，带三位数） */
            Vision_OnScanDone(frame->data, frame->len);
            Transmit_Ack(HOST_CMD_SCAN_DONE);
            break;

        case HOST_CMD_IDENT_ERR:      /* 0x12 识别错误（DATA=错误码 1B） */
            Vision_OnIdentErr(frame->len >= 1 ? frame->data[0] : 0);
            Transmit_Ack(HOST_CMD_IDENT_ERR);
            break;

        case HOST_CMD_TARGET_LOST:    /* 0x13 目标丢失/出视野（纯信号） */
            Vision_OnTargetLost();
            Transmit_Ack(HOST_CMD_TARGET_LOST);
            break;

        /* ================= 排爆系列（0x2x）================= */
        case HOST_CMD_A_RET_3:        /* 0x21 回三目标（LEN=19） */
            if (Vision_OnThreeTarget(frame->data, frame->len) == 0)
                Transmit_Ack(HOST_CMD_A_RET_3);
            break;

        /* ★ 流式坐标帧（0x23/0x26/0x33/0x43）**不回 ACK**（协议 §2.1，2026-10-13 明确）：
         *   它们是 10~100Hz 的高频数据流，不是指令。逐帧回 ACK 会在 115200 上把回程
         *   带宽持续占住，而闭环判到位靠坐标本身，ACK 没有任何作用。 */
        case HOST_CMD_A_TRACK_POS:    /* 0x23 流式单目标（LEN=6） */
            Vision_OnSingleTarget(frame->data, frame->len);
            break;

        case HOST_CMD_A_PLACE_POS:    /* 0x26 流式排爆桶（LEN=6） */
            Vision_OnSingleTarget(frame->data, frame->len);
            break;

        /* ================= 反恐系列（0x3x）================= */
        case HOST_CMD_B_RET_3:        /* 0x31 回三目标（LEN=19） */
            if (Vision_OnThreeTarget(frame->data, frame->len) == 0)
                Transmit_Ack(HOST_CMD_B_RET_3);
            break;

        case HOST_CMD_B_TRACK_POS:    /* 0x33 流式单目标（LEN=6），不回 ACK（见上） */
            Vision_OnSingleTarget(frame->data, frame->len);
            break;

        case HOST_CMD_B_TARGET_DOWN:  /* 0x35 目标倒下（LEN=0） */
            Vision_OnTargetDown();
            Transmit_Ack(HOST_CMD_B_TARGET_DOWN);
            break;

        /* ================= 救援系列（0x4x）================= */
        case HOST_CMD_C_RET_3:        /* 0x41 回三目标（LEN=19） */
            if (Vision_OnThreeTarget(frame->data, frame->len) == 0)
                Transmit_Ack(HOST_CMD_C_RET_3);
            break;

        case HOST_CMD_C_TRACK_POS:    /* 0x43 流式单目标（LEN=6），不回 ACK（见上） */
            Vision_OnSingleTarget(frame->data, frame->len);
            break;

        /* ================= 心跳 / 应答 ================= */
        case HOST_CMD_HEARTBEAT:      /* 0xFE 心跳：回 0xFE，不回 ACK */
            HandleHeartbeat();
            break;

        case HOST_CMD_ACK:            /* 0xFF 收到 ACK：不回 ACK（协议 §2.1） */
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
