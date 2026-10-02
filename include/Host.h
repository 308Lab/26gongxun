/**
 * ==================== ====================           ==================== ====================
 *                                          上下位机通信 (Host 2.0)
 * ==================== ====================           ==================== ====================
 *
 * 移植自 ZQU_JG3085_25Cheng 的 TI-LaunchPad MSPM0G3507 版 Host_2.0，
 * 改为 STM32F103VET6 + HAL 库，使用 USART3（PB10=TX / PB11=RX，115200 8N1）。
 *
 * 帧格式： 0xA5 | CMD | LEN | DATA[0..LEN-1] | XOR
 *   - 帧头 0xA5，帧同步字
 *   - CMD  指令码
 *   - LEN  数据体字节数 (0~32)
 *   - XOR  = CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]
 *
 * 使用：
 *   1. main.c 中调用 Host_Init()（使能 USART3 逐字节接收中断）
 *   2. HAL_UART_RxCpltCallback 中把 USART3 收到的字节喂给 serial_receive_byte()
 *   3. 主循环调用 CommunicateToHost() 消费整帧；处理完记得清 parsed_flag
 *
 * @author ZQU_JG3085_25Cheng | 移植：鸢 (OpenClaw)
 */

#ifndef __HOST_H
#define __HOST_H

#include "main.h"

/* ==================== 协议常量 ==================== */
#define SERIAL_HEADER           0xA5    // 帧头固定值(可修改)
#define SERIAL_MAX_DATA_LEN     32      // 数据体最大字节数(可修改)
#define SERIAL_FRAME_MAX_LEN    (1 + 1 + 1 + SERIAL_MAX_DATA_LEN + 1)  // 最大帧长(不可修改)

/* ==================== 指令定义 ==================== */

/**
 * 指令集（药品搬运车 / 反恐排爆救援机器人 语义），按需增删。
 * 参考协议文档：直行/转弯/急停/状态查询/状态回复/心跳/ACK。
 */
typedef enum {
    HOST_CMD_GO        = 0x01,   // 直行一个路段（速度/距离由 MCU 编码器闭环预设）
    HOST_CMD_TURN      = 0x02,   // 原地旋转：DATA[0]=角度(0~180), DATA[1]=方向(0x00右/0x01左)
    HOST_CMD_STOP      = 0x06,   // 急停，无条件停止所有电机
    HOST_CMD_STATUS_Q  = 0x07,   // 状态查询 (上位机 -> MCU)
    HOST_CMD_STATUS_R  = 0x81,   // 状态回复 (MCU -> 上位机)，4B 载荷
    HOST_CMD_HEARTBEAT = 0xFE,   // 心跳包
    HOST_CMD_ACK       = 0xFF,   // 应答包
} serial_cmd_t;

/* ==================== 接收状态机 ==================== */

typedef enum {
    SERIAL_IDLE    = 0xA5,    // 空闲状态，等待帧头 0xA5
    SERIAL_CMD,               // 已收帧头，等待指令类型
    SERIAL_LEN,               // 已收指令，等待数据长度
    SERIAL_DATA,              // 已收长度，正在收数据体
    SERIAL_XOR,               // 数据体收完，等待校验字节
    SERIAL_EXECUTE,           // 整帧校验通过，等待主循环执行
} serial_state_t;

/* ==================== 帧数据结构 ==================== */

typedef struct {
    uint8_t cmd;                        // 指令类型
    uint8_t len;                        // 数据体长度
    uint8_t data[SERIAL_MAX_DATA_LEN];  // 数据体内容
    uint8_t xor_byte;                   // 收到的 XOR 校验值
} serial_frame_t;

/* ==================== 变量声明 ==================== */

extern serial_frame_t  rx_frame;        // 帧缓冲区
extern serial_state_t  rx_state;        // 状态机，初始为 IDLE
extern uint8_t         rx_data_cnt;     // 数据体计数器
extern uint8_t         rx_byte_cache;   // USART3 逐字节接收缓存（定义于 main.c）

/* ==================== 函数声明 ==================== */

void Host_Init(void);
uint8_t serial_calc_xor(uint8_t cmd, uint8_t len, uint8_t *data);
uint8_t serial_check_xor(serial_frame_t *frame);
void serial_receive_byte(uint8_t byte, serial_frame_t *frame,
                         serial_state_t *state, uint8_t *data_cnt);

#endif /* __HOST_H */
