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
 * 指令集（视觉/上位机 ↔ MCU 通信，协议 v1.4，2026-10-01 定稿）
 * 参考：`通信协议_视觉与MCU_v1.4_20261001.md`
 *
 * 帧格式：0xA5 | CMD | LEN | DATA[0..LEN-1] | XOR
 *   物理层 USART3（PB10=TX / PB11=RX，115200 8N1）。
 *   校验 XOR = CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]（不含帧头）。
 *
 * 应答：仅 MCU 应答（0xFF ACK）；视觉/上位机不回 ACK；收到 ACK 不回 ACK。
 */
typedef enum {
    /* ---- 底盘控制（旧上位机语义，保留兼容） ---- */
    HOST_CMD_GO        = 0x01,   // 直行一个路段（速度/距离由 MCU 编码器闭环预设）
    HOST_CMD_TURN      = 0x02,   // 原地旋转：DATA[0]=角度(0~180), DATA[1]=方向(0x00右/0x01左)
    HOST_CMD_STOP      = 0x06,   // 急停，无条件停止所有电机
    HOST_CMD_STATUS_Q  = 0x07,   // 状态查询 (上位机 -> MCU)
    HOST_CMD_STATUS_R  = 0x81,   // 状态回复 (MCU -> 上位机)，4B 载荷

    /* ---- 全局指令（v1.4 §3.0） ---- */
    HOST_CMD_REQ_SCAN   = 0x10,  // MCU→视觉：请求扫码（LEN=0）
    HOST_CMD_SCAN_DONE  = 0x11,  // 视觉→MCU：扫码完成（LEN=0，纯信号）
    HOST_CMD_IDENT_ERR  = 0x12,  // 视觉→MCU：识别错误（LEN=1，DATA=错误码）
    HOST_CMD_TARGET_LOST= 0x13,  // 视觉→MCU：目标丢失/出视野（LEN=0，纯信号）
    HOST_CMD_POSE_REPORT= 0x14,  // MCU→上位机：末端位姿上报（LEN=6，xyz int16 大端）

    /* ---- 排爆系列（v1.4 §3.3，0x2x） ---- */
    HOST_CMD_A_OBS_3      = 0x20, // MCU→视觉：观察三物体（排爆）LEN=0
    HOST_CMD_A_RET_3      = 0x21, // 视觉→MCU：回三目标（LEN=19，3组xyz+target_id）
    HOST_CMD_A_TRACK_BEGIN= 0x22, // MCU→视觉：开始追踪（抓取段）LEN=2，控制周期Hz
    HOST_CMD_A_TRACK_POS  = 0x23, // 视觉→MCU：流式回单目标（LEN=6）
    HOST_CMD_A_TRACK_STOP = 0x24, // MCU→视觉：停止追踪（抓取段）LEN=0
    HOST_CMD_A_PLACE_BEGIN= 0x25, // MCU→视觉：开始追踪（放置段）LEN=2，控制周期Hz
    HOST_CMD_A_PLACE_POS  = 0x26, // 视觉→MCU：流式回排爆桶（LEN=6）
    HOST_CMD_A_PLACE_STOP = 0x27, // MCU→视觉：停止追踪（放置段）LEN=0

    /* ---- 反恐系列（v1.4 §3.4，0x3x） ---- */
    HOST_CMD_B_OBS_3      = 0x30, // MCU→视觉：观察三物体（反恐标靶）LEN=0
    HOST_CMD_B_RET_3      = 0x31, // 视觉→MCU：回三目标（LEN=19）
    HOST_CMD_B_TRACK_BEGIN= 0x32, // MCU→视觉：开始追踪（锁定标靶）LEN=2
    HOST_CMD_B_TRACK_POS  = 0x33, // 视觉→MCU：流式回单目标（LEN=6，激光系偏移）
    HOST_CMD_B_TRACK_STOP = 0x34, // MCU→视觉：停止追踪（MCU判对准）LEN=0
    HOST_CMD_B_TARGET_DOWN= 0x35, // 视觉→MCU：目标倒下（LEN=0，关激光）

    /* ---- 救援系列（v1.4 §3.5，0x4x） ---- */
    HOST_CMD_C_OBS_3      = 0x40, // MCU→视觉：观察三物体（救援目标）LEN=0
    HOST_CMD_C_RET_3      = 0x41, // 视觉→MCU：回三目标（LEN=19）
    HOST_CMD_C_TRACK_BEGIN= 0x42, // MCU→视觉：开始追踪（锁定救援目标）LEN=2
    HOST_CMD_C_TRACK_POS  = 0x43, // 视觉→MCU：流式回单目标（LEN=6）
    HOST_CMD_C_TRACK_STOP = 0x44, // MCU→视觉：停止追踪（MCU判到位）LEN=0

    /* ---- 心跳 / 应答 ---- */
    HOST_CMD_HEARTBEAT = 0xFE,   // 心跳包（双向对称：收 0xFE 回 0xFE）
    HOST_CMD_ACK       = 0xFF,   // 应答包（仅 MCU 发，DATA=[被应答 CMD]）
} serial_cmd_t;

/* ==================== 协议数据长度约定（v1.4） ==================== */
#define VISION_XYZ_LEN        6     /* int16 大端 xyz 单目标坐标 = 6 字节 */
#define VISION_3TARGET_LEN    19    /* 3 组 xyz(18B) + target_id(1B) = 19 字节 */
#define VISION_TRACK_RATE_LEN 2     /* 开始追踪帧 DATA = 控制周期 Hz int16 大端 = 2 字节 */

/* 回三目标帧的 target_id 取值（v1.4 §2.4 补充） */
#define VISION_TARGET_LEFT    1     /* 左 */
#define VISION_TARGET_MID     2     /* 中 */
#define VISION_TARGET_RIGHT   3     /* 右 */

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

/* 周期驱动：主动播发心跳（协议 §3.1，MCU 是心跳的主动方，200ms/帧）。
 * 放主循环里每 tick 调一次。 */
void Host_Tick(void);

uint8_t serial_calc_xor(uint8_t cmd, uint8_t len, uint8_t *data);
uint8_t serial_check_xor(serial_frame_t *frame);
void serial_receive_byte(uint8_t byte, serial_frame_t *frame,
                         serial_state_t *state, uint8_t *data_cnt);

#endif /* __HOST_H */
