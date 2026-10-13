/**
 * ==================== ====================           ==================== ====================
 *                                         总线舵机 (ZP 系列) 驱动
 * ==================== ====================           ==================== ====================
 *
 * 舵机型号：杭州众灵 ZP 系列单总线回读数字舵机（ZP15S/ZP15D/ZP20S/ZP20D/ZP30D）
 * 通信方式：串口 ASCII 文本指令，单总线半双工，波特率 115200
 *
 *   - 供电：4.8~8.4V（大电流，禁止 USB 供电，建议 7.4V 锂电）
 *   - 级联：最多 255 个，默认 ID=0，255=广播地址
 *   - 指令格式：#<ID 3位>P<PWM 4位>T<TIME 4位>!   （'#' 头，'!' 尾固定）
 *
 * 接线（UART4 半双工，双开关隔离收发）：
 *   - UART4_TX = PC10，UART4_RX = PC11（115200 8N1）
 *   - 发送开关 ServoTx_Switch = PC9 ：高电平导通 / 低电平断开
 *   - 接收开关 ServoRx_Switch = PA11：高电平导通 / 低电平断开
 *   - 发送命令：TX 导通(PC9=H)、RX 断开(PA11=L) → 单向发送，无回显
 *   - 回读应答：TX 断开(PC9=L)、RX 导通(PA11=H) → 单向接收
 *
 * 本文件当前实现：UART4 半双工收发切换底层 + ASCII 指令拼接 + 位置/ID 应答解析。
 *
 * @author 杭州众灵 ZP 协议 | 移植/重写：鸢 (OpenClaw) 2026-10-01
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 协议常量 ==================== */

#define SERVO_UART            (&huart4)       /* 舵机挂在 UART4 */

/* 半双工收发开关（高导通/低断开） */
#define SERVO_TX_SW_PORT     ServoTx_Switch_GPIO_Port
#define SERVO_TX_SW_PIN      ServoTx_Switch_Pin
#define SERVO_RX_SW_PORT     ServoRx_Switch_GPIO_Port
#define SERVO_RX_SW_PIN      ServoRx_Switch_Pin

#define SERVO_BROADCAST_ID    255             /* 广播地址 */
#define SERVO_DEFAULT_ID      0               /* 出厂默认 ID */
#define SERVO_TEST_ID         2               /* 测试目标 ID */

#define SERVO_PWM_MIN         500             /* PWM 最小值 */
#define SERVO_PWM_CENTER      1500            /* PWM 中值（0 位基准） */
#define SERVO_PWM_MAX         2500            /* PWM 最大值 */
#define SERVO_TIME_MAX        9999            /* 时间上限(ms) */

#define SERVO_RX_BUF_SIZE     64              /* 回读接收缓存 */

/* 按 ID 缓存最近位置读回值的容量（缓存 1..SERVO_POS_CACHE_N-1 号）。
 * 本工程只用到 1/2/3 号，给到 8 留余量。 */
#define SERVO_POS_CACHE_N     8

/* ==================== 半双工方向 ==================== */

typedef enum {
    SERVO_DIR_TX = 0,           /* 切到发送：TX 开关导通，RX 开关断开 */
    SERVO_DIR_RX = 1,           /* 切到接收：RX 开关导通，TX 开关断开 */
} servo_dir_t;

/* ==================== 回读应答类型 ==================== */

typedef enum {
    SERVO_REPLY_NONE      = 0,  /* 无应答/未解析 */
    SERVO_REPLY_POSITION  = 1,  /* 位置 #<ID>P<PWM>! */
    SERVO_REPLY_ID        = 2,  /* ID 应答 #<ID>P! */
    SERVO_REPLY_OK        = 3,  /* 操作成功 #OK! */
    SERVO_REPLY_TV        = 4,  /* 温度电压 #<ID>T<温度>V<电压>! */
} servo_reply_kind_t;

typedef struct {
    servo_reply_kind_t kind;    /* 应答类型 */
    uint8_t id;                 /* 舵机 ID（POSITION/ID/TV 应答中有） */
    uint16_t pwm;               /* 位置 PWM 值（POSITION） */
    int16_t  temp_x10;          /* 温度 x10（℃），TV 应答 */
    int16_t  volt_x10;          /* 电压 x10（V），TV 应答 */
} servo_reply_t;

/* ==================== 函数声明 ==================== */

/* ---- 通信底层 ---- */
void Servo_UartInit(void);                        /* 初始化 UART4 半双工接收 */
void Servo_SetDir(servo_dir_t dir);               /* 切换收发方向（PC9/PA11） */
void Servo_SendString(const char *str);           /* 切 TX 发送字符串，发完切回 RX */
uint16_t Servo_Available(void);                   /* 已收字节数 */
uint16_t Servo_Read(uint8_t *buf, uint16_t len);  /* 读出已收字节 */
void Servo_RxCplt(uint8_t byte);                  /* 由接收回调调用，写入缓存 */
void Servo_RxCpltISR(void);                       /* 中断回调入口 */

/* ---- 指令封装（拼接 ASCII 并发送） ----
 * ★ 前两个是【含零位偏移】的常规入口，绝大多数调用点用它。
 *   带 Raw 的是【绕过偏移】的通道：标定自身、回读测试、诊断用（见下方「零位偏移」）。 */
void Servo_SetAngle(uint8_t id, uint16_t pwm);                              /* 立即控制位置（含偏移） */
void Servo_SetAngleTime(uint8_t id, uint16_t pwm, uint16_t time_ms);        /* 带时间控制位置（含偏移） */
void Servo_SetAngleRaw(uint8_t id, uint16_t pwm);                           /* 同上，【不含】偏移 */
void Servo_SetAngleTimeRaw(uint8_t id, uint16_t pwm, uint16_t time_ms);     /* 同上，【不含】偏移 */
void Servo_ReadAngle(uint8_t id);                                           /* 请求回读位置 */
void Servo_ReadTempVolt(uint8_t id);                                       /* 请求回读温度+电压 */
void Servo_ProbeId(uint8_t id);                                             /* 探测 ID 是否在线 */
void Servo_ChangeId(uint8_t old_id, uint8_t new_id);                        /* 修改 ID */
void Servo_Unlock(uint8_t id);                                              /* 释放扭力 */
void Servo_Lock(uint8_t id);                                                /* 恢复扭力 */
void Servo_SetMin(uint8_t id);                                              /* 设置最小值（当前位设为最小值） */
void Servo_SetMax(uint8_t id);                                              /* 设置最大值（当前位设为最大值） */
void Servo_SetInitPos(uint8_t id);                                          /* 设置启动初始位置（当前位设为初始值，默认1500） */
void Servo_SetInitResume(uint8_t id);                                       /* 恢复初始值（启动恢复力矩） */
void Servo_SendGroup(const char *body);                                     /* 成组同步发送：body=多条 #..! 拼接，自动包 {G0000 .. } */

/* ---- PWM 夹爪舵机（独立一路：PD12 / TIM4_CH1，50Hz） ---- */
/* 与总线舵机（UART4 ASCII）无关，走独立 PWM 输出。
 * 角度 0~270° ↔ 500~2500us；TIM4 配置 PSC=71 / ARR=19999 → 1 count = 1 us。 */
void PWM_Servo_SetPulseUs(uint16_t us);     /* 直接设定脉宽（us），钳位 500~2500 */
void PWM_Servo_SetAngle(float deg);         /* 设定角度（°），0~270 */

/* ---- 回读解析 ---- */
void Servo_Feed(uint8_t byte);            /* 喂入回读字节（由调用方驱动） */
void Servo_Poll(void);                    /* 轮询解析完整应答帧（主循环调用） */
uint8_t Servo_HasReply(void);             /* 是否有一帧新应答待取 */
servo_reply_t Servo_GetReply(void);       /* 取出最新解析结果 */

/* ---- 到位查询（执行层用：取代「发命令 + 睡固定时间」）---- */

/**
 * @brief 舵机 id 是否已到 target_pwm（|读回值 - 目标| <= tol）
 * @retval 1=到位；0=未到位；-1=该 ID 未缓存或还没读到过位置
 * @note   读的是 Servo_ReadAngle() 回读并缓存的**实测值**，不是命令值。
 *         调用前得先让它读过一次（主循环 50ms 轮询已在做）。
 */
int Servo_IsReached(uint8_t id, uint16_t target_pwm, uint16_t tol);

/** 最近一次读回的 PWM；0 = 该 ID 还没读到过。给 UI 显示用。
 *  ★ 返回【原始】实测值，不含零位偏移 —— 标定要靠它读舵机的真实物理位置。 */
uint16_t Servo_LastPos(uint8_t id);

/* ---- 零位偏移（机械臂整体平移） ----
 * 标定时把机械臂摆到收起态、记下实测 PWM，偏移 = 记忆值 − arm_config 的收起态基准。
 * 之后所有下发与到位判据都自动带上它。恒等时全 0，行为与不启用时完全一致。
 * 持久化由 param_store 负责（掉电保存）。 */
void    Servo_SetZeroOffset(uint8_t id, int16_t off);
int16_t Servo_GetZeroOffset(uint8_t id);
void    Servo_ClearZeroOffset(void);   /* 全部归零（参数校验失败时的回退路径） */

/* ---- 三路臂舵机（P1/P2/P3）的力矩使能 ----
 * 标定时要人工摆臂，而舵机通电时是抱死的、掰不动，所以先失力。
 * ★ 状态只能靠软件跟踪：舵机的锁定状态在它自己内部，协议里没有查询命令。
 *   上电默认「有力」（舵机上电即保持），与硬件一致。 */
void    Servo_SetArmTorque(uint8_t on);   /* 1=锁定(有力)，0=失力(可手动摆) */
uint8_t Servo_GetArmTorque(void);         /* 1=有力，0=失力 */

/* ==================== 外部变量 ==================== */
extern volatile servo_reply_t servo_last_reply;            /* 最近一次解析结果（跨任务共享，volatile） */

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
