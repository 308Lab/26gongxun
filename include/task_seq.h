/**
 * @file    task_seq.h
 * @brief   任务序列引擎（表驱动）—— 取代旧 formal_mode.c 的巨型 switch
 *
 * 设计要点（2026-10-13 与 Ryan 定稿）：
 *   1. 【表驱动】一个任务 = 一张 step 表；引擎负责"发起动作 → 轮询到位 → 超时兜底 → 推进"。
 *      旧实现里 left/mid/right 三份几乎相同的复制粘贴，在这里变成数据。
 *   2. 【到位判据取代固定延时】旧实现是"发舵机命令 + 睡 4 秒"，现在是轮询
 *      Servo_IsReached() / Arm_IsReached() / Car_IsStopped() —— 到位就立刻推进，
 *      没到位就等，超时才兜底。三个查询接口在 Arm.h / Servo.h / Car_control.h 里。
 *   3. 【任务可独立启动】Task_RunAt(点) 从【当前位置】直接去那个点并执行，
 *      不跑前面的路（调试省时间，也能单独测某个任务）。Task_RunAll() 才是全程。
 *
 * 与 path_runner 的分工（不变）：
 *   path_runner 管"把车开到哪个点"，本模块管"到了点之后做什么"。
 *   PathRunner 到点会进 RUN_ST_TASK 并等 task_done；本引擎跑完序列表就回喂它。
 */

#ifndef __TASK_SEQ_H
#define __TASK_SEQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 动作种类 —— 表里每一步做哪件事
 * ============================================================ */
typedef enum {
    ACT_END = 0,      /* 序列表结束标记 */

    /* 舵机（总线，UART4）：一次可同时发起 1~3 个，等它们都到位。
     * 为什么最多 3：本机就 3 个总线舵机（大臂 P1 / 小臂 P2 / 腕 P3）。 */
    ACT_SERVO1,       /* 1 个舵机：sid1 → spwm1 */
    ACT_SERVO2,       /* 2 个舵机同时 */
    ACT_SERVO3,       /* 3 个舵机同时（展开/收起那种"一起动"的场合） */

    ACT_GRIP,         /* 夹爪（独立 PWM 舵机 PD12）：arg = 角度（度） */

    ACT_STEP1,        /* θ1 步进：去累计脉冲 pulses（软累加绝对定位） */
    ACT_STEP1_SLOW,   /* 同上，但 30 RPM（救援段要慢，避免晃动带偏人质） */

    /* 按视觉选定的 target_id（1左/2中/3右）去对应的预存姿态。
     * arg = 0 用排爆(B)的预存位表，arg = 1 用救援(D)的。
     * ★ 旧 formal_mode.c 把 left/mid/right 三份几乎相同的代码复制了三遍，
     *   这一条动作就是为消灭那三份重复而存在的。 */
    ACT_POSE_BY_TARGET,

    /* 视觉交互（协议 v1.4.1）—— 等回包，带超时 */
    ACT_VIS_SCAN,     /* 发 0x10 请求扫码，等 0x11（协议 §3.2：阻塞式，不设超时） */
    ACT_VIS_OBS3_A,   /* 发 0x20 观察三物体（排爆），等 0x21 */
    ACT_VIS_OBS3_C,   /* 发 0x40 观察三物体（救援），等 0x41 */
    ACT_VIS_TRACK_A,  /* 发 0x22 开始追踪（排爆·抓取），进入视觉伺服闭环 */
    ACT_VIS_TRACK_B,  /* 发 0x32 开始追踪（反恐） */
    ACT_VIS_TRACK_C,  /* 发 0x42 开始追踪（救援） */
    ACT_VIS_TRACK_BUCKET, /* 发 0x25 开始追踪（排爆·放置） */
    ACT_VIS_STOP_A,   /* 发 0x24 停止追踪（排爆·抓取） */
    ACT_VIS_STOP_B,   /* 发 0x34 停止追踪（反恐） */
    ACT_VIS_STOP_C,   /* 发 0x44 停止追踪（救援） */
    ACT_VIS_STOP_BUCKET,  /* 发 0x27 停止追踪（排爆·放置） */

    /* 反恐专用：靠【车前后移动】把激光对准靶心（2026-10-13 Ryan 澄清的设计）。
     * 机械臂姿态在这一步之前就锁死了 —— 高度由姿态保证，所以只用视觉偏移的
     * 前后分量去调车，角速度恒为 0：转向会让激光斜射靶面，破坏光电触发。
     * 完成判据 = 偏移 ≤ ARM_KINE_EPS_MM。 */
    ACT_CAR_AIM,

    ACT_LASER_ON,     /* 激光开 */
    ACT_LASER_OFF,    /* 激光关 */

    /* 反恐收尾：等视觉报"靶倒下"（0x35）。协议 §3.4 的循环末两步就是
     * "开激光 → 等 0x35 → 关激光"。★ 必须配超时 —— 视觉没报也不能让激光一直开着。 */
    ACT_WAIT_TARGET_DOWN,

    ACT_DELAY,        /* 纯延时 arg 毫秒（少数没有"到位"可判的场合） */
} task_act_t;

/* 视觉动作的到位判据由引擎按 act 自动选，不用在表里写 */
#define TSEQ_T_DEFAULT   1500U   /* timeout_ms 填 0 时用这个 */

/* ============================================================
 * 一步
 *
 * 字段不少，但表里【一律用指定初始化器】写（见 task_def.c），别按位置填：
 *   {.act = ACT_SERVO2, .sid1 = ARM_TH4_ID, .sid2 = ARM_TH2_ID,
 *    .spwm1 = ARM_POSE_HOME_P3, .spwm2 = ARM_POSE_HOME_P1, .stime_ms = 2000}
 * ============================================================ */
typedef struct {
    uint8_t  act;         /* task_act_t */

    /* 舵机（ACT_SERVO1/2/3 用，sid=0 表示该位不用） */
    uint8_t  sid1, sid2, sid3;
    uint16_t spwm1, spwm2, spwm3;
    uint16_t stime_ms;    /* 舵机运动时间（给舵机固件的"走多久"） */

    /* 步进（ACT_STEP1 用） */
    int32_t  pulses;      /* θ1 累计目标脉冲 */

    /* 通用参数：ACT_GRIP 的角度 / ACT_DELAY 的毫秒 */
    uint16_t arg;

    uint16_t timeout_ms;  /* 该步的到位超时，0 → TSEQ_T_DEFAULT */
} tstep_t;

/* 一个任务 = 一张以 ACT_END 结尾的 step 表 */
typedef struct {
    const char* name;       /* 给 UI/日志看 */
    const tstep_t* steps;
} task_def_t;

/* ============================================================
 * 引擎接口
 * ============================================================ */

/** 从【当前位置】直接去目标点并执行那个任务（点号：0=A 1=B 2=C 3=D）。 */
void Task_RunAt(uint8_t point_idx);

/** 全程：从当前位置去 A，依次 A→B→C→D（沿用 path_runner 的逐点推进）。 */
void Task_RunAll(void);

/** 每控制周期调用一次（放 MainTask 循环里，紧跟 PathRunner_Tick）。 */
void Task_Tick(void);

/** 是否正在跑任务序列（供 OLED/UI 显示）。 */
uint8_t Task_IsBusy(void);

/** 中止当前任务（急停/手动收起用）：停序列表 + 停底盘，不碰 path_runner 的路径。 */
void Task_Abort(void);

/** 当前步在表里的下标（-1 = 没在跑）。给诊断用。 */
int Task_CurrentStep(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_SEQ_H */
