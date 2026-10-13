/**
 * @file    Vision.h
 * @brief   视觉/上位机通信业务状态层（协议 v1.4 落地载体）
 *
 * 定位：承接 Host/CommParser 解析出的视觉指令，维护通信层的业务状态，
 *       供未来「任务骨架（A/B/C/D）」与「视觉伺服闭环」消费。
 *
 * 现状（2026-10-07）：
 *   - 通信栈（Host 收 / Transmitter 发 / CommParser 解析）已落地协议 v1.4 全指令。
 *   - 本层先搭数据缓存 + 状态机 + 占位接口；具体任务动作（FK、SE(2) 粗调位、
 *     视觉伺服闭环）因任务骨架尚未落地，动作函数先占位（TODO）。
 *
 * 关键约定（对齐协议 v1.4）：
 *   - 坐标 int16 大端，单位 mm，精度 1mm；基准点：排爆/救援=夹爪 TCP，反恐=激光。
 *   - 到位/对准判断权 = MCU。
 *   - 流式追踪：MCU 发「开始追踪(带周期Hz)」→ 视觉连续下传坐标 → MCU 判到位
 *     发「停止追踪」。
 *   - 视野丢失兜底（2026-10-13 执线人定案，覆盖 §2.5）：观察单目标 500ms 超时
 *     抬大臂+回上一帧 PWM 后重发；连续 5 次丢失 → 回展开态，循环重试，永不停机。
 */

#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include "Host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 三目标坐标缓存（一次 0x21/0x31/0x41 回包 = 19 字节，左/中/右）
 * ============================================================ */
#define VISION_3TARGET_N  3   /* 左/中/右 三目标 */

/* ============================================================
 * 识别错误码（0x12 的 DATA[0]，协议 §3.1b 错误码表，与上位机共用）
 * 视觉报错后回空闲态，重试权在 MCU（协议 §2.2）。
 * ============================================================ */
#define VISION_ERR_NONE               0x00  /* 保留：不要当成有效错误码 */
#define VISION_ERR_QR_BAD_PAYLOAD     0x01  /* 码扫到了，但内容不是合法三位数 */
#define VISION_ERR_CALIB_MISSING      0x02  /* 标定/手眼变换缺失或与分辨率不匹配 */
#define VISION_ERR_DETECTOR_NOT_READY 0x03  /* 本任务/本阶段的检测器未就绪 */
#define VISION_ERR_TARGET_NOT_THREE   0x04  /* 检出目标数 != 3，定不了左/中/右 */
#define VISION_ERR_TRACK_TIMEOUT      0x05  /* 追踪段达到最大时长（§3.1c，60s） */
#define VISION_ERR_DEPTH_INVALID      0x06  /* 目标深度无效或超出有效测距范围 */

/* 单目标坐标（mm，int16） */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vision_xyz_t;

/* 三目标回包解析结果 */
typedef struct {
    vision_xyz_t target[VISION_3TARGET_N];  /* [0]=左 [1]=中 [2]=右 */
    uint8_t      target_id;                 /* 选定的抓取/打击目标：1左/2中/3右 */
    uint8_t      valid;                     /* 1=本次回包已解析成功 */
} vision_3target_t;

/* ============================================================
 * 视觉任务系列枚举（对齐协议 CMD 系列号）
 * ============================================================ */
typedef enum {
    VISION_SERIES_NONE = 0,
    VISION_SERIES_A    = 1,   /* 排爆 0x2x */
    VISION_SERIES_B    = 2,   /* 反恐 0x3x */
    VISION_SERIES_C    = 3,   /* 救援 0x4x */
} vision_series_t;

/* ============================================================
 * 流式追踪状态（MCU 主控）
 * ============================================================ */
typedef enum {
    V_TRACK_IDLE = 0,      /* 空闲：未观察、未追踪 */
    V_TRACK_WAIT_OBS3,     /* 阶段1：已发「观察三物体」，等三目标回包（协议 §2.5 阶段1） */
    V_TRACK_RUNNING,       /* 阶段2：追踪中，持续接收流式坐标 */
    V_TRACK_WAIT_STOP,     /* 已发「停止追踪」，等视觉停流 */
} vision_track_state_t;

/* ============================================================
 * 视觉通信业务状态（全局单例）
 * ============================================================ */
typedef struct {
    vision_3target_t obs3;                 /* 最近一次三目标回包 */
    vision_track_state_t track_state;      /* 追踪状态机 */
    vision_series_t      series;           /* 当前追踪所属任务系列 */
    vision_xyz_t         last_pos;         /* 最近一次流式单目标坐标 */
    uint8_t              last_pos_valid;   /* last_pos 是否有新值 */
    uint16_t             track_rate_hz;    /* 当前追踪控制周期（Hz） */
    uint8_t              track_cmd;        /* 当前追踪命令码（0x22/0x25/0x32/0x42），超时重发用它 */
    uint32_t             last_recv_tick;   /* 最近一次收到视觉回包的 tick（超时判据） */

    /* 扫码完成（0x11）事件标志：Vision_OnScanDone 置 1，任务层消费后清 0 */
    uint8_t              scan_done;        /* 1=收到扫码完成 */

    /* 反恐目标倒下（0x35）事件标志：Vision_OnTargetDown 置 1，任务层开激光后消费 */
    uint8_t              target_down;      /* 1=视觉判靶已倒下（协议 §3.4） */

    /* 视野丢失兜底计数（2026-10-13 定案：连续 5 次→回展开态，永不停机） */
    uint8_t              retry_cnt;        /* 当前观察单目标的重发次数（<3） */
    uint8_t              lost_cnt;         /* 连续出视野次数（>=5 回展开态） */

    /* 位姿上报（0x14）定时节拍 */
    uint32_t             pose_tick;        /* 上次上报 tick */

    /* 识别错误（0x12）记录：码值留档供 OLED/调试定位，动作停摆由任务层兜底 */
    uint8_t              last_err_code;    /* 最近一次错误码（0=无，见 VISION_ERR_*） */
    uint16_t             err_cnt;          /* 累计收到的错误帧数 */
} vision_state_t;

extern vision_state_t g_vision;

/* ============================================================
 * 接口：CommParser 解析成功后调用（写入缓存 + 状态迁移）
 * ============================================================ */

/* 三目标回包（0x21/0x31/0x41，LEN=19）落地：解析并缓存，返回 0=成功/-1=长度错 */
int8_t Vision_OnThreeTarget(const uint8_t *data, uint8_t len);

/* 流式单目标坐标（0x23/0x33/0x43，LEN=6）落地：解析并缓存 last_pos */
int8_t Vision_OnSingleTarget(const uint8_t *data, uint8_t len);

/* 识别错误（0x12）落地 */
void Vision_OnIdentErr(uint8_t err_code);

/* 目标丢失（0x13）落地：进入视野丢失兜底 */
void Vision_OnTargetLost(void);

/* 扫码完成（0x11）落地 */
void Vision_OnScanDone(void);

/* 反恐目标倒下（0x35）落地：置 target_down 事件标志，任务层据此关激光（协议 §3.4） */
void Vision_OnTargetDown(void);

/* ============================================================
 * 主动发送（MCU → 视觉）语义封装：内部转调 Transmitter
 * ============================================================ */

void Vision_ReqScan(void);                          /* 0x10 请求扫码 */
void Vision_ObsThree(vision_series_t series);      /* 0x20/0x30/0x40 观察三物体 */
void Vision_TrackBegin(vision_series_t series, uint16_t rate_hz); /* 开始追踪（抓取段） */
void Vision_TrackStop(vision_series_t series);     /* 停止追踪（抓取段） */
void Vision_PlaceBegin(uint16_t rate_hz);          /* 0x25 开始追踪（排爆放置段） */
void Vision_PlaceStop(void);                       /* 0x27 停止追踪（排爆放置段） */

/* ============================================================
 * 周期驱动（放逐帧 tick 处）：视觉超时重发 / 位姿上报节拍
 *
 * 覆盖协议 §2.5 的两个阶段，行为一致但重发的命令不同：
 *   阶段1 观察三物体（V_TRACK_WAIT_OBS3）→ 重发 0x20/0x30/0x40；
 *   阶段2 追踪（V_TRACK_RUNNING）        → 重发 0x22/0x25/0x32/0x42。
 * 统一节奏：500ms 无回包 → 重发本阶段命令 ×3 → 仍无 → lost_cnt++ 并回 IDLE
 * （由 formal_mode 任务层兑现兜底动作）。 */
void Vision_Tick(void);

/* 位姿上报（0x14）落地点：由任务层调 FK 解算 TCP 后写入 xyz 上报。
 * TODO: 待任务骨架接入 arm_fk，当前占位（x/y/z 用缓存/0）。 */
void Vision_ReportPose(int16_t x_mm, int16_t y_mm, int16_t z_mm);

/* 查询并消费「扫码完成」标志（0x11 事件）：
 * 返回 1=发生过扫码完成（并清零），0=未发生。供正式模式 A 任务轮询。 */
uint8_t Vision_ConsumeScanDone(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_H */
