/**
 * @file    Vision.c
 * @brief   视觉/上位机通信业务状态层（协议 v1.4 落地）
 *
 * 说明：
 *   - 本文件只承载「通信业务状态 + 数据缓存 + 语义发送封装」。
 *   - 收到视觉回包后由 CommParser 调用 Vision_OnXxx 落地缓存 / 迁移状态。
 *   - 任务动作（FK / SE(2) 粗调位 / 视觉伺服闭环 / 开关激光）尚未接入任务骨架，
 *     相关动作先 TODO 占位，避免空实现导致编译告警。
 *   - 坐标一律 int16 大端（Host 已约定），单位 mm。
 */

#include "Vision.h"
#include "Transmitter.h"

/* 视野丢失兜底参数（2026-10-13 执线人定案：永不停机） */
#define VISION_RETRY_MAX        3       /* 观察单目标重发次数上限 */
#define VISION_TIMEOUT_MS       500     /* 等目标回包超时（ms） */
#define VISION_LOST_BACK_CNT    5       /* 连续出视野 5 次 → 回展开态（不再停机） */
#define VISION_POSE_PERIOD_MS   100     /* 位姿上报周期（占位，10Hz） */

/* 默认追踪控制周期（Hz，10~100，协议建议区间内） */
#define VISION_DEFAULT_RATE_HZ  50

vision_state_t g_vision = { 0 };

/* ============================================================
 * 内部小工具：int16 大端解码 / 编码
 * ============================================================ */
static int16_t decode_i16_be(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ============================================================
 * 视觉 → MCU 回包落地
 * ============================================================ */

/* 三目标回包（LEN=19）：3 组 xyz(每组6B) + target_id(1B) */
int8_t Vision_OnThreeTarget(const uint8_t *data, uint8_t len)
{
    if (len != VISION_3TARGET_LEN)
        return -1;

    const uint8_t *p = data;
    for (uint8_t i = 0; i < VISION_3TARGET_N; i++)
    {
        g_vision.obs3.target[i].x = decode_i16_be(p + 0);
        g_vision.obs3.target[i].y = decode_i16_be(p + 2);
        g_vision.obs3.target[i].z = decode_i16_be(p + 4);
        p += 6;
    }
    g_vision.obs3.target_id = *p;   /* 1左/2中/3右 */
    g_vision.obs3.valid = 1;

    /* 观察回包 = 阶段一结束，退出等待态并重置重发/丢失计数 */
    g_vision.track_state = V_TRACK_IDLE;
    g_vision.retry_cnt = 0;
    g_vision.lost_cnt  = 0;
    g_vision.last_recv_tick = HAL_GetTick();

    /* TODO: 依据 target_id 选取预存 SE(2) 粗调位值（任务骨架接入） */

    return 0;
}

/* 流式单目标（LEN=6）：xyz */
int8_t Vision_OnSingleTarget(const uint8_t *data, uint8_t len)
{
    if (len != VISION_XYZ_LEN)
        return -1;

    g_vision.last_pos.x = decode_i16_be(data + 0);
    g_vision.last_pos.y = decode_i16_be(data + 2);
    g_vision.last_pos.z = decode_i16_be(data + 4);
    g_vision.last_pos_valid = 1;
    g_vision.last_recv_tick  = HAL_GetTick();

    /* TODO: 视觉伺服闭环（ik_delta + 下发关节增量）在此消费 last_pos */

    return 0;
}

/* 识别错误（0x12）：记录错误码（协议 §3.1b 错误码表），停摆交给任务层兜底。
 * 视觉报错后回空闲态；重试权在 MCU（协议 §2.2），视觉不自己重试。 */
void Vision_OnIdentErr(uint8_t err_code)
{
    g_vision.last_err_code = err_code;
    g_vision.err_cnt++;
    g_vision.track_state = V_TRACK_IDLE;
}

/* 目标丢失（0x13）：进入视野丢失兜底（不关机，计数由任务层消费） */
void Vision_OnTargetLost(void)
{
    g_vision.lost_cnt++;
    g_vision.track_state = V_TRACK_IDLE;   /* 本轮结束：不再超时重发，等任务层兜底 */
    g_vision.retry_cnt   = 0;              /* 计数一并归零，下一轮从零起 */
    /* 兑现动作（抬大臂+回上一帧 PWM / 连续 5 次回展开态）由 formal_mode 任务层执行 */
}

/* 扫码完成（0x11）：阻塞式握手结束 */
void Vision_OnScanDone(void)
{
    g_vision.scan_done = 1;   /* 置扫码完成标志，供正式模式 A 任务轮询消费 */
}

uint8_t Vision_ConsumeScanDone(void)
{
    uint8_t d = g_vision.scan_done;
    g_vision.scan_done = 0;
    return d;
}

/* 反恐目标倒下（0x35）：置事件标志，由任务层关激光（协议 §3.4）。
 * 激光是任务资源不是通信资源，这层只落事件。 */
void Vision_OnTargetDown(void)
{
    g_vision.target_down  = 1;   /* 任务层消费后清 0 */
    g_vision.track_state  = V_TRACK_IDLE;
}

/* ============================================================
 * MCU → 视觉 主动发送封装
 * ============================================================ */

void Vision_ReqScan(void)
{
    Transmit_ReqScan();
}

/* 按系列发「观察三物体」帧（0x20/0x30/0x40） */
static void Vision_SendObs3(vision_series_t series)
{
    switch (series)
    {
        case VISION_SERIES_A: Transmit_ObsThree(HOST_CMD_A_OBS_3);   break;
        case VISION_SERIES_B: Transmit_ObsThree(HOST_CMD_B_OBS_3);   break;
        case VISION_SERIES_C: Transmit_ObsThree(HOST_CMD_C_OBS_3);   break;
        default: break;
    }
}

/* 开始【新一轮】观察（任务层调用）：重置重发计数并进入阶段1等待态。
 * ★ 早先这里没设 track_state —— 观察段于是没有任何超时/重发，视觉丢一帧就永久卡死。 */
void Vision_ObsThree(vision_series_t series)
{
    Vision_SendObs3(series);
    g_vision.series       = series;
    g_vision.retry_cnt    = 0;
    g_vision.track_state  = V_TRACK_WAIT_OBS3;
    g_vision.last_recv_tick = HAL_GetTick();
}

/* 超时【重发】观察命令：不动 retry_cnt、不改状态，只重发 + 重置计时。
 * （与 Vision_ReTrack 同构：重发若走 Vision_ObsThree，计数会被清零 → 无限重发。） */
static void Vision_ReObs3(void)
{
    Vision_SendObs3(g_vision.series);
    g_vision.last_recv_tick = HAL_GetTick();
}

void Vision_TrackBegin(vision_series_t series, uint16_t rate_hz)
{
    uint8_t cmd;

    if (rate_hz == 0)
        rate_hz = VISION_DEFAULT_RATE_HZ;

    switch (series)
    {
        case VISION_SERIES_A: cmd = HOST_CMD_A_TRACK_BEGIN; break;
        case VISION_SERIES_B: cmd = HOST_CMD_B_TRACK_BEGIN; break;
        case VISION_SERIES_C: cmd = HOST_CMD_C_TRACK_BEGIN; break;
        default: return;   /* 未知系列：不发也不改状态 */
    }
    Transmit_TrackBegin(cmd, rate_hz);

    g_vision.series        = series;
    g_vision.track_rate_hz = rate_hz;
    g_vision.track_cmd     = cmd;              /* 超时重发要原样再发这一条 */
    g_vision.retry_cnt     = 0;                /* 新一轮：重发计数从零起 */
    g_vision.track_state   = V_TRACK_RUNNING;
    g_vision.last_recv_tick = HAL_GetTick();
}

void Vision_TrackStop(vision_series_t series)
{
    switch (series)
    {
        case VISION_SERIES_A: Transmit_TrackStop(HOST_CMD_A_TRACK_STOP); break;
        case VISION_SERIES_B: Transmit_TrackStop(HOST_CMD_B_TRACK_STOP); break;
        case VISION_SERIES_C: Transmit_TrackStop(HOST_CMD_C_TRACK_STOP); break;
        default: break;
    }
    g_vision.track_state = V_TRACK_WAIT_STOP;
    g_vision.last_recv_tick = HAL_GetTick();
}

/* 排爆放置段开始追踪（0x25，LEN=2 控制周期）；固定 A 系列 */
void Vision_PlaceBegin(uint16_t rate_hz)
{
    if (rate_hz == 0)
        rate_hz = VISION_DEFAULT_RATE_HZ;

    Transmit_TrackBegin(HOST_CMD_A_PLACE_BEGIN, rate_hz);
    g_vision.series        = VISION_SERIES_A;
    g_vision.track_rate_hz = rate_hz;
    g_vision.track_cmd     = HOST_CMD_A_PLACE_BEGIN;   /* 超时重发要原样再发 0x25 */
    g_vision.retry_cnt     = 0;
    g_vision.track_state   = V_TRACK_RUNNING;
    g_vision.last_recv_tick = HAL_GetTick();
}

/* 排爆放置段停止追踪（0x27） */
void Vision_PlaceStop(void)
{
    Transmit_TrackStop(HOST_CMD_A_PLACE_STOP);
    g_vision.track_state = V_TRACK_WAIT_STOP;
    g_vision.last_recv_tick = HAL_GetTick();
}

/* ============================================================
 * 位姿上报（0x14）
 * ============================================================ */
void Vision_ReportPose(int16_t x_mm, int16_t y_mm, int16_t z_mm)
{
    Transmit_PoseReport(x_mm, y_mm, z_mm);
    g_vision.pose_tick = HAL_GetTick();
}

/* ============================================================
 * 周期驱动：视觉超时重发 + 位姿上报节拍
 * ============================================================ */

/* 重发「开始追踪」命令（协议 §2.5：追踪段超时重发的是它本身，不是「观察三物体」）。
 *
 * ★ 有意【不走】 Vision_TrackBegin/Vision_PlaceBegin：那两个是「开始新一轮」，
 *   会把 retry_cnt 清零；重发路径若清零，就成了无限重发、永不判丢失。
 *   这里只做两件事：原样再发一帧 + 重置计时（不重置计时则下一个 tick 立刻再发 → 忙循环）。 */
static void Vision_ReTrack(void)
{
    Transmit_TrackBegin(g_vision.track_cmd, g_vision.track_rate_hz);
    g_vision.last_recv_tick = HAL_GetTick();
}

/* 阶段1（观察三物体）与阶段2（追踪）共用同一套「超时 → 重发 ×3 → 判出视野」框架，
 * 区别只在重发的是哪条命令。协议 §2.5 对这两段的超时都要兜底：
 *   - 阶段1：仅「收不到就重发」，不套用抬臂回退；
 *   - 阶段2：重发耗尽后由任务层兑现「抬大臂 + 回上一帧位姿 / 回展开态」。 */
void Vision_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t age = (uint32_t)(now - g_vision.last_recv_tick);
    uint8_t  waiting = (g_vision.track_state == V_TRACK_WAIT_OBS3) ||
                       (g_vision.track_state == V_TRACK_RUNNING);

    if (waiting && age >= VISION_TIMEOUT_MS)
    {
        if (g_vision.retry_cnt < VISION_RETRY_MAX)
        {
            g_vision.retry_cnt++;
            if (g_vision.track_state == V_TRACK_WAIT_OBS3)
                Vision_ReObs3();                    /* 重发 0x20/0x30/0x40 */
            else
                Vision_ReTrack();                   /* 重发 0x22/0x25/0x32/0x42 */
        }
        else
        {
            /* 重发耗尽 → 计入一次「出视野」（永不停机：只计数，任务层兑现兜底动作） */
            g_vision.lost_cnt++;
            g_vision.track_state = V_TRACK_IDLE;
            g_vision.retry_cnt = 0;
        }
    }

    /* 位姿上报（0x14）的节拍与 FK 解算在 main.c 的 ReportPoseTick() 里 ——
     * 它要用 θ1 软累加器与舵机实测 PWM，那两个都是 main.c 的状态，不属于通信层。
     * 本层只提供 Vision_ReportPose() 这个发送入口。 */
    (void)g_vision.pose_tick;
}
