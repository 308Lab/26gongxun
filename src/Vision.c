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

    /* 观察回包 = 阶段一结束，重置重发/丢失计数 */
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

/* 识别错误（0x12）：错误码仅记录，动作停摆待任务层处理 */
void Vision_OnIdentErr(uint8_t err_code)
{
    (void)err_code;
    g_vision.track_state = V_TRACK_IDLE;
    /* TODO: 记录错误码供 OLED/上位机查询；当前占位 */
}

/* 目标丢失（0x13）：进入视野丢失兜底（不关机，计数由任务层消费） */
void Vision_OnTargetLost(void)
{
    g_vision.lost_cnt++;
    g_vision.track_state = V_TRACK_IDLE;
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

/* 反恐目标倒下（0x35）：关激光 */
void Vision_OnTargetDown(void)
{
    g_vision.track_state = V_TRACK_IDLE;
    /* TODO: 关闭激光（激光外设 GPIO/TIM 尚未接入）；当前占位 */
}

/* ============================================================
 * MCU → 视觉 主动发送封装
 * ============================================================ */

void Vision_ReqScan(void)
{
    Transmit_ReqScan();
}

void Vision_ObsThree(vision_series_t series)
{
    switch (series)
    {
        case VISION_SERIES_A: Transmit_ObsThree(HOST_CMD_A_OBS_3);   break;
        case VISION_SERIES_B: Transmit_ObsThree(HOST_CMD_B_OBS_3);   break;
        case VISION_SERIES_C: Transmit_ObsThree(HOST_CMD_C_OBS_3);   break;
        default: break;
    }
    g_vision.series = series;
    g_vision.last_recv_tick = HAL_GetTick();
}

void Vision_TrackBegin(vision_series_t series, uint16_t rate_hz)
{
    if (rate_hz == 0)
        rate_hz = VISION_DEFAULT_RATE_HZ;

    switch (series)
    {
        case VISION_SERIES_A: Transmit_TrackBegin(HOST_CMD_A_TRACK_BEGIN, rate_hz); break;
        case VISION_SERIES_B: Transmit_TrackBegin(HOST_CMD_B_TRACK_BEGIN, rate_hz); break;
        case VISION_SERIES_C: Transmit_TrackBegin(HOST_CMD_C_TRACK_BEGIN, rate_hz); break;
        default: break;
    }
    g_vision.series       = series;
    g_vision.track_rate_hz = rate_hz;
    g_vision.track_state  = V_TRACK_RUNNING;
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
void Vision_Tick(void)
{
    uint32_t now = HAL_GetTick();

    /* 追踪中：连续超时无坐标下传 → 视为目标丢失，进入兜底（§2.5） */
    if (g_vision.track_state == V_TRACK_RUNNING)
    {
        if ((int32_t)(now - g_vision.last_recv_tick) >= VISION_TIMEOUT_MS)
        {
            /* 观察单目标超时：重发观察命令 ×3 */
            if (g_vision.retry_cnt < VISION_RETRY_MAX)
            {
                g_vision.retry_cnt++;
                Vision_ObsThree(g_vision.series);
            }
            else
            {
                /* 重发耗尽 → 判定目标出视野（永不停机：只计数，任务层兑现抬大臂/回展开态） */
                g_vision.lost_cnt++;
                g_vision.track_state = V_TRACK_IDLE;
                g_vision.retry_cnt = 0;
            }
        }
    }

    /* 位姿上报节拍：任务层接入 FK 后，此处周期性解算 TCP 并上报。
     * 当前 FK 未接入，占位：不主动上报（避免发假坐标）。 */
    (void)now;
    (void)g_vision.pose_tick;
}
