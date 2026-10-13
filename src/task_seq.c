/**
 * @file    task_seq.c
 * @brief   任务序列引擎实现（表驱动）
 *
 * 一拍做三件事：
 *   ① 没在跑任务时，看 path_runner 是不是到了任务点（RUN_ST_TASK）→ 按点号启动对应表；
 *   ② 在跑时，推进当前步：首次进入先"发起动作"，之后每拍问"到位了吗/超时了吗"；
 *   ③ 表走到 ACT_END → 回喂 PathRunner_TaskDone()，让它去下一个点（或停在终点）。
 *
 * ★ 到位判据取代了旧实现的固定延时。旧代码是
 *     Servo_SetAngleTime(id, pwm, 4000);  TaskStepSet(next, 2000);
 *   —— "告诉舵机 4 秒走完，然后睡 2 秒"，两者都对不上真实到位时刻。
 *   现在改成轮询三个查询接口（Arm/Servo/Car），到位即推进。
 */

#include "task_seq.h"

#include "main.h"
#include "path_runner.h"
#include "Servo.h"
#include "Arm.h"
#include "Car_control.h"
#include "Vision.h"
#include "arm_config.h"
#include "arm_kinematics.h"   /* arm_joint_t / arm_fk / arm_ik（视觉伺服闭环用） */

/* 任务表（由 task_def.c 提供） */
extern const task_def_t g_task_A;   /* 0 = A 点：二维码识别 */
extern const task_def_t g_task_B;   /* 1 = B 点：小球排爆   */
extern const task_def_t g_task_C;   /* 2 = C 点：激光反恐   */
extern const task_def_t g_task_D;   /* 3 = D 点：抓取人质   */

/* 舵机到位的判定死区（PWM 计数）。总线舵机的回读精度约 ±2，
 * 给 10 是留了机械回差与读回抖动 —— 太严会永远判不到位、只能等超时。 */
#define TSEQ_SERVO_TOL   10U

/* timeout_ms 填这个值 = 本步不设超时（等到底）。
 * 用于扫码（协议 §3.2 明说是阻塞式握手）与追踪段（它有自己的视野丢失兜底）。 */
#define TSEQ_T_INFINITE  0xFFFFU

/* ============================================================
 * 引擎状态
 * ============================================================ */
static const task_def_t* s_task  = NULL;   /* 当前任务表 */
static uint8_t  s_step           = 0;      /* 当前步下标 */
static uint8_t  s_begun          = 0;      /* 当前步的动作是否已发起 */
static uint8_t  s_running        = 0;      /* 是否正在跑 */
static uint8_t  s_last_point     = 0xFF;   /* 已启动过任务的点号（防重复启动） */
static uint32_t s_step_tick      = 0;      /* 当前步开始时刻（超时基准） */
static uint32_t s_pose_tick      = 0;

/* 任务跑完后的去向：1 = 停住（Task_RunAt 单独启动某个任务时），
 * 0 = 回喂 PathRunner_TaskDone 继续下一个点（Task_RunAll 全程模式）。 */
static uint8_t  s_stop_after     = 0;

/* 视野丢失兜底计数（协议 §2.5：连续 5 次 → 回展开态，永不停机） */
static uint8_t  s_lost_cnt       = 0;

/* 上一次下发的三个舵机 PWM（伺服回退用） */
static uint16_t s_prev_pwm[3] = {0, 0, 0};

/* ACT_POSE_BY_TARGET 本次的目标（P1/P2/P3）。
 * 单独存一份而不是复用 s_prev_pwm：后者是"上一帧"语义，会被伺服回退覆盖，
 * 而这里需要的是"这一步的目标"——两者混用会在伺服介入后判不到位。 */
static uint16_t s_pose_tgt[3] = {0, 0, 0};

/* ============================================================
 * 小工具
 * ============================================================ */
static uint32_t now_ms(void) { return HAL_GetTick(); }

/** 记录一次舵机 PWM 下发，供"回上一帧位姿"用。id 是舵机 ID（1/2/3）。 */
static void snap_pwm(uint8_t id1, uint16_t p1, uint8_t id2, uint16_t p2)
{
    if (id1 >= 1U && id1 <= 3U) s_prev_pwm[id1 - 1] = p1;
    if (id2 >= 1U && id2 <= 3U) s_prev_pwm[id2 - 1] = p2;
}

/** 回到上一帧位姿（视野丢失兜底的第一步）。 */
static void restore_prev_pwm(void)
{
    Servo_SetAngleTime(ARM_TH2_ID, s_prev_pwm[0], 200);
    Servo_SetAngleTime(ARM_TH3_ID, s_prev_pwm[1], 200);
    Servo_SetAngleTime(ARM_TH4_ID, s_prev_pwm[2], 200);
}

/* ============================================================
 * 视觉伺服闭环（排爆抓取段 / 救援抓取段 / 排爆放置段共用）
 *   协议 §2.3：视觉每帧给的是【相对当前末端基准点的偏移】dx/dy/dz。
 *   本函数每帧：FK(当前关节) → 叠加偏移 → 解析 IK → 下发。
 *   返回：1 = 已判到位；0 = 本帧已修正或无事可做。
 * ============================================================ */
#define TSEQ_TRACK_TIMEOUT_MS  60000U   /* 与协议 §3.1c 的视觉侧上限对齐 */

extern volatile int32_t g_step_accum_pulses;      /* θ1 软累加器（main.c） */
extern volatile uint16_t g_servo_pwm1, g_servo_pwm2, g_servo_pwm3;
extern volatile uint8_t  g_servo_valid1, g_servo_valid2, g_servo_valid3;

static int8_t servo_step_once(void)
{
    if (g_vision.last_pos_valid == 0U)
    {
        return 0;                      /* 本帧没有新坐标 */
    }
    g_vision.last_pos_valid = 0;

    {
        const float dx = (float)g_vision.last_pos.x;
        const float dy = (float)g_vision.last_pos.y;
        const float dz = (float)g_vision.last_pos.z;
        arm_joint_t j_now;
        arm_pos_t   p_now, p_target;
        arm_joint_t j_tgt;

        if ((dx * dx + dy * dy + dz * dz) <= (ARM_KINE_EPS_MM * ARM_KINE_EPS_MM))
        {
            return 1;                  /* ‖偏移‖ ≤ ε：到位 */
        }

        /* 三个舵机的实测位置都读回来过，FK 才有意义（否则 pwm=0 会换算出大负角） */
        if ((g_servo_valid1 == 0U) || (g_servo_valid2 == 0U) || (g_servo_valid3 == 0U))
        {
            return 0;
        }

        j_now.th1 = ARM_STEP_PULSE2DEG((float)g_step_accum_pulses);
        j_now.th2 = arm_pwm_to_angle(ARM_TH2_ID, g_servo_pwm1);
        j_now.th3 = arm_pwm_to_angle(ARM_TH3_ID, g_servo_pwm2);
        j_now.th4 = arm_pwm_to_angle(ARM_TH4_ID, g_servo_pwm3);

        arm_fk(&j_now, &p_now);
        p_target.x = p_now.x + dx;
        p_target.y = p_now.y + dy;
        p_target.z = p_now.z + dz;

        if (arm_ik(&p_target, 0, &j_tgt) != 0)
        {
            return 0;                  /* 超出工作空间：跳过本帧 */
        }

        {
            const uint16_t p2 = (uint16_t)arm_angle_to_pwm(ARM_TH2_ID, j_tgt.th2);
            const uint16_t p3 = (uint16_t)arm_angle_to_pwm(ARM_TH3_ID, j_tgt.th3);
            const uint16_t p4 = (uint16_t)arm_angle_to_pwm(ARM_TH4_ID, j_tgt.th4);

            snap_pwm(ARM_TH2_ID, p2, ARM_TH3_ID, p3);
            s_prev_pwm[2] = p4;
            Servo_SetAngleTime(ARM_TH2_ID, p2, 50);
            Servo_SetAngleTime(ARM_TH3_ID, p3, 50);
            Servo_SetAngleTime(ARM_TH4_ID, p4, 50);
        }
    }
    return 0;
}

/**
 * 视野丢失兜底（协议 §2.5，2026-10-13 现场定案：永不停机）
 *   收到 0x13 或超时重发耗尽 → 抬大臂一小步 + 回上一帧位姿；
 *   连续 5 次 → 回展开态并计数清零（由调用方重新发起观察）。
 * 返回：1 = 该回展开态了；0 = 已抬臂回退，可继续。
 */
static uint8_t lost_fallback(uint16_t deploy_p2, uint16_t deploy_p3, uint16_t deploy_p4)
{
    s_lost_cnt++;
    if (s_lost_cnt >= 5U)
    {
        s_lost_cnt = 0;
        snap_pwm(ARM_TH2_ID, deploy_p2, ARM_TH3_ID, deploy_p3);
        s_prev_pwm[2] = deploy_p4;
        Servo_SetAngleTime(ARM_TH2_ID, deploy_p2, 1000);
        Servo_SetAngleTime(ARM_TH3_ID, deploy_p3, 1000);
        Servo_SetAngleTime(ARM_TH4_ID, deploy_p4, 1000);
        return 1;
    }

    restore_prev_pwm();
    /* 抬大臂一小步：让目标重新进视野（最常见是夹爪挡视线或下探太深）。
     * ARM_TH2_DIR = DEC 表示"PWM 减小 = 抬升"。 */
    {
        uint16_t cur = s_prev_pwm[0];
        uint16_t tgt;
        if (ARM_TH2_DIR == ARM_JOINT_DIR_DEC)
        {
            tgt = (cur >= 30U) ? (uint16_t)(cur - 30U) : ARM_TH2_PWM_MIN;
        }
        else
        {
            tgt = (uint16_t)((cur + 30U <= ARM_TH2_PWM_MAX) ? (cur + 30U) : ARM_TH2_PWM_MAX);
        }
        Servo_SetAngleTime(ARM_TH2_ID, tgt, 300);
        s_prev_pwm[0] = tgt;
    }
    return 0;
}

/* 前向声明：这两个复合动作定义在下面，但 step_begin/step_done 要先用到 */
static void   pose_by_target(uint8_t which);
static int8_t car_aim_step(void);

/* ============================================================
 * 单步：发起动作
 * ============================================================ */
static void step_begin(const tstep_t* st)
{
    switch (st->act)
    {
        case ACT_SERVO1:
            Servo_SetAngleTime(st->sid1, st->spwm1, st->stime_ms);
            snap_pwm(st->sid1, st->spwm1, 0, 0);
            break;

        case ACT_SERVO2:
            Servo_SetAngleTime(st->sid1, st->spwm1, st->stime_ms);
            Servo_SetAngleTime(st->sid2, st->spwm2, st->stime_ms);
            snap_pwm(st->sid1, st->spwm1, st->sid2, st->spwm2);
            break;

        case ACT_SERVO3:
            Servo_SetAngleTime(st->sid1, st->spwm1, st->stime_ms);
            Servo_SetAngleTime(st->sid2, st->spwm2, st->stime_ms);
            Servo_SetAngleTime(st->sid3, st->spwm3, st->stime_ms);
            snap_pwm(st->sid1, st->spwm1, st->sid2, st->spwm2);
            if (st->sid3 >= 1U && st->sid3 <= 3U) s_prev_pwm[st->sid3 - 1] = st->spwm3;
            break;

        case ACT_GRIP:
            PWM_Servo_SetAngle((float)st->arg);
            break;

        case ACT_POSE_BY_TARGET:
            pose_by_target((uint8_t)st->arg);
            break;

        case ACT_STEP1:
        case ACT_STEP1_SLOW:
            /* 软累加绝对定位：算 delta，正走 CW、负走 CCW */
            {
                const int32_t delta = st->pulses - g_step_accum_pulses;
                const uint8_t slow = (uint8_t)(st->act == ACT_STEP1_SLOW);
                if (delta > 0)
                {
                    if (slow != 0U) Arm_MoveRelativeSlow((uint32_t)delta, ARM_DIR_CW);
                    else            Arm_MoveRelative((uint32_t)delta, ARM_DIR_CW);
                    g_step_accum_pulses = st->pulses;
                }
                else if (delta < 0)
                {
                    if (slow != 0U) Arm_MoveRelativeSlow((uint32_t)(-delta), ARM_DIR_CCW);
                    else            Arm_MoveRelative((uint32_t)(-delta), ARM_DIR_CCW);
                    g_step_accum_pulses = st->pulses;
                }
            }
            break;

        case ACT_VIS_SCAN:          Vision_ReqScan();                     break;
        case ACT_VIS_OBS3_A:
        case ACT_VIS_OBS3_C:        Vision_ObsThree((st->act == ACT_VIS_OBS3_A)
                                                        ? VISION_SERIES_A : VISION_SERIES_C);
                                    s_lost_cnt = 0;
                                    break;
        case ACT_VIS_TRACK_A:       Vision_TrackBegin(VISION_SERIES_A, 50); s_lost_cnt = 0; break;
        case ACT_VIS_TRACK_B:       Vision_TrackBegin(VISION_SERIES_B, 50); s_lost_cnt = 0; break;
        case ACT_VIS_TRACK_C:       Vision_TrackBegin(VISION_SERIES_C, 50); s_lost_cnt = 0; break;
        case ACT_VIS_TRACK_BUCKET:  Vision_PlaceBegin(50);                  s_lost_cnt = 0; break;
        case ACT_VIS_STOP_A:        Vision_TrackStop(VISION_SERIES_A);      break;
        case ACT_VIS_STOP_B:        Vision_TrackStop(VISION_SERIES_B);      break;
        case ACT_VIS_STOP_C:        Vision_TrackStop(VISION_SERIES_C);      break;
        case ACT_VIS_STOP_BUCKET:   Vision_PlaceStop();                     break;

        case ACT_CAR_AIM:           break;   /* 每拍在 step_done 里推进，见 car_aim_step */

        case ACT_LASER_ON:
            HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
            /* 清掉可能残留的"靶倒下"标志：只认开激光【之后】收到的 0x35，
             * 否则上一轮的靶倒下会让 ACT_WAIT_TARGET_DOWN 立刻通过、激光一闪就关。 */
            g_vision.target_down = 0;
            break;
        case ACT_LASER_OFF:         HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET); break;

        default: break;
    }
}

/* ============================================================
 * 单步：到位了吗（每拍调用；视觉伺服就在这里面推进）
 * ============================================================ */
static uint8_t step_done(const tstep_t* st)
{
    switch (st->act)
    {
        case ACT_SERVO1:
            return (uint8_t)(Servo_IsReached(st->sid1, st->spwm1, TSEQ_SERVO_TOL) == 1);

        case ACT_SERVO2:
            return (uint8_t)((Servo_IsReached(st->sid1, st->spwm1, TSEQ_SERVO_TOL) == 1) &&
                             (Servo_IsReached(st->sid2, st->spwm2, TSEQ_SERVO_TOL) == 1));

        case ACT_SERVO3:
            return (uint8_t)((Servo_IsReached(st->sid1, st->spwm1, TSEQ_SERVO_TOL) == 1) &&
                             (Servo_IsReached(st->sid2, st->spwm2, TSEQ_SERVO_TOL) == 1) &&
                             (Servo_IsReached(st->sid3, st->spwm3, TSEQ_SERVO_TOL) == 1));

        case ACT_GRIP:
            return 1;   /* 夹爪是普通 PWM 舵机，没有位置回读 —— 只能靠 timeout 走完 */

        case ACT_POSE_BY_TARGET:
            return (uint8_t)((Servo_IsReached(ARM_TH2_ID, s_pose_tgt[0], TSEQ_SERVO_TOL) == 1) &&
                             (Servo_IsReached(ARM_TH3_ID, s_pose_tgt[1], TSEQ_SERVO_TOL) == 1) &&
                             (Servo_IsReached(ARM_TH4_ID, s_pose_tgt[2], TSEQ_SERVO_TOL) == 1));

        case ACT_STEP1:
        case ACT_STEP1_SLOW:
            return (uint8_t)(Arm_IsReached() == 1);

        case ACT_WAIT_TARGET_DOWN:
            return g_vision.target_down;   /* 视觉报 0x35；没报就等 timeout 放行 */

        case ACT_VIS_SCAN:
            return Vision_ConsumeScanDone();

        case ACT_VIS_OBS3_A:
        case ACT_VIS_OBS3_C:
            return g_vision.obs3.valid;

        /* 追踪段：每拍推一次伺服；收敛即完成。
         * 视野丢失（0x13 或超时重发耗尽）由 Vision 层累计到 lost_cnt，这里消费并兜底。 */
        case ACT_VIS_TRACK_A:
        case ACT_VIS_TRACK_B:
        case ACT_VIS_TRACK_C:
        case ACT_VIS_TRACK_BUCKET:
            if (g_vision.lost_cnt > s_lost_cnt)
            {
                s_lost_cnt = g_vision.lost_cnt;
                (void)lost_fallback(ARM_POSE_B_MID_P1, ARM_POSE_B_MID_P2, ARM_POSE_B_MID_P3);
                /* 兜底后是否重发追踪，交给下一步/外部；这里只推进，避免卡死 */
                return 1;
            }
            return (uint8_t)(servo_step_once() == 1);

        case ACT_CAR_AIM:
            /* 对准期间也消费视野丢失（对不准多半是目标跑了） */
            if (g_vision.lost_cnt > s_lost_cnt)
            {
                s_lost_cnt = g_vision.lost_cnt;
                Car_SetTarget(0.0f, 0.0f);
                return 1;
            }
            return (uint8_t)(car_aim_step() == 1);

        case ACT_DELAY:
        default:
            return 0;   /* 由超时判据放行 */
    }
}

/* 按 target_id 选预存姿态并下发（排爆 B 或救援 D）。
 * arg: 0 = 用 B 的预存位表，1 = 用 D 的。见 task_seq.h 的 ACT_POSE_BY_TARGET。 */
static void pose_by_target(uint8_t which)
{
    const uint8_t id = g_vision.obs3.target_id;   /* 1左 / 2中 / 3右 */
    uint16_t p1, p2, p3;
    int32_t  tgt_pulses;

    if (which == 0U)   /* 排爆 B */
    {
        if (id == 1U)      { p1 = ARM_POSE_B_LEFT_P1;  p2 = ARM_POSE_B_LEFT_P2;  p3 = ARM_POSE_B_LEFT_P3;  tgt_pulses = ARM_TH1_B_LEFT_PULSE; }
        else if (id == 3U) { p1 = ARM_POSE_B_RIGHT_P1; p2 = ARM_POSE_B_RIGHT_P2; p3 = ARM_POSE_B_RIGHT_P3; tgt_pulses = ARM_TH1_B_RIGHT_PULSE; }
        else               { p1 = ARM_POSE_B_MID_P1;   p2 = ARM_POSE_B_MID_P2;   p3 = ARM_POSE_B_MID_P3;   tgt_pulses = ARM_TH1_B_MID_PULSE; }
    }
    else               /* 救援 D */
    {
        if (id == 1U)      { p1 = ARM_POSE_D_LEFT_P1;  p2 = ARM_POSE_D_LEFT_P2;  p3 = ARM_POSE_D_LEFT_P3;  tgt_pulses = ARM_TH1_D_LEFT_PULSE; }
        else if (id == 3U) { p1 = ARM_POSE_D_RIGHT_P1; p2 = ARM_POSE_D_RIGHT_P2; p3 = ARM_POSE_D_RIGHT_P3; tgt_pulses = ARM_TH1_D_RIGHT_PULSE; }
        else               { p1 = ARM_POSE_D_MID_P1;   p2 = ARM_POSE_D_MID_P2;   p3 = ARM_POSE_D_MID_P3;   tgt_pulses = ARM_TH1_D_MID_PULSE; }
    }

    Servo_SetAngleTime(ARM_TH2_ID, p1, 2000);
    Servo_SetAngleTime(ARM_TH3_ID, p2, 2000);
    Servo_SetAngleTime(ARM_TH4_ID, p3, 2000);
    snap_pwm(ARM_TH2_ID, p1, ARM_TH3_ID, p2);
    s_prev_pwm[2] = p3;
    s_pose_tgt[0] = p1;
    s_pose_tgt[1] = p2;
    s_pose_tgt[2] = p3;

    /* θ1 同步转到位 */
    {
        const int32_t delta = tgt_pulses - g_step_accum_pulses;
        if (delta > 0)      { Arm_MoveRelative((uint32_t)delta, ARM_DIR_CW);   g_step_accum_pulses = tgt_pulses; }
        else if (delta < 0) { Arm_MoveRelative((uint32_t)(-delta), ARM_DIR_CCW); g_step_accum_pulses = tgt_pulses; }
    }

    g_vision.obs3.valid = 0;   /* 消费掉这次回包 */
}

/**
 * 反恐对准：靠车前后移动把激光打到靶心上（见 task_seq.h 的 ACT_CAR_AIM）。
 * 每帧消费一次视觉偏移；‖偏移‖ ≤ ε 即对准。
 * 返回：1 = 已对准；0 = 本帧已调整或无新数据。
 */
static int8_t car_aim_step(void)
{
    if (g_vision.last_pos_valid == 0U) return 0;
    g_vision.last_pos_valid = 0;

    {
        const float dx = (float)g_vision.last_pos.x;   /* 视觉 x ↔ 车前后方向 */
        const float dy = (float)g_vision.last_pos.y;   /* 视觉 y ↔ 高度（姿态锁死，不参与） */

        if ((dx * dx + dy * dy) <= (ARM_KINE_EPS_MM * ARM_KINE_EPS_MM))
        {
            Car_SetTarget(0.0f, 0.0f);
            return 1;
        }

        /* w 恒为 0：只前后挪。增益是现场标定的旋钮（mm 偏移 → mm/周期 速度），
         * 取 0.001 是保守起点 —— 车几乎不动，联调时往上加。 */
        Car_SetTarget(dx * 0.001f, 0.0f);
    }
    return 0;
}

static uint16_t step_timeout(const tstep_t* st)
{
    if (st->timeout_ms != 0U) return st->timeout_ms;
    /* 视觉相关的默认值单独给：它们比舵机动作慢 */
    switch (st->act)
    {
        case ACT_VIS_SCAN:
        case ACT_VIS_TRACK_A: case ACT_VIS_TRACK_B:
        case ACT_VIS_TRACK_C: case ACT_VIS_TRACK_BUCKET:
            return TSEQ_T_INFINITE;
        default:
            return TSEQ_T_DEFAULT;
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
static const task_def_t* task_of_point(uint8_t idx)
{
    switch (idx)
    {
        case 0: return &g_task_A;
        case 1: return &g_task_B;
        case 2: return &g_task_C;
        case 3: return &g_task_D;
        default: return NULL;
    }
}

void Task_RunAt(uint8_t point_idx)
{
    s_last_point = 0xFF;          /* 允许重新启动同一点 */
    s_running = 0;
    s_stop_after = 1;             /* 单独启动：跑完这个任务就停，不继续往下走 */
    PathRunner_Goto(point_idx);   /* 从当前位置直接去那个点 */
}

void Task_RunAll(void)
{
    s_last_point = 0xFF;
    s_running = 0;
    s_stop_after = 0;             /* 全程模式：每个点做完继续下一个 */
    /* 起步前把陀螺仪归零（原地不动，安全）。旧实现这里还有"等 2s 让陀螺仪稳定"
     * 的前摇，随键位桩一起删掉了 —— 若上机发现起步航向不稳，在 PathRunner_Goto
     * 之前补一段延时即可。 */
    Car_YawCalibrate();
    PathRunner_Goto(0);           /* 去 A，之后由 task_done 逐点推进 */
}

uint8_t Task_IsBusy(void)
{
    return s_running;
}

int Task_CurrentStep(void)
{
    return s_running ? (int)s_step : -1;
}

void Task_Abort(void)
{
    s_running = 0;
    s_task = NULL;
    s_step = 0;
    s_begun = 0;
    s_lost_cnt = 0;
    Car_SetTarget(0.0f, 0.0f);
}

void Task_Tick(void)
{
    /* ── ① 未在跑：等 path_runner 到达任务点 ───────────────────────────── */
    if (s_running == 0U)
    {
        if (g_run_state == RUN_ST_TASK)
        {
            if (s_last_point != g_target_idx)     /* 同一个点只启动一次 */
            {
                const task_def_t* t = task_of_point(g_target_idx);
                s_last_point = g_target_idx;
                if (t != NULL)
                {
                    s_task  = t;
                    s_step  = 0;
                    s_begun = 0;
                    s_running = 1;
                    s_lost_cnt = 0;
                    s_step_tick = now_ms();
                }
                else
                {
                    PathRunner_TaskDone();        /* 该点没定义任务：直接放行 */
                }
            }
        }
        return;
    }

    /* ── ② 在跑：推进当前步 ──────────────────────────────────────────── */
    {
        const tstep_t* st = &s_task->steps[s_step];

        if (st->act == ACT_END)
        {
            s_running = 0;
            s_task = NULL;
            if (s_stop_after != 0U)
            {
                /* 单独启动某个任务（Task_RunAt）：跑完就停在这儿，别继续往下走。
                 * 停 path_runner 用 RUN_ST_IDLE，不用 RUN_ST_DONE
                 *（DONE 会触发终点动作）。 */
                g_run_state = RUN_ST_IDLE;
                Car_SetTarget(0.0f, 0.0f);
            }
            else
            {
                PathRunner_TaskDone();   /* 全程模式：交给 path_runner 去下一个点 */
            }
            return;
        }

        if (s_begun == 0U)
        {
            step_begin(st);
            s_begun = 1;
            s_step_tick = now_ms();
        }

        {
            const uint8_t done = step_done(st);
            uint32_t elapsed = now_ms() - s_step_tick;
            const uint16_t tmo = step_timeout(st);

            if (done != 0U || ((tmo != TSEQ_T_INFINITE) && (elapsed >= (uint32_t)tmo)))
            {
                s_step++;
                s_begun = 0;
            }
        }
    }

    /* 位姿上报（0x14）—— 暂不启用，见 main.c 的同名说明 */
    (void)s_pose_tick;
}
