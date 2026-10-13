/**
 * @file    formal_mode.c
 * @brief   正式模式 Supervisor（全自动 A→B→C→D 任务状态机）实现
 *
 * 架构：
 *   path_runner 负责移动层，把车从起点开到 A/B/C/D/终点（到点进 RUN_ST_TASK 等 task_done）。
 *   本模块接管「任务点」的动作：每个点一个独立的任务动作序列状态机，序列末
 *   调 PathRunner_TaskDone()，path_runner 继续去下一个点。终点到达后置 DONE。
 *
 * 键位 → 终点边界（formal_end_t）：
 *   键1 FORMAL_END_AFTER_D  键2 FORMAL_END_AFTER_A
 *   键3 FORMAL_END_AFTER_B  键4 FORMAL_END_AFTER_C
 *   —— 234 是「完整执行到指定任务收尾停下」，机械臂 + 视觉全程都做，不是只跑车。
 *
 * 视觉伺服（2016-10-13 执线人定案，对齐协议 v1.4 §2.3/§2.5/§3）：
 *   - 坐标语义：视觉流式回包 xyz = 相对当前末端基准点（夹爪 TCP/激光）的偏移 dx/dy/dz。
 *   - 闭环（B/D 抓取段）：每帧收偏移 → ‖偏移‖≤ε 判到位；否则 FK→叠加偏移得绝对点 →解析 IK→下发。
 *     ε = 5mm（ARM_KINE_EPS_MM）。
 *   - 视野丢失兜底（永不停机）：收 0x13 或 500ms 超时 → 抬大臂 θ2 小步 + 回上一帧 PWM 快照 →
 *     重新观察；连续 5 次丢失 → 回展开态重新观察。循环重试、无上限、永不停机。
 *   - 任务 C（反恐）：走车对准、不动大小臂。P1/P2 不动、P3 平伸、θ1 左转 90°，
 *     对准靠移车 + 视觉偏移反馈（Car_SetTarget 微调车位），不套 FK+IK 调臂。
 */

#include "formal_mode.h"
#include "path_runner.h"
#include "Vision.h"
#include "arm_kinematics.h"
#include "arm_config.h"
#include "Arm.h"
#include "Servo.h"
#include "Car_control.h"

/* ============================================================
 * 全局状态
 * ============================================================ */
volatile formal_state_t g_formal_state = FORMAL_IDLE;

static formal_end_t s_end = FORMAL_END_AFTER_D;   /* 终点边界 */

/* 任务动作序列步进 */
static uint8_t  s_task_step = 0;      /* 当前任务内步骤号（0=未启动） */
static uint32_t s_step_due = 0;       /* 当前步骤延时截止 tick */

/* 视野丢失兜底状态 */
static uint8_t  s_lost_cnt = 0;       /* 当前任务连续出视野计数（>=5 回展开态） */

/* 手动收起兜底：置位后 A 任务跳过等 0x11，直接走收起步 */
static volatile uint8_t s_force_retract = 0;

/* ============================================================
 * 小工具
 * ============================================================ */

extern volatile int32_t g_step_accum_pulses;   /* θ1 软累加器（定义于 main.c） */

static void TaskStepSet(uint8_t next_step, uint32_t delay_ms)
{
    s_task_step = next_step;
    s_step_due  = HAL_GetTick() + delay_ms;
}

/* 步进 θ1 绝对定位（软累加器），target_pulses 带符号（正=左转 CW，负=右转 CCW） */
static void FormalStepGoTo(int32_t target_pulses)
{
    int32_t delta = target_pulses - g_step_accum_pulses;
    if (delta > 0)
        Arm_MoveRelative((uint32_t)delta, ARM_DIR_CW);
    else if (delta < 0)
        Arm_MoveRelative((uint32_t)(-delta), ARM_DIR_CCW);
    g_step_accum_pulses = target_pulses;
}

/* 慢速步进（D 救援段 30 RPM） */
static void FormalStepGoToSlow(int32_t target_pulses)
{
    int32_t delta = target_pulses - g_step_accum_pulses;
    if (delta > 0)
        Arm_MoveRelativeSlow((uint32_t)delta, ARM_DIR_CW);
    else if (delta < 0)
        Arm_MoveRelativeSlow((uint32_t)(-delta), ARM_DIR_CCW);
    g_step_accum_pulses = target_pulses;
}

/* 激光开关（GPIOC.1 = Laser_Pin） */
#define LASER_ON_LEVEL   GPIO_PIN_SET
static void Formal_LaserSet(uint8_t on)
{
    GPIO_PinState s = on ? LASER_ON_LEVEL
                         : ((LASER_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, s);
}

/* ============================================================
 * 上一帧 PWM 快照（视野丢失回退用）
 * ============================================================ */
static uint16_t s_prev_pwm2 = 0;   /* θ2 大臂上一帧下发 PWM */
static uint16_t s_prev_pwm3 = 0;   /* θ3 小臂上一帧下发 PWM */
static uint16_t s_prev_pwm4 = 0;   /* θ4 腕上一帧下发 PWM */

/* 记录一次下发的 PWM 快照 */
static void SnapPwm(uint16_t p2, uint16_t p3, uint16_t p4)
{
    s_prev_pwm2 = p2;
    s_prev_pwm3 = p3;
    s_prev_pwm4 = p4;
}

/* 回上一帧 PWM 快照 */
static void RestorePrevPwm(void)
{
    Servo_SetAngleTime(ARM_TH2_ID, s_prev_pwm2, 200);
    Servo_SetAngleTime(ARM_TH3_ID, s_prev_pwm3, 200);
    Servo_SetAngleTime(ARM_TH4_ID, s_prev_pwm4, 200);
}

/* 抬大臂一小步（θ2 大臂 PWM 按 ARM_DIR_DEC=升高减，即 PWM 减小 → 抬升）。
 * 视野丢失最常见是夹爪挡视线/下探太深，抬臂让目标重新入视野。 */
#define LIFT_PWM_STEP   30   /* 抬臂步长 PWM（待联调，先 30） */
static void LiftArmLift(void)
{
    uint16_t cur = s_prev_pwm2;
    uint16_t target;
    if (ARM_TH2_DIR == ARM_JOINT_DIR_DEC)   /* 升高减 */
    {
        target = (cur >= LIFT_PWM_STEP) ? (cur - LIFT_PWM_STEP) : ARM_TH2_PWM_MIN;
    }
    else
    {
        target = (cur + LIFT_PWM_STEP <= ARM_TH2_PWM_MAX) ? (cur + LIFT_PWM_STEP) : ARM_TH2_PWM_MAX;
    }
    Servo_SetAngleTime(ARM_TH2_ID, target, 300);
    s_prev_pwm2 = target;   /* 更新快照，下一帧回退回到抬后位 */
}

/* ============================================================
 * 视觉伺服闭环（B/D 抓取段；§6.6，解析 IK 非雅可比）
 * ============================================================
 * 输入：g_vision.last_pos（流式偏移 dx/dy/dz）。返回运算结果码：
 *   1 = 已判到位（‖偏移‖≤ε）
 *   0 = 本帧刚下发一次 FK+IK 修正，继续等下一帧
 *  -1 = 本帧无新坐标（last_pos 未被视觉更新）
 */
static int8_t Servo_StepOnce_Pick(void)
{
    if (!g_vision.last_pos_valid)
    {
        return -1;
    }
    g_vision.last_pos_valid = 0;

    float dx = (float)g_vision.last_pos.x;
    float dy = (float)g_vision.last_pos.y;
    float dz = (float)g_vision.last_pos.z;

    /* 到位判据：‖偏移‖ ≤ ε（用 L1 或 L2；此处用欧氏距离） */
    float norm = dx * dx + dy * dy + dz * dz;
    if (norm <= ARM_KINE_EPS_MM * ARM_KINE_EPS_MM)
    {
        return 1;   /* 对准了 */
    }

    /* 未到位：FK 当前末端 → 叠加偏移 → 解析 IK → 下发 */
    arm_joint_t j_now;
    extern volatile uint16_t g_servo_pwm1;
    extern volatile uint16_t g_servo_pwm2;
    extern volatile uint16_t g_servo_pwm3;

    j_now.th2 = arm_pwm_to_angle(ARM_TH2_ID, g_servo_pwm1);
    j_now.th3 = arm_pwm_to_angle(ARM_TH3_ID, g_servo_pwm2);
    j_now.th4 = arm_pwm_to_angle(ARM_TH4_ID, g_servo_pwm3);
    j_now.th1 = ARM_STEP_PULSE2DEG((float)g_step_accum_pulses);

    arm_pos_t p_now;
    arm_fk(&j_now, &p_now);

    arm_pos_t p_target;
    p_target.x = p_now.x + dx;
    p_target.y = p_now.y + dy;
    p_target.z = p_now.z + dz;

    arm_joint_t j_tgt;
    if (arm_ik(&p_target, 0, &j_tgt) != 0)
    {
        return 0;   /* 目标超出工作空间：跳过本帧 */
    }

    uint16_t p2 = (uint16_t)arm_angle_to_pwm(ARM_TH2_ID, j_tgt.th2);
    uint16_t p3 = (uint16_t)arm_angle_to_pwm(ARM_TH3_ID, j_tgt.th3);
    uint16_t p4 = (uint16_t)arm_angle_to_pwm(ARM_TH4_ID, j_tgt.th4);

    SnapPwm(p2, p3, p4);
    FormalStepGoTo(arm_theta1_to_pulse(j_tgt.th1));
    Servo_SetAngleTime(ARM_TH2_ID, p2, 50);
    Servo_SetAngleTime(ARM_TH3_ID, p3, 50);
    Servo_SetAngleTime(ARM_TH4_ID, p4, 50);

    return 0;
}

/* 视野丢失兜底触发：抬大臂 + 回上一帧 PWM；连续 5 次 → 回展开态。
 * 返回：1 = 该回展开态了；0 = 已抬臂+回退，可重新观察。 */
static uint8_t Servo_LostFallback(uint8_t deploy_p2, uint8_t deploy_p3, uint8_t deploy_p4)
{
    s_lost_cnt++;
    if (s_lost_cnt >= 5)
    {
        /* 连续 5 次丢失 → 回展开态重新观察 */
        s_lost_cnt = 0;
        SnapPwm(deploy_p2, deploy_p3, deploy_p4);
        Servo_SetAngleTime(ARM_TH2_ID, deploy_p2, 1000);
        Servo_SetAngleTime(ARM_TH3_ID, deploy_p3, 1000);
        Servo_SetAngleTime(ARM_TH4_ID, deploy_p4, 1000);
        return 1;
    }
    /* 未到 5 次：回上一帧 PWM + 抬大臂小步 */
    RestorePrevPwm();
    LiftArmLift();
    return 0;
}

/* ============================================================
 * 各任务点动作序列
 * ============================================================ */

static void TaskComplete(void);   /* 前向声明：任务完成 + 终点边界拦截 */

/* A 任务（扫码）：只动腕 P3 平伸，发 0x10，等 0x11，回位。无停机。 */
static void TaskA_Step(void)
{
    switch (s_task_step)
    {
        case 1:   /* 展开：大臂 P1 + 小臂 P2 同时（4s） */
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_A_SCAN_P1, 4000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_A_SCAN_P2, 4000);
            TaskStepSet(2, 2000);
            break;
        case 2:   /* 等 2s 后腕部再动 */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_A_SCAN_P3, 2000);
            TaskStepSet(3, 2500);
            break;
        case 3:   /* 扫码 */
            Vision_ReqScan();
            TaskStepSet(4, 0);
            break;
        case 4:   /* 等 0x11；可被再按「手动收起」跳过 */
            if (Vision_ConsumeScanDone() || s_force_retract)
            {
                s_force_retract = 0;
                TaskStepSet(5, 0);
            }
            break;
        case 5:   /* 收起：腕 P3 + 大臂 P1 同时（大臂 3.5s，腕 2s） */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_HOME_P1, 3500);
            TaskStepSet(6, 1000);
            break;
        case 6:   /* 过 1s 后收小臂 P2（2s） */
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_HOME_P2, 2000);
            TaskStepSet(7, 2500);
            break;
        default:
            TaskComplete();
            break;
    }
}

/* B 任务（排爆）：展开→观察→选目标→粗调→抓取伺服→夹取→升起→转桶→放置→回收 */
static void TaskB_Step(void)
{
    switch (s_task_step)
    {
        case 1:   /* 展开：夹爪开 + 舵机先动 + 步进后动 */
            PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_DEPLOY_P1, 3000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_DEPLOY_P2, 3000);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_DEPLOY_P3, 3000);
            SnapPwm(ARM_POSE_B_DEPLOY_P1, ARM_POSE_B_DEPLOY_P2, ARM_POSE_B_DEPLOY_P3);
            TaskStepSet(2, 2000);
            break;
        case 2:   /* 步进 θ1 左转 90°（后动） */
            FormalStepGoTo(ARM_TH1_B_MID_PULSE);
            TaskStepSet(3, 4500);
            break;
        case 3:   /* 观察三物体 */
            Vision_ObsThree(VISION_SERIES_A);
            TaskStepSet(4, 0);
            break;
        case 4:   /* 等三目标回包 → 选预存粗调位 */
            if (g_vision.obs3.valid)
            {
                uint8_t id = g_vision.obs3.target_id;   /* 1左/2中/3右 */
                if (id == 1)
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_LEFT_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_LEFT_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_LEFT_P3, 2000);
                    FormalStepGoTo(ARM_TH1_B_LEFT_PULSE);
                    SnapPwm(ARM_POSE_B_LEFT_P1, ARM_POSE_B_LEFT_P2, ARM_POSE_B_LEFT_P3);
                }
                else if (id == 3)
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_RIGHT_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_RIGHT_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_RIGHT_P3, 2000);
                    FormalStepGoTo(ARM_TH1_B_RIGHT_PULSE);
                    SnapPwm(ARM_POSE_B_RIGHT_P1, ARM_POSE_B_RIGHT_P2, ARM_POSE_B_RIGHT_P3);
                }
                else
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_MID_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_MID_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_MID_P3, 2000);
                    FormalStepGoTo(ARM_TH1_B_MID_PULSE);
                    SnapPwm(ARM_POSE_B_MID_P1, ARM_POSE_B_MID_P2, ARM_POSE_B_MID_P3);
                }
                g_vision.obs3.valid = 0;
                s_lost_cnt = 0;
                TaskStepSet(5, 2500);
            }
            break;
        case 5:   /* 开始追踪（抓取段） */
            Vision_TrackBegin(VISION_SERIES_A, 50);
            TaskStepSet(6, 0);
            break;
        case 6:   /* 抓取段伺服闭环：收敛判据 + 视野丢失兜底 */
        {
            int8_t r = Servo_StepOnce_Pick();
            if (r == 1)
            {
                /* 到位 → 停追踪 → 夹取 */
                Vision_TrackStop(VISION_SERIES_A);
                s_lost_cnt = 0;
                TaskStepSet(7, 0);
            }
            else if (g_vision.lost_cnt > s_lost_cnt)
            {
                /* 视野丢失事件（0x13 或超时触发）→ 兜底 */
                if (Servo_LostFallback(ARM_POSE_B_MID_P1, ARM_POSE_B_MID_P2, ARM_POSE_B_MID_P3))
                {
                    /* 连丢 5 次 → 回展开态后重新观察三物体 */
                    Vision_ObsThree(VISION_SERIES_A);
                    TaskStepSet(4, 0);
                }
                else
                {
                    /* 抬臂+回退后重新开始追踪 */
                    Vision_TrackBegin(VISION_SERIES_A, 50);
                    /* 停留本步，继续伺服 */
                }
            }
            /* 无新坐标或无事件：停留本步继续 */
            break;
        }
        case 7:   /* 下翻 1210 + 夹取 95° */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_TH4_PWM_DOWN, 1000);
            PWM_Servo_SetAngle(ARM_GRIP_CLOSE_DEG);
            TaskStepSet(8, 1500);
            break;
        case 8:   /* 步进 θ1 转桶（右侧 -5600，先步进） */
            FormalStepGoTo(-ARM_STEP_PULSE_TURNTABLE / 4);
            TaskStepSet(9, 5000);
            break;
        case 9:   /* 舵机转桶预存位（1605/1980/1210，后舵机） */
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_BUCKET_P1, 1500);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_BUCKET_P2, 1500);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_BUCKET_P3, 1500);
            SnapPwm(ARM_POSE_BUCKET_P1, ARM_POSE_BUCKET_P2, ARM_POSE_BUCKET_P3);
            TaskStepSet(10, 2000);
            break;
        case 10:  /* 放置段开始追踪（0x25） */
            Vision_PlaceBegin(50);
            s_lost_cnt = 0;
            TaskStepSet(11, 0);
            break;
        case 11:  /* 放置段伺服闭环（逼近桶，复用夹爪 TCP 偏移） */
        {
            int8_t r = Servo_StepOnce_Pick();
            if (r == 1)
            {
                Vision_PlaceStop();
                s_lost_cnt = 0;
                TaskStepSet(12, 0);
            }
            else if (g_vision.lost_cnt > s_lost_cnt)
            {
                if (Servo_LostFallback(ARM_POSE_BUCKET_P1, ARM_POSE_BUCKET_P2, ARM_POSE_BUCKET_P3))
                {
                    Vision_PlaceStop();
                    TaskStepSet(9, 0);   /* 连丢 5 次 → 回桶预存位重新观察 */
                }
                else
                {
                    Vision_PlaceBegin(50);
                }
            }
            break;
        }
        case 12:  /* 放置：夹爪张开 45° */
            PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
            TaskStepSet(13, 1200);
            break;
        case 13:  /* 收夹爪中性 135° */
            PWM_Servo_SetAngle(ARM_GRIP_PARK_DEG);
            TaskStepSet(14, 1200);
            break;
        case 14:  /* 步进 θ1 回 0（先步进） */
            FormalStepGoTo(0);
            TaskStepSet(15, 5000);
            break;
        case 15:  /* 收起（舵机 home，后动） */
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_HOME_P1, 3000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_HOME_P2, 3000);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 3000);
            TaskStepSet(16, 3500);
            break;
        default:
            TaskComplete();
            break;
    }
}

/* C 任务（反恐）：走车对准、不动大小臂。θ1 左转90° + P3 平伸 + 激光2s关。 */
static void TaskC_Step(void)
{
    switch (s_task_step)
    {
        case 1:   /* 步进 θ1 左转 90°（先动）；P1/P2 不动 */
            FormalStepGoTo(ARM_TH1_C_MID_PULSE);
            TaskStepSet(2, 5000);
            break;
        case 2:   /* 腕 P3 平伸（P1/P2 仍不动） */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_P3_FLAT_108, 2000);
            TaskStepSet(3, 2500);
            break;
        case 3:   /* 开始追踪（锁定标靶） */
            Vision_TrackBegin(VISION_SERIES_B, 50);
            TaskStepSet(4, 0);
            break;
        case 4:   /* 走车对准：拿视觉偏移调车（不动大小臂） */
        {
            /* 有新的流式偏移 → 换算成车位微调（Car_SetTarget 逆运动学入口）。
             * dx = 前后偏移（车直行方向），dy = 左右偏移。
             * 简化：dx 映射线速度 v，dy 映射角速度 w（符号联调再定）。 */
            if (g_vision.last_pos_valid)
            {
                g_vision.last_pos_valid = 0;
                float dx = (float)g_vision.last_pos.x;
                float dy = (float)g_vision.last_pos.y;

                float norm = dx * dx + dy * dy;
                if (norm <= ARM_KINE_EPS_MM * ARM_KINE_EPS_MM)
                {
                    /* 对准 → 停追踪 → 开激光 */
                    Vision_TrackStop(VISION_SERIES_B);
                    Car_SetTarget(0.0f, 0.0f);
                    s_lost_cnt = 0;
                    TaskStepSet(5, 0);
                }
                else
                {
                    /* 按偏移微调车位（量纲/增益联调时标定） */
                    float v = dx * 0.001f;   /* 线速度（前后） */
                    float w = dy * 0.001f;   /* 角速度（左右） */
                    Car_SetTarget(v, w);
                }
            }
            else if (g_vision.lost_cnt > s_lost_cnt)
            {
                /* 目标丢失：停车（不停机、继续重新追踪） */
                Car_SetTarget(0.0f, 0.0f);
                Vision_TrackBegin(VISION_SERIES_B, 50);
            }
            break;
        }
        case 5:   /* 开激光 */
            Formal_LaserSet(1);
            TaskStepSet(6, 2000);
            break;
        case 6:   /* 关激光（2 秒，不等 0x35 倒下） */
            Formal_LaserSet(0);
            TaskStepSet(7, 0);
            break;
        case 7:   /* 步进 θ1 转回 0 */
            FormalStepGoTo(0);
            TaskStepSet(8, 5000);
            break;
        case 8:   /* 腕 P3 回位 */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
            TaskStepSet(9, 2500);
            break;
        default:
            TaskComplete();
            break;
    }
}

/* D 任务（救援）：展开→观察→选目标→粗调→抓取伺服→夹取→升起(慢)→收起(慢，夹爪保持95°) */
static void TaskD_Step(void)
{
    switch (s_task_step)
    {
        case 1:   /* 展开：夹爪开 + 舵机先动（慢 4s） */
            PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_DEPLOY_P1, 4000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_DEPLOY_P2, 4000);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_DEPLOY_P3, 4000);
            SnapPwm(ARM_POSE_D_DEPLOY_P1, ARM_POSE_D_DEPLOY_P2, ARM_POSE_D_DEPLOY_P3);
            TaskStepSet(2, 2000);
            break;
        case 2:   /* 步进 θ1 左转 90°（慢速，后动） */
            FormalStepGoToSlow(ARM_TH1_D_MID_PULSE);
            TaskStepSet(3, 5000);
            break;
        case 3:   /* 观察三物体 */
            Vision_ObsThree(VISION_SERIES_C);
            TaskStepSet(4, 0);
            break;
        case 4:   /* 等三目标回包 → 选预存 */
            if (g_vision.obs3.valid)
            {
                uint8_t id = g_vision.obs3.target_id;   /* 1左/2中/3右 */
                if (id == 1)
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_LEFT_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_LEFT_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_LEFT_P3, 2000);
                    FormalStepGoTo(ARM_TH1_D_LEFT_PULSE);
                    SnapPwm(ARM_POSE_D_LEFT_P1, ARM_POSE_D_LEFT_P2, ARM_POSE_D_LEFT_P3);
                }
                else if (id == 3)
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_RIGHT_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_RIGHT_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_RIGHT_P3, 2000);
                    FormalStepGoTo(ARM_TH1_D_RIGHT_PULSE);
                    SnapPwm(ARM_POSE_D_RIGHT_P1, ARM_POSE_D_RIGHT_P2, ARM_POSE_D_RIGHT_P3);
                }
                else
                {
                    Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_MID_P1, 2000);
                    Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_MID_P2, 2000);
                    Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_MID_P3, 2000);
                    FormalStepGoTo(ARM_TH1_D_MID_PULSE);
                    SnapPwm(ARM_POSE_D_MID_P1, ARM_POSE_D_MID_P2, ARM_POSE_D_MID_P3);
                }
                g_vision.obs3.valid = 0;
                s_lost_cnt = 0;
                TaskStepSet(5, 2500);
            }
            break;
        case 5:   /* 开始追踪（抓取段） */
            Vision_TrackBegin(VISION_SERIES_C, 50);
            TaskStepSet(6, 0);
            break;
        case 6:   /* 抓取段伺服闭环 */
        {
            int8_t r = Servo_StepOnce_Pick();
            if (r == 1)
            {
                Vision_TrackStop(VISION_SERIES_C);
                s_lost_cnt = 0;
                TaskStepSet(7, 0);
            }
            else if (g_vision.lost_cnt > s_lost_cnt)
            {
                if (Servo_LostFallback(ARM_POSE_D_MID_P1, ARM_POSE_D_MID_P2, ARM_POSE_D_MID_P3))
                {
                    Vision_ObsThree(VISION_SERIES_C);
                    TaskStepSet(4, 0);
                }
                else
                {
                    Vision_TrackBegin(VISION_SERIES_C, 50);
                }
            }
            break;
        }
        case 7:   /* 下翻夹取：P3 下翻 1210 + 夹爪 95°（B/D 均下翻，非直伸） */
            Servo_SetAngleTime(ARM_TH4_ID, ARM_TH4_PWM_DOWN, 1500);
            PWM_Servo_SetAngle(ARM_GRIP_CLOSE_DEG);
            TaskStepSet(8, 2000);
            break;
        case 8:   /* 升到展开态（中转，慢 4s） */
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_DEPLOY_P1, 4000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_DEPLOY_P2, 4000);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_DEPLOY_P3, 4000);
            SnapPwm(ARM_POSE_D_DEPLOY_P1, ARM_POSE_D_DEPLOY_P2, ARM_POSE_D_DEPLOY_P3);
            TaskStepSet(9, 2000);
            break;
        case 9:   /* 转身 θ1→0（慢速） */
            FormalStepGoToSlow(0);
            TaskStepSet(10, 5000);
            break;
        case 10:  /* 收起（慢，舵机 4s home，夹爪保持 95° 不关） */
            Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_HOME_P1, 4000);
            Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_HOME_P2, 4000);
            Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 4000);
            TaskStepSet(11, 4500);
            break;
        default:
            TaskComplete();
            break;
    }
}

/* ============================================================
 * 任务点分发
 * ============================================================ */

static void TaskStart(uint8_t target_idx)
{
    s_task_step = 1;
    s_step_due  = 0;
    s_lost_cnt  = 0;
    (void)target_idx;
}

static void TaskComplete(void)
{
    /* 边界判断：当前任务点 g_target_idx 已达到终点边界 s_end 时，
     * 不再推进到下一个点，直接结束正式模式并停掉 path_runner。
     * s_end 语义 = 最后一个要执行的目标点索引（AFTER_A=0/AFTER_B=1/AFTER_C=2/AFTER_D=3）。 */
    if (g_target_idx >= (uint8_t)s_end)
    {
        g_formal_state = FORMAL_DONE;
        g_run_state = RUN_ST_IDLE;   /* 停 path_runner（勿置 DONE，避免误触发终点动作） */
        Car_SetTarget(0.0f, 0.0f);
    }
    else
    {
        PathRunner_TaskDone();
    }
}

static void TaskTick(uint8_t target_idx)
{
    switch (target_idx)
    {
        case 0: TaskA_Step(); break;
        case 1: TaskB_Step(); break;
        case 2: TaskC_Step(); break;
        case 3: TaskD_Step(); break;
        default: TaskComplete(); break;
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */

void Formal_Start(formal_end_t end)
{
    s_end = end;
    s_task_step = 0;
    s_step_due  = 0;
    s_lost_cnt  = 0;
    s_force_retract = 0;
    g_formal_state = FORMAL_MOVING;
    PathRunner_Start();
}

void Formal_Tick(void)
{
    if (g_formal_state == FORMAL_IDLE || g_formal_state == FORMAL_DONE)
        return;

    if (g_run_state == RUN_ST_TASK && s_task_step == 0)
    {
        TaskStart(g_target_idx);
        g_formal_state = FORMAL_TASK;
    }

    if (g_formal_state == FORMAL_TASK && s_task_step != 0)
    {
        if ((int32_t)(HAL_GetTick() - s_step_due) >= 0)
        {
            TaskTick(g_target_idx);
        }
    }

    if (g_run_state == RUN_ST_DONE)
    {
        g_formal_state = FORMAL_DONE;
    }
}

uint8_t Formal_IsBusy(void)
{
    return (g_formal_state != FORMAL_IDLE && g_formal_state != FORMAL_DONE);
}

void Formal_Abort(void)
{
    g_formal_state = FORMAL_IDLE;
    s_task_step = 0;
    s_step_due = 0;
    s_lost_cnt = 0;
    s_force_retract = 0;
    Car_SetTarget(0.0f, 0.0f);
}

void Formal_ForceTaskRetract(void)
{
    /* 仅当正在任务阶段（非跑车）时，置手动收起标志；
     * A 任务等 0x11 那一步会响应，跳到收起步。 */
    if (g_formal_state == FORMAL_TASK)
        s_force_retract = 1;
}
