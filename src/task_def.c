/**
 * @file    task_def.c
 * @brief   四个任务点的动作表（纯数据；逻辑全在 task_seq.c）
 *
 * 这些表是从旧 formal_mode.c 的四个巨型 switch 搬过来的，**参数值没动**
 * （它们多数是现场实测/联调出来的），变的是表达方式：
 *   · 旧："发命令 + 睡固定时间"两行一对  →  现在一行，到位判据由引擎按 act 决定
 *   · 旧：left/mid/right 三份复制粘贴    →  现在一句 ACT_POSE_BY_TARGET
 *
 * 所有字段一律用【指定初始化器】写（.act = ...），不要按位置填 —— tstep_t 有十来个
 * 字段，按位置填必然出错。
 *
 * ⚠️ 时序参数（stime_ms / timeout_ms）是【待联调】的：到位判据换成轮询后，
 *    stime_ms 只是给舵机固件的"走多久"，超时才是真正的保险。首次上机要确认
 *    每一步的超时够不够（不够会提前放行、动作没做完就进下一步）。
 */

#include "task_seq.h"
#include "arm_config.h"

/* ============================================================
 * 任务 A：二维码识别
 *   2026-10-14 改版（Ryan 定稿）：
 *     ① P1(第1舵机/大臂) → 1580（零位基准 ARM_POSE_HOME_P1=1892，减 312 ≈ 抬 42°）
 *        P2/P3【不发指令，保持原位】（Ryan：其它两个关节锁死 = 保持不动）
 *        θ1 底盘【顺时针 90°】—— 与 P1 动作【同时启动】
 *     ② 减速扫描（车慢速直行 500，车侧摄像头观察）—— ★ 必须在转向【之后】，
 *        所以扫描放在任务里而不是路径里（见 task_seq.h 的 ACT_CAR_SCAN 说明）
 *     ③ 扫码握手
 *
 * ★ 顺时针 90° 的脉冲增量：代码里"向右转"= 顺时针 = 负增量，转台 90° = 5600，
 *   故 pulses = -5600（ARM_POSE_A_SCAN_TH1_PULSE）。
 * ★ 此处【不发收臂动作】：A 点扫码姿态就是上面这个（P1 抬到 1580），扫完停在扫码姿态。
 * ★ 方向符号依据：Arm.h「ARM_DIR_CCW 实测=转台顺时针」，
 *   与 B 任务"转向桶（右侧）"用的 -ARM_STEP_PULSE_TURNTABLE/4 同源。
 * ============================================================ */
static const tstep_t k_steps_a[] = {
    /* ① P1 摆到 1580 的同时，θ1 顺时针 90°（两者各发各的、同拍启动） */
    {.act = ACT_SERVO1_STEP, .sid1 = ARM_TH2_ID, .spwm1 = ARM_POSE_A_SCAN_P1,
     .stime_ms = 2000, .pulses = ARM_POSE_A_SCAN_TH1_PULSE, .timeout_ms = 8000},

    /* ② 减速扫描：转向完成后车才慢速直行通过目标区。
     * 500mm @ 0.70mm/周期 ≈ 3.6s，超时给 8s 兜底。 */
    {.act = ACT_CAR_SCAN, .arg = 500, .timeout_ms = 8000},

    /* ③ 协议 §3.2 的阻塞式握手：发 0x10 等 0x11。
     * ★ 必须给超时（2026-10-14 加）：这一步的完成判据是"收到 0x11"，没有超时的话，
     *   上位机没连 / 程序没起 / 扫码失败时会**永远等下去** —— 机械臂卡在扫码姿态上，
     *   连按键返回都停不下来（任务引擎还在跑）。10s 后照常往下走。 */
    {.act = ACT_VIS_SCAN, .timeout_ms = 10000},

    {.act = ACT_END},
};
const task_def_t g_task_A = {"二维码识别", k_steps_a};

/* ============================================================
 * 任务 B：小球排爆
 *   展开 → θ1 转中位 → 观察三物体 → 选预存位 → 追踪抓取 → 夹取
 *   → 转桶 → 放置段追踪 → 放入 → 回收
 * ============================================================ */
static const tstep_t k_steps_b[] = {
    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_OPEN_DEG},   /* 夹爪张开 */

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_B_DEPLOY_P1, .spwm2 = ARM_POSE_B_DEPLOY_P2, .spwm3 = ARM_POSE_B_DEPLOY_P3,
     .stime_ms = 3000, .timeout_ms = 5000},          /* 展开 */

    {.act = ACT_STEP1, .pulses = ARM_TH1_B_MID_PULSE, .timeout_ms = 6000},  /* θ1 转中位 */

    {.act = ACT_VIS_OBS3_A, .timeout_ms = 12000},    /* 观察三物体 → 等 0x21 */

    {.act = ACT_POSE_BY_TARGET, .arg = 0, .timeout_ms = 5000}, /* 按 target_id 选预存位 */

    {.act = ACT_VIS_TRACK_A},                        /* 发 0x22 开始追踪 + 伺服闭环 */

    {.act = ACT_SERVO1, .sid1 = ARM_TH4_ID, .spwm1 = ARM_TH4_PWM_DOWN,
     .stime_ms = 1500, .timeout_ms = 3000},          /* 腕下翻 */
    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_CLOSE_DEG},   /* 夹取 */

    {.act = ACT_STEP1, .pulses = -((int32_t)ARM_STEP_PULSE_TURNTABLE / 4),
     .timeout_ms = 8000},                            /* θ1 转向桶（右侧） */

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_BUCKET_P1, .spwm2 = ARM_POSE_BUCKET_P2, .spwm3 = ARM_POSE_BUCKET_P3,
     .stime_ms = 1500, .timeout_ms = 3000},          /* 转桶预存位 */

    {.act = ACT_VIS_TRACK_BUCKET},                   /* 发 0x25 放置段追踪 */

    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_OPEN_DEG},    /* 放入：张开 */
    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_PARK_DEG},    /* 收夹爪中性 */

    {.act = ACT_STEP1, .pulses = 0, .timeout_ms = 8000},      /* θ1 回 0 */

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_HOME_P1, .spwm2 = ARM_POSE_HOME_P2, .spwm3 = ARM_POSE_HOME_P3,
     .stime_ms = 3000, .timeout_ms = 5000},          /* 收起 */

    {.act = ACT_END},
};
const task_def_t g_task_B = {"小球排爆", k_steps_b};

/* ============================================================
 * 任务 C：激光反恐
 *   姿态锁死（高度对准靶心）→ θ1 转位 → 腕平伸 → 追踪 → 车前后移动对准
 *   → 开激光 → 等靶倒下（或超时）→ 关激光 → 回收
 * ============================================================ */
static const tstep_t k_steps_c[] = {
    {.act = ACT_STEP1, .pulses = ARM_TH1_C_MID_PULSE, .timeout_ms = 6000},
    {.act = ACT_SERVO1, .sid1 = ARM_TH4_ID, .spwm1 = ARM_POSE_P3_FLAT_108,
     .stime_ms = 2000, .timeout_ms = 4000},          /* 腕平伸（P1/P2 不动 = 高度锁死） */

    {.act = ACT_VIS_TRACK_B},                        /* 发 0x32 开始追踪 */
    {.act = ACT_CAR_AIM},                            /* 车前后移动对准（w 恒为 0） */

    {.act = ACT_LASER_ON},
    {.act = ACT_WAIT_TARGET_DOWN, .timeout_ms = 5000},  /* 等 0x35；不报就 5s 关 */
    {.act = ACT_LASER_OFF},

    {.act = ACT_STEP1, .pulses = 0, .timeout_ms = 6000},
    {.act = ACT_SERVO1, .sid1 = ARM_TH4_ID, .spwm1 = ARM_POSE_HOME_P3,
     .stime_ms = 2000, .timeout_ms = 4000},

    {.act = ACT_END},
};
const task_def_t g_task_C = {"激光反恐", k_steps_c};

/* ============================================================
 * 任务 D：抓取人质
 *   与 B 的抓取段同构，但全用慢速（避免晃动带偏人质），且收起时夹爪保持夹持
 * ============================================================ */
static const tstep_t k_steps_d[] = {
    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_OPEN_DEG},

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_D_DEPLOY_P1, .spwm2 = ARM_POSE_D_DEPLOY_P2, .spwm3 = ARM_POSE_D_DEPLOY_P3,
     .stime_ms = 4000, .timeout_ms = 6000},          /* 展开（慢） */

    {.act = ACT_STEP1_SLOW, .pulses = ARM_TH1_D_MID_PULSE, .timeout_ms = 8000},

    {.act = ACT_VIS_OBS3_C, .timeout_ms = 12000},    /* 观察三物体 → 等 0x41 */

    {.act = ACT_POSE_BY_TARGET, .arg = 1, .timeout_ms = 5000}, /* 用 D 的预存位 */

    {.act = ACT_VIS_TRACK_C},                        /* 发 0x42 开始追踪 + 伺服闭环 */

    {.act = ACT_SERVO1, .sid1 = ARM_TH4_ID, .spwm1 = ARM_TH4_PWM_DOWN,
     .stime_ms = 1500, .timeout_ms = 3000},
    {.act = ACT_GRIP, .arg = (uint16_t)ARM_GRIP_CLOSE_DEG},   /* 夹取（保持夹持到最后） */

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_D_DEPLOY_P1, .spwm2 = ARM_POSE_D_DEPLOY_P2, .spwm3 = ARM_POSE_D_DEPLOY_P3,
     .stime_ms = 4000, .timeout_ms = 6000},          /* 升到展开态（中转） */

    {.act = ACT_STEP1_SLOW, .pulses = 0, .timeout_ms = 8000},

    {.act = ACT_SERVO3, .sid1 = ARM_TH2_ID, .sid2 = ARM_TH3_ID, .sid3 = ARM_TH4_ID,
     .spwm1 = ARM_POSE_HOME_P1, .spwm2 = ARM_POSE_HOME_P2, .spwm3 = ARM_POSE_HOME_P3,
     .stime_ms = 4000, .timeout_ms = 6000},          /* 收起（夹爪不关，人质带回） */

    {.act = ACT_END},
};
const task_def_t g_task_D = {"抓取人质", k_steps_d};
