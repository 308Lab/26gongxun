#include "Car_control.h"
#include <math.h>
#include "odometer.h"

/* ============================================================================
 * Car_control.c —— 车控模块实现
 *
 * 函数按功能块分类（与 .h 对应）：
 *   A. 全局量 / 状态变量
 *   B. 初始化
 *   C. 直行速度环（含直行角度环）
 *   D. 转弯差速速度环 + 逆运动学入口
 *   E. 航向 / 角度公共工具
 *   F. 原地转弯（Turn）
 *   G. 直行距离（Go）
 *   H. 弧线转（Arc）
 *   I. 编码器累计
 * ========================================================================== */

/* ==================== A. 全局量 / 状态变量 ==================== */

int32_t g_enc_l = 0;
int32_t g_enc_r = 0;
int32_t enc_total_l = 0;
int32_t enc_total_r = 0;

/* 调试量：弧线转左右轮 PWM / 反馈（OLED 临时显示） */
int32_t g_dbg_outL = 0;
int32_t g_dbg_outR = 0;
int32_t g_dbg_fbL  = 0;
int32_t g_dbg_fbR  = 0;

/* 调试量：直行角度环内部量（OLED 临时显示） */
int32_t g_dbg_heading_err = 0;   /* 航向误差 err(度) ×10 */
int32_t g_dbg_heading_dv  = 0;   /* 速度差 Δv(mm/周期) ×1000 */

/* 直行速度环 PID（左右轮独立） */
static PosPID_TypeDef pid_L;
static PosPID_TypeDef pid_R;

/* 原地转弯角度环 */
static PosPID_TypeDef pid_turn;
/* 【临时注释·Turn速度环】原地转内环：角速度环 */
static PosPID_TypeDef pid_turn_speed;

/* 【临时注释·Turn速度环】调试量 */
int32_t g_dbg_turn_w_set = 0;
int32_t g_dbg_turn_w_fb  = 0;

/* 弧线转角度环（外环）+ 左右轮速度环（内环） */
static PosPID_TypeDef pid_arc;
static PosPID_TypeDef pid_arcL;
static PosPID_TypeDef pid_arcR;

/* 倒车专用速度环（允许负输出）：pid_L/pid_R 的 out_min=0 会鉝掉负 PWM，
 * 倒车目标速度是负的，故单独建两个允许负输出的 PID。 */
static PosPID_TypeDef pid_backL;
static PosPID_TypeDef pid_backR;

/* 直行角度环（航向锁定） */
static PosPID_TypeDef pid_heading;

/* 直行角度环状态 */
static float   heading_target = 0.0f;
static uint8_t heading_locked = 0;

/* 弧线转轮距修正值（标定旋钮） */
float wheel_base_correction = WHEEL_BASE_CORR_INIT_MM;

/* 陀螺仪 yaw 基准偏移：Car_YawCalibrate 把当前 yaw 记为 offset */
static float yaw_offset = 0.0f;

/* ==================== B. 初始化 ==================== */

void Car_Control_Init(void)
{
    Motor_Init();

    /* 直行速度环 */
    PosPID_Init(&pid_L, L_KP, L_KI, L_KD, (float)L_MAX, (float)L_MIN, 500.0f, -500.0f);
    PosPID_Init(&pid_R, R_KP, R_KI, R_KD, (float)R_MAX, (float)R_MIN, 500.0f, -500.0f);

    /* 原地转弯角度环 */
    PosPID_Init(&pid_turn, TURN_KP, TURN_KI, TURN_KD,
                TURN_PWM_MAX, -TURN_PWM_MAX, TURN_INTEGRAL_MAX, -TURN_INTEGRAL_MAX);

    /* 【临时注释·Turn速度环】原地转内环角速度环 */
    PosPID_Init(&pid_turn_speed, TURN_SPEED_KP, TURN_SPEED_KI, TURN_SPEED_KD,
                TURN_SPEED_OUT_MAX, -TURN_SPEED_OUT_MAX,
                TURN_SPEED_OUT_MAX, -TURN_SPEED_OUT_MAX);

    /* 弧线转角度环（外环），输出角速度修正 rad/周期 */
    PosPID_Init(&pid_arc, ARC_KP, ARC_KI, ARC_KD,
                ARC_CORR_MAX, -ARC_CORR_MAX, ARC_CORR_MAX, -ARC_CORR_MAX);

    /* 弧线转左右轮速度环（内环），允许负输出 */
    PosPID_Init(&pid_arcL, ARC_SPEED_KP, ARC_SPEED_KI, ARC_SPEED_KD,
                ARC_SPEED_OUT_MAX, -ARC_SPEED_OUT_MAX, 500.0f, -500.0f);
    PosPID_Init(&pid_arcR, ARC_SPEED_KP, ARC_SPEED_KI, ARC_SPEED_KD,
                ARC_SPEED_OUT_MAX, -ARC_SPEED_OUT_MAX, 500.0f, -500.0f);

    /* 直行角度环 */
    PosPID_Init(&pid_heading, HEADING_KP, HEADING_KI, HEADING_KD,
                HEADING_CORR_MAX, -HEADING_CORR_MAX,
                HEADING_CORR_MAX, -HEADING_CORR_MAX);

    /* 倒车速度环：复用直行环增益（L_KP/L_KI/L_KD），但允许负输出（±999） */
    PosPID_Init(&pid_backL, L_KP, L_KI, L_KD, 999.0f, -999.0f, 500.0f, -500.0f);
    PosPID_Init(&pid_backR, R_KP, R_KI, R_KD, 999.0f, -999.0f, 500.0f, -500.0f);
}

/* ==================== C. 直行速度环 ==================== */

/*
 * 直行速度环：输入左右轮目标速度(mm/周期)，前馈+PID 出 PWM。
 * PID 反馈来自里程计 Odom_GetVL()/Odom_GetVR()（mm/周期，与目标同量纲）。
 */
void Go_Ahead_WithPID(float target_speed_L, float target_speed_R)
{
    float fb_L = Odom_GetVL();   /* 左轮实测速度 mm/周期 */
    float fb_R = Odom_GetVR();   /* 右轮实测速度 mm/周期 */

    /* 机械偏补偿 */
    target_speed_L += SPEED_BIAS_L;
    target_speed_R += SPEED_BIAS_R;

    /* 直行角度环（航向锁定）：yaw 误差 → 左右轮速度差 Δv。
     * 符号约定（已验）：车左转 → yaw 增大；右转 → yaw 减小。
     * Δv 加给右轮、减给左轮（左轮快=车右转，故反向）。 */
    if (heading_locked)
    {
        float heading_err = AngleDiff_Normalize(heading_target, Car_GetYaw());
        /* PosPID 内部 err = target - feedback。把 heading_err 当 target、feedback=0，
         * 使内部 err = heading_err，输出符号与航向误差一致。 */
        float heading_dv  = PosPID_Calc(&pid_heading, heading_err, 0.0f);
        target_speed_L -= heading_dv;
        target_speed_R += heading_dv;

        g_dbg_heading_err = (int32_t)(heading_err * 10.0f);
        g_dbg_heading_dv  = (int32_t)(heading_dv  * 1000.0f);
    }

#if USE_FEEDFORWARD
    float ff_L = target_speed_L * L_FEEDFORWARD_GAIN;
    float ff_R = target_speed_R * R_FEEDFORWARD_GAIN;

    float trim_L = PosPID_Calc(&pid_L, 0.0f, fb_L - target_speed_L);
    float trim_R = PosPID_Calc(&pid_R, 0.0f, fb_R - target_speed_R);

    float out_L = ff_L + trim_L;
    float out_R = ff_R + trim_R;
#else
    float out_L = PosPID_Calc(&pid_L, target_speed_L, fb_L);
    float out_R = PosPID_Calc(&pid_R, target_speed_R, fb_R);
#endif

    /* 限幅（直行不倒退，负输出钳 0） */
    if (out_L > 999.0f) out_L = 999.0f;
    if (out_L < 0.0f)   out_L = 0.0f;
    if (out_R > 999.0f) out_R = 999.0f;
    if (out_R < 0.0f)   out_R = 0.0f;

    SetWheelSpeed((int32_t)out_L, (int32_t)out_R);
}

/* ==================== D. 转弯差速速度环 + 逆运动学入口 ==================== */

/*
 * 转弯差速速度环（弧线转专用）：输入左右轮目标速度(mm/周期)，输出带符号 PWM。
 * 与直行环区别：允许负输出；不叠加机械偏补偿（转弯要精确差速）。
 */
void Go_Turn_WithPID(float target_speed_L, float target_speed_R)
{
    float fb_L = Odom_GetVL();
    float fb_R = Odom_GetVR();

    float ff_L = target_speed_L * L_FEEDFORWARD_GAIN;
    float ff_R = target_speed_R * R_FEEDFORWARD_GAIN;

    float trim_L = PosPID_Calc(&pid_arcL, 0.0f, fb_L - target_speed_L);
    float trim_R = PosPID_Calc(&pid_arcR, 0.0f, fb_R - target_speed_R);

    float out_L = ff_L + trim_L;
    float out_R = ff_R + trim_R;

    /* 对称 ±999 限幅（保留负号用于差速/倒转） */
    if (out_L >  999.0f) out_L =  999.0f;
    if (out_L < -999.0f) out_L = -999.0f;
    if (out_R >  999.0f) out_R =  999.0f;
    if (out_R < -999.0f) out_R = -999.0f;

    /* 侧滑补偿：内侧轮（|目标速度|较小侧）额外加固定 PWM，方向同目标符号 */
    if (fabsf(target_speed_L) < fabsf(target_speed_R))
    {
        out_L += (target_speed_L >= 0.0f ? 1.0f : -1.0f) * (float)ARC_SIDE_SLIP_COMP;
    }
    else
    {
        out_R += (target_speed_R >= 0.0f ? 1.0f : -1.0f) * (float)ARC_SIDE_SLIP_COMP;
    }

    if (out_L >  999.0f) out_L =  999.0f;
    if (out_L < -999.0f) out_L = -999.0f;
    if (out_R >  999.0f) out_R =  999.0f;
    if (out_R < -999.0f) out_R = -999.0f;

    g_dbg_outL = (int32_t)out_L;
    g_dbg_outR = (int32_t)out_R;
    g_dbg_fbL  = (int32_t)(fb_L * 1000.0f);
    g_dbg_fbR  = (int32_t)(fb_R * 1000.0f);

    SetWheelSpeed((int32_t)out_L, (int32_t)out_R);
}

/*
 * 逆运动学入口：车体中心线速度 v(mm/周期) + 角速度 w(rad/周期)
 *   → 左右履带目标速度 → 转弯差速速度环。
 */
void Car_SetTarget(float v_mm_per_cycle, float w_rad_per_cycle)
{
    /* 零速守卫：v、w 都为 0 表示「停车」，直接物理归零 + 清转弯积分。
     * 修复「收起后持续左转」：原实现只把目标速度设 0，转弯 PID 积分
     * (pid_arcL/pid_arcR) 未清 + 零目标走侧滑补偿 else 给右轮 +ARC_SIDE_SLIP_COMP，
     * 形成持续左转偏置。这里统一等价 Stop()。 */
    if (v_mm_per_cycle == 0.0f && w_rad_per_cycle == 0.0f)
    {
        PosPID_ClearAll(&pid_arcL);
        PosPID_ClearAll(&pid_arcR);
        Stop();
        return;
    }

    float B_eff = WHEEL_BASE_GEO_MM + wheel_base_correction;
    float half = w_rad_per_cycle * B_eff / 2.0f;   /* mm/周期 */

    float vL_target = v_mm_per_cycle - half;
    float vR_target = v_mm_per_cycle + half;

    Go_Turn_WithPID(vL_target, vR_target);
}

/* ==================== E. 航向 / 角度公共工具 ==================== */

float Car_GetYaw(void)
{
    Gyro_Angles_t a = Gyro_GetAngles();
    return AngleDiff_Normalize(a.yaw, yaw_offset);
}

void Car_YawCalibrate(void)
{
    Gyro_Angles_t a = Gyro_GetAngles();
    yaw_offset = a.yaw;
}

/* 角度差归一化到 [-180, 180]，跨 ±180 边界正确 */
float AngleDiff_Normalize(float a, float b)
{
    float d = a - b;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

float Car_TurnError(float target_yaw)
{
    return AngleDiff_Normalize(target_yaw, Car_GetYaw());
}

/* ==================== F. 原地转弯（Turn，扫描式非阻塞） ==================== */

static uint8_t  turn_active = 0;       /* 0=空闲，1=转弯中，2=刹车中 */
static float    turn_target = 0.0f;
static uint32_t turn_stable_cnt = 0;
static uint32_t turn_brake_cnt = 0;    /* 【临时注释·Turn反向脉冲】刹车周期计数 */
static int8_t   turn_dir = 0;          /* 【临时注释·Turn反向脉冲】+1左/-1右，用于反向脉冲方向 */

void Turn_Start(float rel_deg)
{
    float yaw0 = Car_GetYaw();
    turn_target = yaw0 + rel_deg;
    turn_stable_cnt = 0;
    turn_brake_cnt = 0;
    turn_dir = (rel_deg >= 0.0f) ? 1 : -1;   /* 【临时注释·Turn反向脉冲】记录转向 */
    PosPID_ClearAll(&pid_turn);
    PosPID_ClearAll(&pid_turn_speed); /* 【临时注释·Turn速度环】 */
    turn_active = 1;
}

void Turn_Update(void)
{
    if (turn_active == 0)
    {
        return;
    }

    /* 【临时注释·Turn反向脉冲】刹车态：反向差速脉冲吃惯性，持续 TURN_BRAKE_TICKS → Stop */
    if (turn_active == 2)
    {
        if (++turn_brake_cnt >= TURN_BRAKE_TICKS)
        {
            Stop();
            turn_active = 0;
        }
        return;
    }

    float cur = Car_GetYaw();
    float err = AngleDiff_Normalize(turn_target, cur);

    /* 到位判据：|err| < 容差 且连续稳定 → 进刹车态（反向脉冲，不吃惯性滑） */
    if (fabsf(err) < TURN_TOLERANCE_DEG)
    {
        if (++turn_stable_cnt >= TURN_STABLE_CNT)
        {
            int32_t brake;
            if (turn_dir > 0)   /* 左转 → 反向朝右刹（左轮+，右轮-） */
            {
                brake = +TURN_BRAKE_PWM;
            }
            else                /* 右转 → 反向朝左刹（左轮-，右轮+） */
            {
                brake = -TURN_BRAKE_PWM;
            }
            SetWheelSpeed(brake, -brake);
            turn_active = 2;
            turn_brake_cnt = 0;
            return;
        }
    }
    else
    {
        turn_stable_cnt = 0;
    }

    /* 【临时注释·Turn速度环】外环：yaw位置环 → 目标角速度 ω_set。
     * 目标 0（追误差归零），反馈 err；输出符号与 err 一致（+左/-右）。 */
    float w_set = PosPID_Calc(&pid_turn, 0.0f, err);

    /* 【临时注释·Turn速度环】内环：编码器角速度反馈追 ω_set → PWM。
     * ω_实测 = (vR - vL)/B_eff，右侧快→左转（符号同 yaw）。 */
    float B_eff = WHEEL_BASE_GEO_MM + wheel_base_correction;
    float w_fb  = (Odom_GetVR() - Odom_GetVL()) / B_eff;

    float out = PosPID_Calc(&pid_turn_speed, w_set, w_fb);

    /* 【临时注释·Turn速度环】仅保留极小启动死区：防 PWM 毛刺抖动。
     * 静摩擦由速度环闭环顶，故不再依赖固定 150（载重变化不再失效）。 */
    if (out > 0.0f && out <  TURN_SPEED_DEADZONE) out =  TURN_SPEED_DEADZONE;
    if (out < 0.0f && out > -TURN_SPEED_DEADZONE) out = -TURN_SPEED_DEADZONE;

    /* 限幅（冗余保护） */
    if (out >  TURN_SPEED_OUT_MAX) out =  TURN_SPEED_OUT_MAX;
    if (out < -TURN_SPEED_OUT_MAX) out = -TURN_SPEED_OUT_MAX;

    g_dbg_turn_w_set = (int32_t)(w_set * 100000.0f);
    g_dbg_turn_w_fb  = (int32_t)(w_fb  * 100000.0f);

    /* 原地差速：一正一反 */
    SetWheelSpeed((int32_t)out, -(int32_t)out);
}

uint8_t Turn_IsActive(void)
{
    return turn_active;
}

/* ==================== G. 直行距离（Go，扫描式非阻塞） ==================== */

static uint8_t  go_active = 0;       /* 0=空闲，1=直行中，2=刹车中 */
static float    go_target = 0.0f;
static uint32_t go_brake_cnt = 0;
static float    go_speed_L = TEST_BASE_SPEED_L;   /* 直行目标速度(mm/周期)，Scan 复写为慢速 */
static float    go_speed_R = TEST_BASE_SPEED_R;

void Go_Start(float dist_mm, float heading_deg)
{
    Odom_Reset();
    go_target = dist_mm;
    go_active = 1;
    go_brake_cnt = 0;
    go_speed_L = TEST_BASE_SPEED_L;
    go_speed_R = TEST_BASE_SPEED_R;

    heading_target = heading_deg;
    PosPID_ClearAll(&pid_heading);
    heading_locked = 1;
}

void Go_Update(void)
{
    if (go_active == 0)
    {
        return;
    }

    if (go_active == 1)
    {
        /* 距离判据：走满目标距离进刹车态 */
        if (Odom_GetDistanceMM() >= go_target)
        {
            go_active = 2;
            go_brake_cnt = 0;
            SetWheelSpeed(-BRAKE_PWM, -BRAKE_PWM);
            return;
        }

        Go_Ahead_WithPID(go_speed_L, go_speed_R);
    }
    else if (go_active == 2)
    {
        if (++go_brake_cnt >= BRAKE_TICKS)
        {
            Stop();
            go_active = 0;
        }
    }
}

uint8_t Go_IsActive(void)
{
    return go_active;
}

/* 倒车专用速度环：前馈 + pid_backL/pid_backR 微调（允许负输出）。
 * 与 Go_Turn_WithPID 结构一致：负目标速度 → 前馈给负 PWM → PID 微调。
 * 不用直行环 Go_Ahead_WithPID（它会把负输出鉝 0）。 */
static void Back_WithPID(float target_speed_L, float target_speed_R)
{
    float fb_L = Odom_GetVL();
    float fb_R = Odom_GetVR();

    float ff_L = target_speed_L * L_FEEDFORWARD_GAIN;
    float ff_R = target_speed_R * R_FEEDFORWARD_GAIN;

    /* 增量式借用位置式 PID：target=0，feedback=fb-target，得 err=target-fb */
    float trim_L = PosPID_Calc(&pid_backL, 0.0f, fb_L - target_speed_L);
    float trim_R = PosPID_Calc(&pid_backR, 0.0f, fb_R - target_speed_R);

    float out_L = ff_L + trim_L;
    float out_R = ff_R + trim_R;

    /* 对称 ±999 限幅（保留负号用于倒车） */
    if (out_L >  999.0f) out_L =  999.0f;
    if (out_L < -999.0f) out_L = -999.0f;
    if (out_R >  999.0f) out_R =  999.0f;
    if (out_R < -999.0f) out_R = -999.0f;

    SetWheelSpeed((int32_t)out_L, (int32_t)out_R);
}

/* ==================== G.5 减速扫描（Scan）与倒车（Back） ==================== */

static uint8_t  scan_active = 0;      /* 0=空闲，1=扫描中（前进），2=刹车中 */
static float    scan_target = 0.0f;
static uint32_t scan_brake_cnt = 0;

void Scan_Start(float dist_mm, float heading_deg)
{
    Odom_Reset();
    scan_target = dist_mm;
    scan_active = 1;
    scan_brake_cnt = 0;

    heading_target = heading_deg;
    PosPID_ClearAll(&pid_heading);
    heading_locked = 1;
}

void Scan_Update(void)
{
    if (scan_active == 0)
    {
        return;
    }

    if (scan_active == 1)
    {
        /* 慢速前进，走满目标区域长度 → 进刹车态 */
        if (Odom_GetDistanceMM() >= scan_target)
        {
            scan_active = 2;
            scan_brake_cnt = 0;
            SetWheelSpeed(-SCAN_BRAKE_PWM, -SCAN_BRAKE_PWM);
            return;
        }

        Go_Ahead_WithPID(SCAN_BASE_SPEED_L, SCAN_BASE_SPEED_R);
    }
    else if (scan_active == 2)
    {
        if (++scan_brake_cnt >= SCAN_BRAKE_TICKS)
        {
            Stop();
            scan_active = 0;
        }
    }
}

uint8_t Scan_IsActive(void)
{
    return scan_active;
}

static uint8_t  back_active = 0;      /* 0=空闲，1=倒车中，2=刹车中 */
static float    back_target = 0.0f;
static uint32_t back_brake_cnt = 0;

void Back_Start(float dist_mm)
{
    Odom_Reset();
    back_target = dist_mm;
    back_active = 1;
    back_brake_cnt = 0;
    PosPID_ClearAll(&pid_backL);
    PosPID_ClearAll(&pid_backR);
}

void Back_Update(void)
{
    if (back_active == 0)
    {
        return;
    }

    if (back_active == 1)
    {
        /* 倒车：编码器增量负向，Odom_GetDistanceMM 递减（车体中心位移为负）。
         * 用 |distance| 判走满倒回距离。 */
        float d = Odom_GetDistanceMM();
        if (fabsf(d) >= back_target)
        {
            back_active = 2;
            back_brake_cnt = 0;
            SetWheelSpeed(SCAN_BRAKE_PWM, SCAN_BRAKE_PWM);   /* 反打正向刹车吃惯性 */
            return;
        }

        /* 反向直行：不锁航向（倒车锁航向会反），用倒车专用速度环（pid_L/pid_R 闭环） */
        Back_WithPID(-BACK_BASE_SPEED_L, -BACK_BASE_SPEED_R);
    }
    else if (back_active == 2)
    {
        if (++back_brake_cnt >= SCAN_BRAKE_TICKS)
        {
            Stop();
            back_active = 0;
        }
    }
}

uint8_t Back_IsActive(void)
{
    return back_active;
}

/* ==================== H. 弧线转（Arc，扫描式非阻塞） ==================== */

static uint8_t  arc_active = 0;       /* 0=空闲，1=弧线转中，2=刹车中 */
static float    arc_target = 0.0f;
static int8_t   arc_dir = 0;          /* +1 左转 / -1 右转 */
static uint32_t arc_stable_cnt = 0;
static uint32_t arc_brake_cnt = 0;

void Arc_Start(int8_t dir, float deg)
{
    float yaw0 = Car_GetYaw();
    arc_target = yaw0 + (float)dir * deg;
    arc_dir = dir;
    arc_stable_cnt = 0;
    arc_brake_cnt = 0;
    PosPID_ClearAll(&pid_arc);
    PosPID_ClearAll(&pid_arcL);
    PosPID_ClearAll(&pid_arcR);
    arc_active = 1;
}

void Arc_Update(void)
{
    if (arc_active == 0)
    {
        return;
    }

    /* 刹车态：反向 PWM 吃惯性，持续 BRAKE_TICKS → Stop */
    if (arc_active == 2)
    {
        if (++arc_brake_cnt >= BRAKE_TICKS)
        {
            Stop();
            arc_active = 0;
        }
        return;
    }

    /* ==== 转弯态（arc_active == 1） ==== */
    float cur = Car_GetYaw();
    float err_deg = AngleDiff_Normalize(arc_target, cur);

    /* 到位 → 反向差速刹车（刹角速度） */
    if (fabsf(err_deg) < ARC_TOLERANCE_DEG)
    {
        if (++arc_stable_cnt >= ARC_STABLE_CNT)
        {
            int32_t brake_l, brake_r;
            if (arc_dir > 0)   /* 左转 → 刹车向右 */
            {
                brake_l =  ARC_BRAKE_PWM;
                brake_r = -ARC_BRAKE_PWM;
            }
            else               /* 右转 → 刹车向左 */
            {
                brake_l = -ARC_BRAKE_PWM;
                brake_r =  ARC_BRAKE_PWM;
            }
            SetWheelSpeed(brake_l, brake_r);
            arc_active = 2;
            arc_brake_cnt = 0;
            return;
        }
    }
    else
    {
        arc_stable_cnt = 0;
    }

    /* 外环：yaw 角度环 → 角速度修正 w_corr（rad/周期） */
    float w_corr = PosPID_Calc(&pid_arc, 0.0f, -err_deg);

    /* 开环基线角速度 w_nominal = dir × v/R，临近目标按比例减速（治过冲） */
    float arc_scale = 1.0f;
    if (fabsf(err_deg) < ARC_SLOWDOWN_DEG)
    {
        arc_scale = fabsf(err_deg) / ARC_SLOWDOWN_DEG;
        if (arc_scale < ARC_SLOWDOWN_MIN) arc_scale = ARC_SLOWDOWN_MIN;
    }
    float w_nominal = (float)arc_dir * (V_TURN_MM_PER_CYCLE / TURN_RADIUS_MM) * arc_scale;

    float w_total = w_nominal + w_corr;

    Car_SetTarget(V_TURN_MM_PER_CYCLE, w_total);
}

uint8_t Arc_IsActive(void)
{
    return arc_active;
}

/* ==================== I. 编码器累计 ==================== */

void Encoder_Total_Counter(int32_t enc_l, int32_t enc_r)
{
    enc_total_l += enc_l;
    enc_total_r += enc_r;
}
