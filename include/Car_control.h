#ifndef __CAR_CONTROL_H
#define __CARCONTROL_H

#include "main.h"
#include "MotorConfig.h"
#include "Motor.h"
#include "Encoder.h"
#include "cmsis_os.h"
#include "IncPID.h"
#include "PosPID.h"
#include "Gyro_JY61P.h"

/* ============================================================================
 * Car_control.h —— 车控模块（直行/原地转/弧线转 三类运动控制）
 *
 * 本文件按「功能块」分类收纳：
 *   1. 全局量声明（编码器/调试量）
 *   2. 直行速度环 —— 参数 + 接口
 *   3. 航向/角度公共工具 —— yaw 读取、校准、归一化
 *   4. 原地转弯（Turn）—— 参数 + 接口
 *   5. 直行距离（Go）—— 参数 + 接口
 *   6. 弧线转（Arc）—— 参数 + 接口
 *   7. 逆运动学入口（Car_SetTarget）
 * ========================================================================== */

/* ==================== 1. 全局量声明 ==================== */

extern osMessageQueueId_t EncQueueHandle;

/* 编码器原始脉冲（供调试/外部引用） */
extern int32_t g_enc_l;
extern int32_t g_enc_r;
extern int32_t enc_total_l;
extern int32_t enc_total_r;

/* 调试量（OLED 临时显示） */
extern int32_t g_dbg_outL;             /* 弧线转左轮 PWM 输出 */
extern int32_t g_dbg_outR;             /* 弧线转右轮 PWM 输出 */
extern int32_t g_dbg_fbL;              /* 弧线转左轮反馈速度 (mm/周期×1000) */
extern int32_t g_dbg_fbR;              /* 弧线转右轮反馈速度 (mm/周期×1000) */
extern int32_t g_dbg_heading_err;      /* 直行航向误差 err(度)×10 */
extern int32_t g_dbg_heading_dv;       /* 直行速度差 Δv(mm/周期)×1000 */

/* ==================== 2. 直行速度环 ==================== */

/* 左右轮速度环位置式 PID 参数（目标/反馈均 mm/周期） */
#define L_KP        0.5f
#define L_KI        0.05f
#define L_KD        0.00f
#define L_MAX       999
#define L_MIN       0.0

#define R_KP        0.5f
#define R_KI        0.05f
#define R_KD        0.00f
#define R_MAX       999
#define R_MIN       0.0

/* 前馈系数（目标速度 → PWM 粗映射），左右轮分开。
 * ⚠️ 量纲 mm/周期：FF = PWM/(mm/周期)。原脉冲口径 0.59 ≈ 0.59/0.002244 ≈ 263。 */
#define L_FEEDFORWARD_GAIN   263.0f   /* PWM/(mm/周期)，左轮 */
#define R_FEEDFORWARD_GAIN   252.0f   /* PWM/(mm/周期)，右轮 */

/* 是否启用前馈：1=前馈+PID微调（推荐），0=纯PID */
#define USE_FEEDFORWARD      1

/* 一阶低通滤波（编码器反馈平滑，当前预留未用） */
#define SPEED_FILTER_ALPHA   0.3f

/* 直行机械偏补偿：抵消机械/结构不对称造成的偏航。
 * 车持续右偏 → 给左轮目标加正偏置（左多走）。 */
#define SPEED_BIAS_L         0        /* 左轮目标偏置（mm/周期） */
#define SPEED_BIAS_R         0        /* 右轮目标偏置（mm/周期） */

/* 直行角度环（航向锁定，治直行跑偏）。
 * 直行时记录目标航向 heading_target，每周期 yaw 误差经 PID 输出左右轮速度差 Δv，
 * 把车头拉回正方向。 */
#define HEADING_KP            0.06f   /* mm/度：KP×err(度)=Δv(mm/周期) */
#define HEADING_KI            0.0f
#define HEADING_KD            0.0f
#define HEADING_CORR_MAX      0.8f    /* Δv 限幅(mm/周期)，防过度差速 */

/* 直行速度环接口（供弧线转内环复用，允许给定左右轮目标速度） */
void Go_Ahead_WithPID(float target_speed_L, float target_speed_R);

/* 转弯差速速度环：允许负值（内外侧轮差速/倒转），输出带符号。
 * 不复用直行速度环（直行环会钳掉负输出，转弯差速会出错）。 */
void Go_Turn_WithPID(float target_speed_L, float target_speed_R);

/* ==================== 3. 航向 / 角度公共工具 ==================== */

/* 读取当前 Z 角度（偏航角 yaw，±180°，相对 yaw_offset 基准） */
float Car_GetYaw(void);

/* 校准：把当前 yaw 记为基准（归零）。后续 Car_GetYaw 返回相对基准的航向。
 * 用于「到位后归零」，使后续路径航向以该点朝向为 0° 基准。 */
void Car_YawCalibrate(void);

/* 角度差归一化到 ±180°，跨 ±180 边界也正确 */
float AngleDiff_Normalize(float a, float b);

/* 转弯到达判断：当前 yaw 与目标 yaw 的误差（度） */
float Car_TurnError(float target_yaw);

/* ==================== 4. 原地转弯（Turn，扫描式非阻塞） ==================== */

/* 原地转弯角度环 PID 参数（起步纯 P，后续按需加 Ki/Kd） */
#define TURN_KP           6.0f
#define TURN_KI           0.0f
#define TURN_KD           0.0f
#define TURN_PWM_MAX      400     /* 原地转 PWM 上限 */
#define TURN_PWM_MIN      150     /* 最小启动 PWM（死区），克服静摩擦防卡住 */
#define TURN_INTEGRAL_MAX 300.0f  /* 积分限幅（anti-windup） */

/* 【临时注释·Turn速度环】原地转内环：角速度环（载重变化后替代固定死区 150）。
 * 外环 yaw位置环(pid_turn) 输出目标角速度 ω_set(rad/周期)；
 * 内环速度环(pid_turn_speed) 用编码器角速度反馈追 ω_set → PWM。
 * 反馈口径：ω_实测 = (vR - vL)/B_eff（rad/周期，右侧快→左转，符号同 yaw）。 */
#define TURN_SPEED_KP     1.5f    /* 比例：2.0→1.5，减少过冲冲劲 */
#define TURN_SPEED_KI     0.2f    /* 积分：0.1→0.2，补静差解决"差一点不到位" */
#define TURN_SPEED_KD     0.3f    /* 微分：0.5→0.3，别把目标速度压过头致提前停 */
#define TURN_SPEED_OUT_MAX 400.0f /* 内环 PWM 上限，与 TURN_PWM_MAX 一致 */
#define TURN_SPEED_DEADZONE 0.0f  /* 【临时】启动死区：→0，速度环纯闭环（静摩擦由闭环顶，死区会致震荡） */

/* 到位判据：|yaw误差| < 容差 且连续 N 周期稳定 */
#define TURN_TOLERANCE_DEG 2.0f
#define TURN_STABLE_CNT    10     /* 连续 10 周期(≈50ms)稳定才判到位 */

/* 【临时注释·Turn速度环】调试量：外环目标角速度/实际角速度（OLED 临时显示） */
extern int32_t g_dbg_turn_w_set;  /* ω_set(rad/周期)×100000 */
extern int32_t g_dbg_turn_w_fb;   /* ω_实测(rad/周期)×100000 */

/* 【临时注释·Turn反向脉冲】到位反向差速刹车：打反向脉冲吃惯性，防滑过目标震荡。 */
#define TURN_BRAKE_PWM    260     /* 反向差速刹车 PWM（与 ARC_BRAKE_PWM 对齐） */
#define TURN_BRAKE_TICKS  10      /* 刹车持续周期（5ms/周期→约50ms） */

/* rel_deg>0 左转、<0 右转 */
void Turn_Start(float rel_deg);   /* 设目标+清PID+置active，立即返回 */
void Turn_Update(void);           /* 每周期刷，到位自动 Stop */
uint8_t Turn_IsActive(void);

/* ==================== 5. 直行距离（Go，扫描式非阻塞） ==================== */

/* 直行基础速度（mm/周期）= 750 脉冲标准直道速 */
#define TEST_BASE_SPEED_L    1.68f
#define TEST_BASE_SPEED_R    1.68f

/* 到位反打刹车：判到位后反向小 PWM 吃惯性，再停 */
#define BRAKE_PWM            180     /* 反向刹车 PWM（太强会倒退） */
#define BRAKE_TICKS          10      /* 刹车持续周期（5ms/周期→约50ms） */

/* Go_Start 清里程计并启动；Go_Update 每周期刷，走满距离自动 Stop。
 * heading_deg = 直行目标航向（相对 yaw 基准，由 path_runner 传累计理论航向）。 */
void Go_Start(float dist_mm, float heading_deg);
void Go_Update(void);
uint8_t Go_IsActive(void);

/* ==================== 6. 弧线转（Arc，扫描式非阻塞） ==================== */

/* 轮距双参数：几何实测 + 标定修正（带入 = 两者之和）。履带转向侧滑，用修正值吸滑移。 */
#define WHEEL_BASE_GEO_MM           198.48f   /* 履带中心距（实测） */
#define WHEEL_BASE_CORR_INIT_MM    20.0f     /* 修正值（标定旋钮） */

/* 弧线转弯半径（赛道地图设定，200mm 不可改） */
#define TURN_RADIUS_MM              200.0f

/* 弧线转车速（mm/周期）= 450 脉冲/周期过弯标准移速 */
#define V_TURN_MM_PER_CYCLE         1.01f

/* 到位判据 */
#define ARC_TOLERANCE_DEG           2.0f
#define ARC_STABLE_CNT              6       /* 连续稳定周期（原10→6，减判位延迟） */

/* 临近目标减速：误差 < 阈值时按比例缩开环角速度，治过冲 */
#define ARC_SLOWDOWN_DEG            20.0f   /* 减速起始角度（度） */
#define ARC_SLOWDOWN_MIN            0.3f    /* 最小角速度比例（防停死） */

/* 弧线转 PWM 上限/最小死区（速度环/预留，外环已改用 ARC_CORR_MAX） */
#define ARC_PWM_MAX                 400
#define ARC_PWM_MIN                 150

/* 弧线转角度环 PID（外环，输出角速度修正 w_corr，rad/周期，弱纠偏） */
#define ARC_KP                      0.0003f  /* rad/度 */
#define ARC_KI                      0.0f
#define ARC_KD                      0.0f
#define ARC_CORR_MAX                0.003f   /* rad/周期，≈w_nominal */
#define ARC_INTEGRAL_MAX            300.0f

/* 弧线转左右轮速度环（转弯专用，允许负输出差速） */
#define ARC_SPEED_KP                2.0f
#define ARC_SPEED_KI                0.5f
#define ARC_SPEED_KD                0.0f
#define ARC_SPEED_OUT_MAX           999.0f

/* 侧滑补偿：内侧履带承侧向刮地阻力（恒负载），欠速致弧半径内缩。
 * 对内侧轮（|目标速度|较小侧）加固定 PWM。内缩→加大，外飘→减小。 */
#define ARC_SIDE_SLIP_COMP          150

/* 弧线转反向刹车 PWM（差速反打刹角速度，独立于直行 BRAKE_PWM） */
#define ARC_BRAKE_PWM              260

/* dir=+1左转 / -1右转，deg=转角 */
void Arc_Start(int8_t dir, float deg);
void Arc_Update(void);
uint8_t Arc_IsActive(void);

/* ==================== 7. 逆运动学入口 ==================== */

/* 车体中心线速度 v(mm/周期) + 角速度 w(rad/周期) → 左右轮目标速度 → 转弯差分速度环。
 * w 只在此处与轮距相乘（唯一度/长度交汇点），且只用于弧度。 */
void Car_SetTarget(float v_mm_per_cycle, float w_rad_per_cycle);

/* ==================== 初始化 & 编码器累计 ==================== */

void Car_Control_Init(void);
void Encoder_Total_Counter(int32_t enc_l, int32_t enc_r);

#endif
