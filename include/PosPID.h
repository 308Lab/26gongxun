#ifndef __POSPID_H
#define __POSPID_H

#include "main.h"

/* ============================================================
 * 位置式 PID 控制器（浮点版）
 *
 * 公式：u(k) = Kp·e(k) + Ki·Σe(k) + Kd·(e(k) - e(k-1))
 *   e(k)   = target - feedback
 *   Σe     = 积分累加（带限幅 anti-windup，防积分饱和）
 *
 * 与增量式 IncPID 的区别：
 *   - IncPID 输出「增量 Δu」再累加，跨周期累积；
 *   - PosPID 直接输出「绝对 u」，物理意义直白（角度环里 u 就是 PWM）。
 *
 * 浮点版：target/feedback 用 float，适合「角度环」这类需要 <1 精度
 * 的场景（yaw 误差只有零点几度时，int 会截成 0 丢精度）。
 *
 * 典型用途：转弯角度环（陀螺仪 yaw 反馈），out_min 设负值以支持反打。
 * ============================================================ */

typedef struct
{
    float Kp;            /* 比例系数 */
    float Ki;            /* 积分系数 */
    float Kd;            /* 微分系数 */

    float target;        /* 目标值 */
    float feedback;      /* 反馈值 */

    float err;           /* e(k)   本次误差 */
    float err_last;      /* e(k-1) 上次误差 */

    float integral;      /* 积分累加 Σe(k) */

    float out_max;       /* 输出上限 */
    float out_min;       /* 输出下限 */

    float integral_max;  /* 积分限幅上限（anti-windup） */
    float integral_min;  /* 积分限幅下限 */

    float output;        /* 当前输出 u(k) */
} PosPID_TypeDef;

void    PosPID_Init(PosPID_TypeDef *pid,
                    float kp, float ki, float kd,
                    float out_max, float out_min,
                    float integral_max, float integral_min);

float   PosPID_Calc(PosPID_TypeDef *pid, float target, float feedback);

void    PosPID_ClearAll(PosPID_TypeDef *pid);

#endif /* __POSPID_H */
