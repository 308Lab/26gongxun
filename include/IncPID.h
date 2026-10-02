#ifndef __INCPID_H
#define __INCPID_H

#include "main.h"

/* ============================================================
 * 增量式 PID 控制器（精简版）
 *
 * 公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 *       u(k)  = u(k-1) + Δu(k)
 *
 * 特点：
 *   - 算的是"输出变化量 Δu"，再累加成最终输出
 *   - 只保留最近 3 个误差 err/err_last/err_prev + 上一拍输出 output
 *   - 输出 int32_t，直接喂电机（PWM 0~999），按目标范围限幅
 * ============================================================ */

/* ==================== PID 结构体 ==================== */

typedef struct
{
    float Kp;            /* 比例系数 */
    float Ki;            /* 积分系数 */
    float Kd;            /* 微分系数 */

    int32_t target;      /* 目标速度 */
    int32_t feedback;    /* 实际速度（本次反馈） */

    int32_t err;         /* e(k)   本次误差 */
    int32_t err_last;    /* e(k-1) 上次误差 */
    int32_t err_prev;    /* e(k-2) 上上次误差 */

    int32_t output;      /* 当前输出 u(k)，跨周期累加，必须保留 */

    int32_t out_max;     /* 输出上限 */
    int32_t out_min;     /* 输出下限 */
} IncPID_TypeDef;

/* ==================== 函数声明 ==================== */

void    IncPID_Init(IncPID_TypeDef *pid,
                    float kp, float ki, float kd,
                    int32_t out_max, int32_t out_min);

int32_t IncPID_Calc(IncPID_TypeDef *pid, int32_t target, int32_t feedback);

void    IncPID_ClearAll(IncPID_TypeDef *pid);

#endif /* __INCPID_H */
