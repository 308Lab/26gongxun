#include "PosPID.h"

/**
 * @brief  位置式 PID 初始化（浮点版）
 * @param  pid           PID 结构体指针
 * @param  kp/ki/kd      PID 系数
 * @param  out_max       输出上限（如 999）
 * @param  out_min       输出下限（如 -999，转弯反打需允许负值）
 * @param  integral_max  积分限幅上限（anti-windup）
 * @param  integral_min  积分限幅下限
 */
void PosPID_Init(PosPID_TypeDef *pid,
                 float kp, float ki, float kd,
                 float out_max, float out_min,
                 float integral_max, float integral_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target   = 0.0f;
    pid->feedback = 0.0f;

    pid->err      = 0.0f;
    pid->err_last = 0.0f;

    pid->integral = 0.0f;

    pid->out_max = out_max;
    pid->out_min = out_min;

    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->output = 0.0f;
}

/**
 * @brief  位置式 PID 计算（浮点版）
 * @param  pid       PID 结构体指针
 * @param  target    目标值
 * @param  feedback  反馈值
 * @retval 限幅后的绝对输出（float，直接喂 PWM，带符号）
 *
 * 公式：u(k) = Kp·e(k) + Ki·Σe(k) + Kd·(e(k)-e(k-1))
 */
float PosPID_Calc(PosPID_TypeDef *pid, float target, float feedback)
{
    float p_term, i_term, d_term;
    float u;

    pid->target   = target;
    pid->feedback = feedback;

    pid->err = target - feedback;                 /* e(k) */

    /* 比例项 */
    p_term = pid->Kp * pid->err;

    /* 积分项：累加 + 限幅（anti-windup，防积分饱和导致超调/震荡） */
    pid->integral += pid->err;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    i_term = pid->Ki * pid->integral;

    /* 微分项：误差的差分 */
    d_term = pid->Kd * (pid->err - pid->err_last);

    u = p_term + i_term + d_term;

    /* 硬限幅输出 */
    if (u > pid->out_max) u = pid->out_max;
    if (u < pid->out_min) u = pid->out_min;

    pid->output = u;

    /* 误差移位，为下一次准备 */
    pid->err_last = pid->err;

    return pid->output;
}

/**
 * @brief  PID 全部清零（丢线/重启/切换目标时调用）
 */
void PosPID_ClearAll(PosPID_TypeDef *pid)
{
    pid->target   = 0.0f;
    pid->feedback = 0.0f;
    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output   = 0.0f;
}
