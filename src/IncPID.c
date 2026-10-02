#include "IncPID.h"

/**
 * @brief  增量式 PID 初始化
 * @param  pid      PID 结构体指针
 * @param  kp/ki/kd PID 系数
 * @param  out_max  输出上限（如 999）
 * @param  out_min  输出下限（如 -999）
 */
void IncPID_Init(IncPID_TypeDef *pid,
                 float kp, float ki, float kd,
                 int32_t out_max, int32_t out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target   = 0;
    pid->feedback = 0;

    pid->err      = 0;
    pid->err_last = 0;
    pid->err_prev = 0;

    pid->output   = 0;

    pid->out_max = out_max;
    pid->out_min = out_min;
}

/**
 * @brief  增量式 PID 计算
 * @param  pid       PID 结构体指针
 * @param  target    目标值
 * @param  feedback  反馈值
 * @retval 限幅后的累计输出（int32_t，直接喂给电机）
 */
int32_t IncPID_Calc(IncPID_TypeDef *pid, int32_t target, int32_t feedback)
{
    float d_out;   /* 本次增量 Δu */

    pid->target   = target;
    pid->feedback = feedback;

    pid->err = target - feedback;                 /* e(k) */

    /* 增量式标准公式 */
    d_out = pid->Kp * (pid->err - pid->err_last)
          + pid->Ki *  pid->err
          + pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);

    /* 累加增量 */
    pid->output += (int32_t)d_out;

    /* 硬限幅，防溢出/防电机疯转 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    /* 误差移位：为下一次准备 */
    pid->err_prev = pid->err_last;    /* e(k-2) ← e(k-1) */
    pid->err_last = pid->err;         /* e(k-1) ← e(k)   */

    return pid->output;
}

/**
 * @brief  PID 全部清零（丢线/重启/切换目标时调用）
 */
void IncPID_ClearAll(IncPID_TypeDef *pid)
{
    pid->target   = 0;
    pid->feedback = 0;
    pid->err      = 0;
    pid->err_last = 0;
    pid->err_prev = 0;
    pid->output   = 0;
}
