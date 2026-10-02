#include "odometer.h"
#include "Encoder.h"
#include "Car_control.h"

/* ============================================================
 * 里程计实现（增量累加 + int32_t + 双轮平均）
 *
 * 数据源：直接复用采样任务读-清得到的每周期增量 g_enc_l/g_enc_r
 *   —— 读-清已把 16 位 TIM 计数器回绕问题消掉（每次读后清零），
 *      且本身就是「差值法」的增量语义（不丢脉冲、无绝对值回绕）。
 * ============================================================ */

static float dist_total_mm = 0.0f;
static float v_L_mm = 0.0f;
static float v_R_mm = 0.0f;
static uint8_t first = 1;   /* 首周期只记录基准，不算增量 */

void Odom_Reset(void)
{
    dist_total_mm = 0.0f;
    v_L_mm = 0.0f;
    v_R_mm = 0.0f;
    first = 1;
}

void Odom_Sample(void)
{
    /* 首周期：读-清后的 g_enc_l/r 可能含初始化噪声，跳过 */
    if (first)
    {
        first = 0;
        return;
    }

    /* g_enc_l/r 是本周期(5ms)读-清的脉冲增量（带符号） */
    float dL_mm = (float)g_enc_l * MM_PER_PULSE;
    float dR_mm = (float)g_enc_r * MM_PER_PULSE;

    v_L_mm = dL_mm;
    v_R_mm = dR_mm;

    dist_total_mm += (dL_mm + dR_mm) / 2.0f;   /* 车体中心位移 = 左右平均 */
}

float Odom_GetDistanceMM(void) { return dist_total_mm; }
float Odom_GetVL(void)         { return v_L_mm; }
float Odom_GetVR(void)         { return v_R_mm; }
