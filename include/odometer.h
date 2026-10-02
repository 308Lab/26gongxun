#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "main.h"

/* ============================================================
 * 里程计（采集层：唯一「脉冲 → mm」换算点）
 *
 * 差值法：不丢脉冲（消除原"读-清"空窗造成的左偏）。
 * 车体中心位移 = 左右轮位移的平均值。
 * ============================================================ */

/* 脉冲 → mm 换算系数（初值按节圆41mm：π*41/122880 ≈ 0.001048，待实测校准） */
#define MM_PER_PULSE   0.002244f   /* 实测标定：509显示≈545实际，系数再×1.07 */

void  Odom_Sample(void);        /* 每控制周期调一次，读编码器累计距离并换算 */
float Odom_GetDistanceMM(void); /* 累计距离 mm */
float Odom_GetVL(void);         /* 左轮速度 mm/周期 */
float Odom_GetVR(void);         /* 右轮速度 mm/周期 */
void  Odom_Reset(void);         /* 清零（切段/重启时用） */

#endif
