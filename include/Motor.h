#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "MotorConfig.h"

/* ==================== 函数声明 ==================== */

/* 初始化所有轮（启动 PWM） */
void Motor_Init(void);

/* 单轮控制：Speed 带符号，正=前进，负=后退（方向由 MotorConfig.h 的 REVERSE 宏决定） */
void Motor_A_Move(int32_t Speed);
void Motor_B_Move(int32_t Speed);
#if (MOTOR_WHEEL_COUNT)
void Motor_C_Move(int32_t Speed);
void Motor_D_Move(int32_t Speed);
#endif
/* 设置速度（在原始速度基础上） */
void SetWheelSpeed(int32_t left_speed, int32_t right_speed);

/* 整车停止 */
void Stop(void);

/* 原地转弯 */
void lefTurn(void);
void rightTurn(void);

#endif
