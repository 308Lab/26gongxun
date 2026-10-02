#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include "main.h"

/* ============================================================
 * 电机驱动配置头文件
 * 用途：集中管理「轮数」「每轮方向」「引脚映射」，改这里即可，不改 Motor.c
 *
 * 引脚说明：
 *   PORT/PIN 宏直接「引用」 main.h 里的 CubeMX 生成宏
 *   （AIN_1_GPIO_Port / AIN_1_Pin ...），这样 CubeMX 改引脚后，
 *   main.h 自动更新，这里自动跟随，Motor.c 完全不用动。
 *
 * 使用方式：
 *   MOTOR_WHEEL_COUNT  0=双轮，1=四轮
 *   MOTOR_x_REVERSE    0=不反转，1=反转（解决接线方向反）
 * ============================================================ */

/* ---------- 轮数配置 ---------- */
/* 0 = 双轮，1 = 四轮 */
#define MOTOR_WHEEL_COUNT   0

/* ---------- 每轮方向反转控制（0/1） ---------- */
/*
 * 当某轮接线导致「正转命令」实际反转时，置 1 即可软件层翻转方向。
 * 新板实测：Motor_A接左轮（A=0 前进）、Motor_B接右轮（B=1 前进）。
 */
#define MOTOR_A_REVERSE    0
#define MOTOR_B_REVERSE    1
#define MOTOR_C_REVERSE    0
#define MOTOR_D_REVERSE    0

/* ---------- 基础速度 ---------- */
#define BASE_SPEED         220


/* ---------- 编码器配置 ---------- */
/*
 * MOTOR_USE_ENCODER  0 = 不使用编码器（纯开环，只按给定速度驱动）
 *                     1 = 使用编码器（速度闭环反馈，需 CubeMX 配 TIM2/TIM4 为 Encoder 模式）
 */
#define MOTOR_USE_ENCODER   1

/* 编码器定时器（A轮=TIM1，B轮=TIM8），供编码器读取模块引用 */
#define ENCODER_TIM_MA      &htim1
#define ENCODER_TIM_MB      &htim8

/* 编码器引脚（引用 CubeMX 宏，记录用） */
#define ENC_A_1_PORT    Enc_A_1_GPIO_Port
#define ENC_A_1_PIN     Enc_A_1_Pin
#define ENC_A_2_PORT    Enc_A_2_GPIO_Port
#define ENC_A_2_PIN     Enc_A_2_Pin
#define ENC_B_1_PORT    Enc_B_1_GPIO_Port
#define ENC_B_1_PIN     Enc_B_1_Pin
#define ENC_B_2_PORT    Enc_B_2_GPIO_Port
#define ENC_B_2_PIN     Enc_B_2_Pin


/* ---------- 引脚映射（引用 CubeMX 宏） ---------- */

// 电机 A（PWM 引脚 PB0 = TIM3_CH3）
#define AIN_1_PORT    AIN_1_GPIO_Port
#define AIN_1_PIN     AIN_1_Pin
#define AIN_2_PORT    AIN_2_GPIO_Port
#define AIN_2_PIN     AIN_2_Pin
#define PWM_A         TIM_CHANNEL_3

// 电机 B（PWM 引脚 PB1 = TIM3_CH4）
#define BIN_1_PORT    BIN_1_GPIO_Port
#define BIN_1_PIN     BIN_1_Pin
#define BIN_2_PORT    BIN_2_GPIO_Port
#define BIN_2_PIN     BIN_2_Pin
#define PWM_B         TIM_CHANNEL_4

#if (MOTOR_WHEEL_COUNT)
// 电机 C
#define CIN_1_PORT    CIN_1_GPIO_Port
#define CIN_1_PIN     CIN_1_Pin
#define CIN_2_PORT    CIN_2_GPIO_Port
#define CIN_2_PIN     CIN_2_Pin
#define PWM_C         TIM_CHANNEL_3

// 电机 D
#define DIN_1_PORT    DIN_1_GPIO_Port
#define DIN_1_PIN     DIN_1_Pin
#define DIN_2_PORT    DIN_2_GPIO_Port
#define DIN_2_PIN     DIN_2_Pin
#define PWM_D         TIM_CHANNEL_4

#endif


#endif 
