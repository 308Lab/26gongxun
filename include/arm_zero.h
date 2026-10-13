/**
 * @file    arm_zero.h
 * @brief   机械臂零位标定：记录收起态 → 算出整体偏移 → 落盘
 *
 * 分层：
 *   · Servo.c        —— 【应用】偏移（下发与到位判据都自动带上）
 *   · arm_zero.c     —— 【产生 / 保存】偏移（本模块）
 *   · param_store.c  —— Flash 读写的纯机制
 *
 * 语义：偏移 = 实测收起态 PWM − arm_config.h 的 ARM_ZERO_REF_*。
 *       机械重装后把臂摆回收起态、按键记忆，全部姿态一起平移。
 *       未标定 / 记录损坏时偏移全 0 —— 行为与不启用本功能完全一致（安全退化）。
 */

#ifndef __ARM_ZERO_H
#define __ARM_ZERO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 上电初始化：从 Flash 载入零位偏移并发布到 Servo 层
 * @note  必须在任何任务下发舵机角度【之前】调用（见 StartMainTask）。
 *        无有效记录 → 偏移全 0。
 */
void ArmZero_Init(void);

/**
 * @brief 把传入的三路实测 PWM 记为「收起态零位」
 * @param raw_pwm 三路原始回读值，顺序 = P1/P2/P3（即 Servo_LastPos(1..3)）
 * @retval 0=成功已落盘；-1=某路回读无效(0)或入参为空；-2=偏移超出合理范围；-3=Flash 写失败
 * @note   落盘成功后才发布到 RAM —— 避免 Flash 写失败却已开始按新偏移动作。
 *         ★ 传入的必须是【原始】回读值（Servo_LastPos 的返回值），不是加了偏移的命令值。
 */
int ArmZero_Record(const uint16_t raw_pwm[3]);

/** 是否已标定过（Flash 里有有效记录）。给 UI 显示用。 */
uint8_t ArmZero_IsCalibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_ZERO_H */
