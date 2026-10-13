/**
 * @file    arm_kinematics.h
 * @brief   机械臂正逆解（FK/IK）纯算法层，不依赖硬件
 *
 * 构型（定稿）：
 *   θ1 底座（水平回转）+ θ2 大臂（俯仰）+ θ3 小臂（俯仰）+ θ4 腕（俯仰）
 *   θ2/θ3/θ4 共面，θ1 水平解耦。
 *
 * 坐标系（基座系，右手）：
 *   +X 前方，+Y 左方，+Z 垂直向上。
 *   θ1 绕 Z 轴旋转（水平方位角 atan2(y,x)）。
 *   θ2/θ3 在 θ1 旋转后的垂直面内摆动，决定 (r, z)。
 *
 * 单位约定：
 *   角度一律用「度」。位置/长度一律「mm」。
 *   内部计算转弧度，对外接口用度。
 *
 * 依赖：arm_config.h
 */

#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#include "arm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 关节角结构：θ1~θ4（单位：度）
 * ============================================================ */
typedef struct {
    float th1;   /* 底座水平方位角（°），atan2(y,x) */
    float th2;   /* 大臂抬升角（°） */
    float th3;   /* 小臂角（°），相对大臂 */
    float th4;   /* 腕角（°），相对小臂 */
} arm_joint_t;

/* ============================================================
 * 基座系末端位姿（当前只解位置，不含姿态）
 * ============================================================ */
typedef struct {
    float x;   /* mm */
    float y;   /* mm */
    float z;   /* mm，相对 θ1 回转面（未加 H0，或加 H0 见约定） */
} arm_pos_t;

/* ---- FK：关节角 → 末端位置 ---- */

/**
 * @brief 正解：由 θ1~θ4 求末端（θ4 轴心）在基座系的位置
 * @param  j   输入的关节角（度）。th4 不参与位置求解，但保留参数。
 * @param  out 输出末端位置（mm），z 相对底座回转面（未含 H0）。
 * @note  当前 L4=0（夹爪 TCP 暂缓），故末端 = θ4 轴心点。
 *        若要夹爪 TCP，把 ARM_L4_MM 填实测值，末端沿 θ4 轴线再延伸 L4。
 */
void arm_fk(const arm_joint_t *j, arm_pos_t *out);

/* ---- IK：末端位置 → 关节角（解析法） ---- */

/**
 * @brief 逆解：由基座系目标点求 θ1~θ4（解析法，共面构型）
 * @param  p        目标位置（mm），z 相对底座回转面。
 * @param  elbow_up 肘构型选择：1=肘上（小臂上翻），0=肘下（小臂下垂，默认抓取）。
 * @param  out      输出关节角（度）。th4 固定为下翻极限（跟随，不参与求解）。
 * @retval 0=成功，-1=目标超出工作空间（不可达）。
 * @note   θ1 = atan2(y, x)；降维到 (r, z) 后对 θ2/θ3 用余弦定理解。
 *         多解按 elbow_up 选择；θ4 按 ARM_TH4_FIXED 策略固定。
 */
int arm_ik(const arm_pos_t *p, uint8_t elbow_up, arm_joint_t *out);

/* ---- 增量逆解（视觉伺服，数值雅可比） ---- */

/**
 * @brief 增量逆解：由「当前末端位姿 + 期望位移」求关节增量（用于视觉伺服闭环）
 * @param  cur      当前末端位置（mm）
 * @param  dxyz     期望位移（mm，目标 - 当前）
 * @param  j_cur    当前关节角（度，作为迭代起点）
 * @param  out_dth  输出关节增量（度，θ1/θ2/θ3；θ4 不变）
 * @retval 迭代次数；<0 表示未收敛。
 * @note   3 关节 × 3 维位置，雅可比满秩，可用解析雅可比一步逼近。
 *         当前实现：解析雅可比求逆 + 一次牛顿步（可视精度叠代）。
 */
int arm_ik_delta(const arm_pos_t *cur, const arm_pos_t *dxyz,
                 const arm_joint_t *j_cur, arm_joint_t *out_dth);

/* ============================================================
 * 换算层：角度 ↔ PWM ↔ 脉冲
 * ============================================================ */

/**
 * @brief 大臂/小臂/腕（θ2/θ3/θ4）机械角 → 总线舵机 PWM
 * @param  joint  关节号（2/3/4）
 * @param  deg    机械角（度）
 * @retval PWM 值（钳位到该关节 MIN/MAX）
 * @note  内部按 arm_config 的方向符号与零位换算；零位角基准待实测后校准。
 */
int32_t arm_angle_to_pwm(uint8_t joint, float deg);

/**
 * @brief 总线舵机 PWM → 机械角
 * @param  joint  关节号（2/3/4）
 * @param  pwm    PWM 值
 * @retval 机械角（度）
 */
float arm_pwm_to_angle(uint8_t joint, uint16_t pwm);

/**
 * @brief θ1 底座角 → 步进脉冲
 * @param  deg  目标角度（°）
 * @retval 脉冲数（带符号，正=ARM_DIR_CW 方向）
 */
int32_t arm_theta1_to_pulse(float deg);

/* ============================================================
 * θ1 朝向 / 转向（相对定位，需调用方维护朝向状态）
 * ============================================================ */

/*
 * 因步进无绝对零点（无物理止点 + 编码器掉电不保存），θ1 只能相对定位。
 * MCU 需在软件里维护「当前朝向（度）」累加器 g_th1_heading_deg。
 * 方向约定（实测锁定）：
 *   ARM_TH1_DIR_CW (0x00) = 左转 = 逆时针  → heading +
 *   ARM_TH1_DIR_CCW(0x01) = 右转 = 顺时针  → heading -
 */

/* θ1 转向结果：反向时携带方向与脉冲数，供调用方下发 Arm_MoveRelative */
typedef struct {
    uint8_t  dir;      /* ARM_TH1_DIR_CW / ARM_TH1_DIR_CCW */
    uint32_t pulses;   /* 脉冲增量（正数） */
} arm_th1_turn_t;

/**
 * @brief 计算「从当前朝向转到目标相对角」的方向与脉冲数
 * @param  heading_deg  当前 θ1 朝向（调用方维护的累加器，度）
 * @param  target_deg   目标相对角（相对收起态基准，度）
 * @param  force_ccw    非 0 = 强制向右转（顺时针/ARM_DIR_CCW），忽略短路径优化
 *                       （排爆桶在车体右侧、防机械卡臂时置 1）
 * @param  out          输出：方向 + 脉冲数
 * @retval 0=成功（out 已填）；-1=已在目标（delta 过小）
 * @note   force_ccw=0 时自动短路径（|delta|>180 反向）；force_ccw!=0 时
 *         固定向右转整圈补角，不优化。
 */
int arm_theta1_turn(float heading_deg, float target_deg, uint8_t force_ccw,
                    arm_th1_turn_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_KINEMATICS_H */
