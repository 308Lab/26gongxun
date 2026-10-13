/**
 * @file    formal_mode.h
 * @brief   正式模式 Supervisor（全自动 A→B→C→D 任务状态机）
 *
 * 定位：
 *   正式模式 = path_runner（移动层 Supervisor，跑车到 A/B/C/D/终点）
 *             + 每个任务点的「任务动作状态机」（本模块）。
 *
 *   path_runner 到任务点后进入 RUN_ST_TASK 等 task_done；
 *   本模块在每个任务点驱动「任务动作序列」（视觉握手 + 机械臂 + 伺服闭环），
 *   序列末调 PathRunner_TaskDone() 让 path_runner 继续去下一个点。
 *
 * 运行节奏（全自动）：
 *   键 1 = 完整正式模式（A + B + C + D，一路跑完不停）
 *   键 2 = 到 A 完成扫码即停（终态停在 A）
 *   键 3 = A + B（终态停在 B 结束）
 *   键 4 = A + B + C（终态停在 C 结束）
 *   ——「停在最末任务结束」，不中途停下等确认。
 *
 * 视觉伺服闭环（对齐《机械臂运动与参数.md》§6.6，非雅可比）：
 *   每帧：FK(θ_now)→P_now → P_target = P_now + (dx,dy,dz)
 *        → arm_ik(P_target) 解析逆解 → arm_angle_to_pwm → 下发舵机。
 *   arm_ik_delta（数值雅可比）遗留/可删，不进主路径。
 *
 * 依赖：path_runner.h / Vision.h / arm_kinematics.h / arm_config.h / Arm.h / Servo.h
 */

#ifndef __FORMAL_MODE_H
#define __FORMAL_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 正式模式运行范围（键位选择：跑完整 vs 停在某个任务末）
 * ============================================================ */
typedef enum {
    FORMAL_END_AFTER_A = 0,   /* 键2：到 A 完成扫码即停 */
    FORMAL_END_AFTER_B = 1,   /* 键3：A + B */
    FORMAL_END_AFTER_C = 2,   /* 键4：A + B + C */
    FORMAL_END_AFTER_D = 3,   /* 键1：完整 A+B+C+D */
} formal_end_t;

/* ============================================================
 * 运行状态（供 OLED / 调试查询）
 * ============================================================ */
typedef enum {
    FORMAL_IDLE = 0,        /* 空闲 */
    FORMAL_MOVING,          /* path_runner 跑车中（去某任务点） */
    FORMAL_TASK,            /* 在任务点执行任务动作序列 */
    FORMAL_DONE,            /* 已跑到最末任务并完成 */
} formal_state_t;

extern volatile formal_state_t g_formal_state;

/* ============================================================
 * 接口
 * ============================================================ */

/**
 * @brief 启动正式模式，跑至指定终点任务
 * @param  end  终点范围（FORMAL_END_AFTER_A/B/C/D）
 * @note   内部调用 PathRunner_Start()（从 A 开始），并记录终点边界。
 */
void Formal_Start(formal_end_t end);

/**
 * @brief 每控制周期 tick（放 MainTask 循环里，紧跟 PathRunner_Tick）
 * @note   内部：
 *   - 观察 path_runner 的 g_run_state / g_target_idx：
 *     进入 RUN_ST_TASK 时启动该点任务动作序列；
 *   - 驱动任务动作序列状态机（视觉握手/机械臂/伺服闭环）。
 */
void Formal_Tick(void);

/**
 * @brief 查询是否正在跑正式模式
 * @retval 1=运行中（非 IDLE/DONE）
 */
uint8_t Formal_IsBusy(void);

/**
 * @brief 复位（清运行状态，回 IDLE；供急停/结束调用）
 */
void Formal_Abort(void);

/**
 * @brief 强制收起（供「再按一键手动收起」兕底）
 * @note   仅对 A 任务生效：若正处于 A 扫码等 0x11 阶段，则跳过等待直接走收起步。
 */
void Formal_ForceTaskRetract(void);

#ifdef __cplusplus
}
#endif

#endif /* __FORMAL_MODE_H */
