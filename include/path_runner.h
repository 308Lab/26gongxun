#ifndef __PATH_RUNNER_H
#define __PATH_RUNNER_H

#include <stdint.h>

/*
 * path_runner —— 路径决策层（放 AAATask 里跑）
 *
 * 职责：
 *   1) 维护路段状态机：判断当前段类型 / 判到位 / 切段；
 *   2) 产出命令标志位 g_cmd（写），供 RunTask（执行层）消费；
 *   3) 到 A 点后进入「任务 A」阶段（当前占位）；
 *   4) 全部完成后回 IDLE，OLED 显示 arrived A。
 *
 * 关键约定：
 *   - AAATask 只「决策 + 写命令」，不直接调 Go_Start/Arc_Start，不碰 PWM；
 *   - RunTask 只「读命令 + 执行」，用 g_cmd.seq 识别新命令防重复；
 *   - 到位回读靠 Car_control 暴露的 Go_IsActive()/Arc_IsActive()（纯读、无副作用）。
 */

/* 命令类型：AAATask → RunTask */
#define CMD_NONE   0
#define CMD_GO     1    /* 直行：param = 距离 mm */
#define CMD_ARC    2    /* 弧线转：dir 用，param = 角度 deg */
#define CMD_TURN   3    /* 原地转弯：dir 用，param = 角度 deg */
#define CMD_SCAN   4    /* 减速扫描：param = 距离 mm（慢速通过 + 记录编码器） */
#define CMD_BCK    5    /* 倒车：param = 距离 mm（反向退到目标区中间） */

/* 命令标志位：AAATask 写，RunTask 读。
 * seq 每次下发新命令 +1，RunTask 据此判断「这条新命令还没执行过」。 */
typedef struct {
    uint8_t  cmd;       /* CMD_NONE / CMD_GO / CMD_ARC */
    int8_t   dir;       /* CMD_ARC 的方向 +1左/-1右 */
    float    param;     /* CMD_GO=距离mm / CMD_ARC=角度deg */
    float    heading;   /* CMD_GO=目标航向(度)；CMD_ARC 忽略 */
    uint16_t seq;       /* 命令序号，自增 */
} RunnerCmd;

extern volatile RunnerCmd g_cmd;

/* 【临时诊断】段完成时的 yaw 快照（OLED 冻结显示用） */
extern volatile int32_t g_dbg_seg_idx;   /* 刚完成的段号（0-based） */
extern volatile int32_t g_dbg_seg_yaw;    /* 该段结束时 yaw×10（带符号） */
extern volatile int32_t g_dbg_arc_dir;   /* 最近一次 ARC/TURN 的 dir（+1左/-1右） */

/* PathRunner 自身的运行模式（供外部/OLED 查询） */
typedef enum {
    RUN_ST_IDLE = 0,      /* 空闲（未启动） */
    RUN_ST_MOVING,        /* 正在跑「去某目标点」的路径段 */
    RUN_ST_TASK,          /* 到达某目标点，执行任务（等待完成标志位） */
    RUN_ST_DONE,          /* 终点到达，全部完成 */
} RunState;

extern volatile RunState g_run_state;

/* 当前目标点索引：0=A, 1=B, 2=C, 3=D, 4=终点（供 OLED/调试查询） */
extern volatile uint8_t g_target_idx;

/* 起点到达标志：path_runner 走完终点段后置 1，DDDTask 检测到后执行终点动作。 */
extern volatile uint8_t g_end_reached;

/* 启动整条任务链（从「去 A」路径第 0 段开始）。由触发源（左键）调用。 */
void PathRunner_Start(void);

/* 分段测试：直达指定目标点跑对应路径（0=A 1=B 2=C 3=D 4=终点）。 */
void PathRunner_Goto(uint8_t target_idx);

/* 分段测试（带截断）：跑指定目标点路径的前 seg_count 段。
 * seg_count=0 表示跑完整路径（等价 PathRunner_Goto）。
 * 例：PathRunner_GotoSegs(1, 11) = 到 B 减速点（跑完前 11 段即停，不含 SCAN/BCK 尾段）。 */
void PathRunner_GotoSegs(uint8_t target_idx, uint32_t seg_count);

/* 从指定段续跑（保留累计航向）：键4 到减速点后，键5 接着跑 SCAN+BCK 尾段。
 * start_seg 为 0-based 起始段号，跑到路径末尾即停。 */
void PathRunner_GotoSegsFrom(uint8_t target_idx, uint32_t start_seg);

/* 置「当前任务完成」标志位：当前处于哪个任务点（A/B/C/D），右键就把该点标记完成，
 * 触发去下一个目标点行进。 */
void PathRunner_TaskDone(void);

/* 每控制周期 tick 一次（放 AAATask 循环里）。 */
void PathRunner_Tick(void);

/* 查询是否正在跑（非 IDLE/DONE）。 */
uint8_t PathRunner_IsBusy(void);

#endif /* __PATH_RUNNER_H */
