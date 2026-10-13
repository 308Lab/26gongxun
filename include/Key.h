#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/*
 * 无阻塞矩阵键盘驱动（3×3 矩阵，行扫描）
 *
 * 硬件接线（2026-10-12 更新：Row_2 已由 PE8 换脚到 PB2）：
 *   行 Row_1(PE10)、Row_2(PB2)、Row_3(PE7)  → OUTPUT_PP（扫描驱动侧）
 *   列 Col_1(PE13)、Col_2(PE14)、Col_3(PE15) → INPUT + PULLUP（读取侧）
 *
 * 键位映射（第一行 = 7/8/9，第一列在左）：
 *         Col_1   Col_2   Col_3
 *   Row_1   7      8      9       (PE10)
 *   Row_2   4      5      6       (PB2)
 *   Row_3   1      2      3       (PE7)
 *
 * 逻辑功能键：
 *   KEY_START : 键 1（初始启动）
 *   KEY_DONE  : 键 2（完成任务）
 *   KEY_CALIB : 键 3（陀螺仪校准）
 *
 * 事件位：HOLD / DOWN / UP / SINGLE / DOUBLE / LONG / REPEAT
 * 消抖：Key_Tick 每 5ms 调用，内部 Count>=2 采样（约 10ms 一次）。
 */

/* 物理按键总数 = 3 行 × 3 列 */
#define KEY_ROWS				3
#define KEY_COLS				3
#define KEY_COUNT				(KEY_ROWS * KEY_COLS)

/* 逻辑功能键编号（对应矩阵上的物理键位号 row*3+col）
 * 分段测试方案：
 *   键1→去A  键2→去B  键3→去C
 *   键7→去D  键8→去终点
 *   键9→陀螺仪校准 */
#define KEY_TO_A     6	/* 键 1：Row_3 × Col_1 = 6 */
#define KEY_TO_B     7	/* 键 2：Row_3 × Col_2 = 7 */
#define KEY_TO_C     8	/* 键 3：Row_3 × Col_3 = 8 */
#define KEY_TO_D     0	/* 键 7：Row_1 × Col_1 = 0 */
#define KEY_TO_END   1	/* 键 8：Row_1 × Col_2 = 1 */
#define KEY_CALIB    2	/* 键 9：Row_1 × Col_3 = 2（校准从键3移到键9） */
#define KEY_GO_A     3	/* 键 4：Row_2 × Col_1 = 3（跑到 A 点） */
#define KEY_FWD      4	/* 键 5：Row_2 × Col_2 = 4（到 B 校准点） */
#define KEY_SLOW     5	/* 键 6：Row_2 × Col_3 = 5（到 B 减速点） */

/* 事件标志位 */
#define KEY_HOLD				0x01
#define KEY_DOWN				0x02
#define KEY_UP					0x04
#define KEY_SINGLE				0x08
#define KEY_DOUBLE				0x10
#define KEY_LONG				0x20
#define KEY_REPEAT				0x40

uint8_t Key_Check(uint8_t n, uint8_t Flag);
void Key_Tick(void);

#endif
