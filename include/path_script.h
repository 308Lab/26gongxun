#ifndef __PATH_SCRIPT_H
#define __PATH_SCRIPT_H

#include <stdint.h>

/*
 * 路段表（到 A 点路径，新方案 v1）
 *
 * 新距离方案（不覆盖旧方案，独立成表）：
 *   直行350 → 右转90° → 直行350 → 左转90° → 直行625 → 左转90° → 直行350
 */

/* 段类型 */
#define PATH_STR     0    /* 直行：param = 距离 mm */
#define PATH_ARC     1    /* 弧线转：dir 用，param = 角度 deg */
#define PATH_TURN    2    /* 原地转弯：dir 用，param = 角度 deg */
#define PATH_CALIB   3    /* 停车校准：dir 忽略，param = 停车时长 ms（停车 + 校准陀螺仪 yaw） */
#define PATH_SCAN    4    /* 减速扫描：减速慢行通过目标区域，车侧摄像头观察物体顺序，param = 距离 mm */
#define PATH_BCK     5    /* 倒车：反向退到目标区域中间，param = 距离 mm */
#define PATH_TASK    6    /* 任务点：param = 任务索引（预留） */
#define PATH_END     7    /* 终点 */

typedef struct {
    uint8_t type;     /* 段类型：PATH_STR/PATH_ARC/PATH_TURN/PATH_CALIB/PATH_SCAN/PATH_BCK/PATH_TASK/PATH_END */
    int8_t  dir;      /* 弧线转方向：+1 左转 / -1 右转（直行/任务/校准/扫描/倒车忽略） */
    float   param;    /* 直行/扫描/倒车=距离mm / 弧线=角度deg / 校准=停车ms / 任务=索引 */
} PathSeg;

/* 到 A 点的路段表 */
extern const PathSeg g_path_to_A[];
extern const uint32_t   g_path_to_A_count;

/* 到 B 点的路段表（A 点任务完成 → B 点，可写：菜单可调第 4/6 段距离） */
extern PathSeg g_path_to_B[];
extern const uint32_t   g_path_to_B_count;

/* 到 C 点的路段表（B 点任务完成 → C 点） */
extern const PathSeg g_path_to_C[];
extern const uint32_t   g_path_to_C_count;

/* 到 D 点的路段表（C 点任务完成 → D 点） */
extern const PathSeg g_path_to_D[];
extern const uint32_t   g_path_to_D_count;

/* 到终点的路段表（D 点任务完成 → 终点） */
extern const PathSeg g_path_to_END[];
extern const uint32_t   g_path_to_END_count;

#endif /* __PATH_SCRIPT_H */
