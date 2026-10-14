#include "path_script.h"

/*
 * 到 A 点路段表（新方案 v1）
 *   直行 → 右转90° → 直行 → 左转90° → 直行 → 左转90° → 直行
 *
 * 弧线转方向约定（与 Car_control 中 Arc_Start 一致）：
 *   dir=-1 → 右转（w<0，左轮外侧快）
 *   dir=+1 → 左转（w>0，右轮外侧快）
 */
/*
 * 【原 A 点路径（已注释保留，供参考直行距离）】
 *   直行420 → 右转90 → 直行180 → 左转90 → 直行480 → 左转90 → 直行400
 */
#if 0
const PathSeg g_path_to_A[] = {
    { PATH_STR,  0, 420.0f },
    { PATH_ARC, -1,  90.0f },
    { PATH_STR,  0, 180.0f },
    { PATH_ARC, +1,  90.0f },
    { PATH_STR,  0, 480.0f },
    { PATH_ARC, +1,  90.0f },
    { PATH_STR,  0, 400.0f },
};
#endif

/*
 * 【新 A 点路径】（2026-10-14 修订）
 *   直行420 → 右转90° → 直行180 → 左转90° → 直行480 → 左转90° → 直行200
 *   （到 A 点扫描起点后停 —— 扫描不在这里跑！）
 *
 * ★ 扫描段【已移入任务 A】（task_def.c 的 ACT_CAR_SCAN）：要求"机械臂转向在扫描之前完成"，
 *   而路径里的扫描总在任务点【之前】跑、任务在路径【之后】才启动 —— 顺序天然冲突。
 *   故路径只负责把车开到扫描起点，扫描交给任务、排在转向动作之后。
 *
 * 方向约定：dir=-1 右转，dir=+1 左转（见文件头 / Arc_Start）。
 */
const PathSeg g_path_to_A[] = {
    { PATH_STR,   0, 420.0f },   /* 1 直行 */
    { PATH_ARC,  -1,  90.0f },   /* 2 右转 90° */
    { PATH_STR,   0, 485.0f },   /* 3 直行 */
    { PATH_ARC,  +1,  90.0f },   /* 4 左转 90° */
    { PATH_STR,   0, 480.0f },   /* 5 直行 */
    { PATH_ARC,  +1,  90.0f },   /* 6 左转 90° */
    { PATH_STR,   0, 200.0f },   /* 7 直行（到达 A 扫描起点，停车待任务） */
};

const uint32_t g_path_to_A_count = sizeof(g_path_to_A) / sizeof(g_path_to_A[0]);

/*
 * 【原 B 点路径（已注释保留，供参考直行距离）】
 *   直行1350 → 右转90 → 直行530 → 右转90 → 直行450
 */
#if 0
const PathSeg g_path_to_B[] = {
    { PATH_STR,  0, 1350.0f },
    { PATH_ARC, -1,  90.0f },
    { PATH_STR,  0,  530.0f },
    { PATH_ARC, -1,  90.0f },
    { PATH_STR,  0,  450.0f },
};
#endif

/*
 * 【新 B 点路径】
 *   原地左转 → 直行 → 右转(非原地) → 直行 → 左转(非原地) → 直行
 *   → 右转(非原地) → 直行 → 右转(非原地) → 直行
 *
 * 段类型：PATH_TURN=原地转弯，PATH_ARC=弧线转（非原地）。
 * 距离均待定（参考原 1350/530/450）。
 */
PathSeg g_path_to_B[] = {
    { PATH_TURN, +1,  90.0f },  /* 1 原地左转 90° */
    { PATH_STR,   0, 485.0f },  /* 2 直行（距离待定） */
    { PATH_ARC,  -1,  90.0f },  /* 3 右转（非原地） */
    { PATH_STR,   0, 720.0f },  /* 4 直行（距离待定） */
    { PATH_CALIB, 0, 500.0f },  /* 5 停车 0.5s + 校准陀螺仪 yaw（过减速带后重标） */
    { PATH_TURN, +1,  90.0f },  /* 6 原地左转 90° */
    { PATH_STR,   0, 600.0f },  /* 7 直行（校准后左转第一个直线） */
    { PATH_ARC,  -1,  90.0f },  /* 8 右转（非原地） */
    { PATH_STR,   0, 555.0f },  /* 9 直行（距离待定） */
    { PATH_ARC,  -1,  90.0f },  /* 10 右转（非原地） */
    { PATH_STR,   0, 170.0f },  /* 11 直行（到达 B 减速点，距离待定） */
    { PATH_SCAN,  0, 500.0f },  /* 12 减速扫描：慢速通过 B 目标区（车侧摄像头观察顺序） */
    { PATH_BCK,   0, 250.0f },  /* 13 倒回 B 目标区中间（三物体中间，写死） */
};

const uint32_t g_path_to_B_count = sizeof(g_path_to_B) / sizeof(g_path_to_B[0]);

/*
 * 到 C 点路段表（B 点任务完成 → C 点，初稿文档数据）
 *   直行1730（中间无转弯）
 *
 * 弧线转方向约定同上：dir=-1 右转，dir=+1 左转。
 */
const PathSeg g_path_to_C[] = {
    { PATH_STR,  0, 1460.0f },   /* 1 直行（到达 C 减速点） */
    { PATH_SCAN, 0,  500.0f },   /* 2 减速扫描：慢速通过 C 目标区（车侧摄像头观察顺序） */
    { PATH_BCK,  0,  250.0f },   /* 3 倒回 C 目标区中间（写死） */
};

const uint32_t g_path_to_C_count = sizeof(g_path_to_C) / sizeof(g_path_to_C[0]);

/*
 * 到 D 点路段表（C 点任务完成 → D 点，初稿文档数据）
 *   直行425 → 右转90° → 直行830
 *
 * 弧线转方向约定同上：dir=-1 右转，dir=+1 左转。
 */
const PathSeg g_path_to_D[] = {
    { PATH_STR,  0, 420.0f },    /* 1 直行 */
    { PATH_ARC, -1,  90.0f },    /* 2 右转 90° */
    { PATH_STR,  0, 480.0f },    /* 3 直行（到达 D 减速点） */
    { PATH_SCAN, 0, 400.0f },    /* 4 减速扫描：慢速通过 D 目标区（车侧摄像头观察顺序） */
    { PATH_BCK,  0, 200.0f },    /* 5 倒回 D 目标区中间（写死） */
};

const uint32_t g_path_to_D_count = sizeof(g_path_to_D) / sizeof(g_path_to_D[0]);

/*
 * 到终点路段表（D 点任务完成 → 终点，初稿文档数据）
 *   直行1600
 */
const PathSeg g_path_to_END[] = {
    { PATH_STR,  0, 1655.0f },   /* 1 直行（到达终点） */
};

const uint32_t g_path_to_END_count = sizeof(g_path_to_END) / sizeof(g_path_to_END[0]);
