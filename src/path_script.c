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
 * 【新 A 点路径】一段直行
 *   直行（距离待定，参考原方案的 420/180/480/400）
 */
const PathSeg g_path_to_A[] = {
    { PATH_STR,  0, 615.0f },   /* 1 直行（到达 A 点，距离待定） */
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
const PathSeg g_path_to_B[] = {
    { PATH_TURN, +1,  90.0f },  /* 1 原地左转 90° */
    { PATH_STR,   0, 450.0f },  /* 2 直行（距离待定） */
    { PATH_ARC,  -1,  90.0f },  /* 3 右转（非原地） */
    { PATH_STR,   0, 530.0f },  /* 4 直行（距离待定） */
    { PATH_ARC,  +1,  90.0f },  /* 5 左转（非原地） */
    { PATH_STR,   0, 400.0f },  /* 6 直行（距离待定） */
    { PATH_ARC,  -1,  90.0f },  /* 7 右转（非原地） */
    { PATH_STR,   0, 520.0f },  /* 8 直行（距离待定） */
    { PATH_ARC,  -1,  90.0f },  /* 9 右转（非原地） */
    { PATH_STR,   0, 380.0f },  /* 10 直行（到达 B 点，距离待定） */
};

const uint32_t g_path_to_B_count = sizeof(g_path_to_B) / sizeof(g_path_to_B[0]);

/*
 * 到 C 点路段表（B 点任务完成 → C 点，初稿文档数据）
 *   直行1730（中间无转弯）
 *
 * 弧线转方向约定同上：dir=-1 右转，dir=+1 左转。
 */
const PathSeg g_path_to_C[] = {
    { PATH_STR,  0, 1700.0f },   /* 1 直行（到达 C 点） */
};

const uint32_t g_path_to_C_count = sizeof(g_path_to_C) / sizeof(g_path_to_C[0]);

/*
 * 到 D 点路段表（C 点任务完成 → D 点，初稿文档数据）
 *   直行425 → 右转90° → 直行830
 *
 * 弧线转方向约定同上：dir=-1 右转，dir=+1 左转。
 */
const PathSeg g_path_to_D[] = {
    { PATH_STR,  0, 400.0f },    /* 1 直行 */
    { PATH_ARC, -1,  90.0f },    /* 2 右转 90° */
    { PATH_STR,  0, 690.0f },    /* 3 直行（到达 D 点） */
};

const uint32_t g_path_to_D_count = sizeof(g_path_to_D) / sizeof(g_path_to_D[0]);

/*
 * 到终点路段表（D 点任务完成 → 终点，初稿文档数据）
 *   直行1600
 */
const PathSeg g_path_to_END[] = {
    { PATH_STR,  0, 1625.0f },   /* 1 直行（到达终点） */
};

const uint32_t g_path_to_END_count = sizeof(g_path_to_END) / sizeof(g_path_to_END[0]);
