#include "path_runner.h"
#include "path_script.h"
#include "Car_control.h"

/*
 * 路径决策层（AAATask 跑）。
 *
 * 每 5ms tick：
 *   根据当前段类型，向 RunTask 发对应命令（带 seq 自增），
 *   并通过 Go_IsActive()/Arc_IsActive() 的 0→1→0 变化判断「一段完成」。
 *
 * 任务链（通用化，5 个目标点）：
 *   IDLE --左键--> 去A --> TASK(A) --右键--> 去B --> TASK(B) --右键-->
 *   去C --> TASK(C) --右键--> 去D --> TASK(D) --右键--> 去终点 --> DONE
 *
 * 每个目标点到位后自动 Car_YawCalibrate() 归零，后续以该点朝向为航向基准。
 * 终点到达后置 g_end_reached=1，由 DDDTask 检测并执行终点动作。
 */

volatile RunnerCmd  g_cmd        = { CMD_NONE, 0, 0.0f, 0.0f, 0 };
volatile RunState   g_run_state  = RUN_ST_IDLE;
volatile uint8_t    g_target_idx = 0;      /* 0=A 1=B 2=C 3=D 4=终点 */
volatile uint8_t    g_end_reached = 0;     /* 终点到达标志（DDDTask 消费） */

/* 当前任务完成标志位：处于 TASK 态时检测到置 1 即切去下一个目标点。由右键置 1。 */
static volatile uint8_t task_done = 0;

/* 目标点个数（A/B/C/D + 终点 = 5）*/
#define TARGET_COUNT  5

/* 各目标的路径表指针/长度（索引 0~4 对应 A/B/C/D/终点）*/
static const PathSeg *target_path[TARGET_COUNT];
static uint32_t       target_len[TARGET_COUNT];

/* 当前正在跑的路段表及其长度 / 索引 */
static const PathSeg *cur_path      = NULL;
static uint32_t       cur_path_len  = 0;
static uint32_t       cur_seg_idx   = 0;

/* 段内子状态：协调「发命令 → 等 active 变 1 → 等 active 变 0」 */
typedef enum {
    SEG_WAIT_SEND = 0,   /* 该段还没发命令（切入瞬间发一次） */
    SEG_WAIT_ACTIVE,     /* 命令已发，等执行层把 active 拉 1 */
    SEG_WAIT_DONE,       /* active 已 1，等它归 0 = 该段完成 */
} SegPhase;

static SegPhase seg_phase = SEG_WAIT_SEND;
static uint16_t cmd_seq = 0;

/* 累计理论航向（相对当前 yaw 基准的度）。跑一条路径前清零，
 * 每遇到一段 PATH_ARC 就累加 dir*deg（左转+90/右转-90），
 * 直行段用它作航向目标。这样转弯过冲不会被下一段直行锁死。 */
static float heading_ref = 0.0f;

/* PATH_CALIB 段停车计时（ms，每 5ms tick 累加一次） */
static uint32_t calib_waited_ms = 0;

/* 【临时诊断】段完成时的 yaw 快照：记录「刚完成的段号 + 该段结束 yaw(×10)」。
 * 供 OLED 冻结显示「每段结束瞬间的车头朝向」，避免盯实时滚动数字。 */
volatile int32_t g_dbg_seg_idx = -1;      /* 刚完成的段号（0-based） */
volatile int32_t g_dbg_seg_yaw  = 0;       /* 该段结束时 yaw×10（带符号） */
volatile int32_t g_dbg_arc_dir  = 0;       /* 最近一次 ARC/TURN 的 dir（+1左/-1右） */

/* 发一条命令（seq 自增），并进入「等 active」子状态 */
static void IssueCmd(uint8_t cmd, int8_t dir, float param)
{
    g_cmd.cmd     = cmd;
    g_cmd.dir     = dir;
    g_cmd.param   = param;
    g_cmd.heading = heading_ref;   /* 直行时携带当前累计航向；弧线转忽略 */
    g_cmd.seq     = ++cmd_seq;
    seg_phase     = SEG_WAIT_ACTIVE;

    /* 【临时诊断】记录最近一次转弯的方向 */
    if (cmd == CMD_ARC || cmd == CMD_TURN)
    {
        g_dbg_arc_dir = (int32_t)dir;
    }
}

/* 切到下一段 */
static void SegNext(void)
{
    /* 【临时诊断】冻结「刚完成段」的 yaw 快照，供 OLED 显示 */
    g_dbg_seg_idx = (int32_t)cur_seg_idx;
    g_dbg_seg_yaw = (int32_t)(Car_GetYaw() * 10.0f);
    cur_seg_idx++;
    seg_phase = SEG_WAIT_SEND;
}

/* 切换当前路段表 + 复位索引 / 子相位。
 * reset_heading=1 同时清零累计航向（跑一条全新路径用）；
 * reset_heading=0 保留累计航向（从中间段续跑、接着上一段的方向走，如 SCAN/BCK 尾段）。 */
static void SwitchPath(const PathSeg *path, uint32_t len, uint8_t reset_heading)
{
    cur_path     = path;
    cur_path_len = len;
    cur_seg_idx  = 0;
    seg_phase    = SEG_WAIT_SEND;
    cmd_seq      = 0;
    if (reset_heading)
    {
        heading_ref  = 0.0f;
    }
    g_cmd.cmd    = CMD_NONE;
    g_cmd.seq    = 0;
}

/* 初始化目标路径表数组（A/B/C/D/终点） */
static void PathTargetsInit(void)
{
    target_path[0] = g_path_to_A;   target_len[0] = g_path_to_A_count;
    target_path[1] = g_path_to_B;   target_len[1] = g_path_to_B_count;
    target_path[2] = g_path_to_C;   target_len[2] = g_path_to_C_count;
    target_path[3] = g_path_to_D;   target_len[3] = g_path_to_D_count;
    target_path[4] = g_path_to_END; target_len[4] = g_path_to_END_count;
}

/* 开始跑「去某个目标点」的路径（target_idx 已在调用前设好） */
static void StartMovingToTarget(void)
{
    task_done = 0;
    SwitchPath(target_path[g_target_idx], target_len[g_target_idx], 1);
    g_run_state = RUN_ST_MOVING;
}

void PathRunner_Start(void)
{
    PathTargetsInit();
    g_target_idx = 0;            /* 从 A 开始 */
    g_end_reached = 0;
    task_done = 0;
    StartMovingToTarget();
}

/* 分段测试：直达指定目标点跑对应路径（0=A 1=B 2=C 3=D 4=终点） */
void PathRunner_Goto(uint8_t target_idx)
{
    PathRunner_GotoSegs(target_idx, 0);   /* 0 = 跑完整路径 */
}

/* 分段测试（带截断）：seg_count=0 跑完整，否则跑前 seg_count 段 */
void PathRunner_GotoSegs(uint8_t target_idx, uint32_t seg_count)
{
    if (target_idx >= TARGET_COUNT)
    {
        return;
    }
    PathTargetsInit();
    g_target_idx  = target_idx;
    g_end_reached = 0;
    task_done = 0;

    /* 跑完整路径：走 SwitchPath(len)；截断：临时写短 cur_path_len。
     * 注意：cur_path_len 是「跑完就切 TASK/DONE」的边界，截断后同样走到
     * 段末即停（进 TASK 或 DONE），语义一致。 */
    SwitchPath(target_path[target_idx], target_len[target_idx], 1);
    if (seg_count > 0 && seg_count < target_len[target_idx])
    {
        cur_path_len = seg_count;
    }

    g_run_state = RUN_ST_MOVING;
}

/* 从指定段开始续跑（保留累计航向）：用于「键5 到位后，键6 接着跑 SCAN+BCK 尾段」。
 * start_seg 为 0-based 起始段号；跑完 cur_path_len 即停。
 * 注意：上一段路径跑完时 path_runner 已 Car_YawCalibrate() 把 yaw 归零，
 * 故此处 heading_ref 也必须归零（与新的 0° 基准一致），否则 SCAN 直行航向
 * 锁会用残留的旧理论航向（如 -90°）对抗已经校准归零的实际 yaw，导致车打转/倒退。 */
void PathRunner_GotoSegsFrom(uint8_t target_idx, uint32_t start_seg)
{
    if (target_idx >= TARGET_COUNT)
    {
        return;
    }
    PathTargetsInit();
    g_target_idx  = target_idx;
    g_end_reached = 0;
    task_done = 0;

    SwitchPath(target_path[target_idx], target_len[target_idx], 1);  /* 归零 heading_ref（与 yaw 校准一致） */
    if (start_seg < target_len[target_idx])
    {
        cur_seg_idx = start_seg;
    }

    g_run_state = RUN_ST_MOVING;
}

void PathRunner_TaskDone(void)
{
    task_done = 1;
}

void PathRunner_Tick(void)
{
    if (g_run_state == RUN_ST_IDLE || g_run_state == RUN_ST_DONE)
    {
        return;
    }

    /* ==== 任务阶段：等当前任务的完成标志位 ==== */
    if (g_run_state == RUN_ST_TASK)
    {
        if (task_done)
        {
            /* 当前任务完成，去下一个目标点 */
            g_target_idx++;
            if (g_target_idx < TARGET_COUNT)
            {
                /* 还有下一个点（B/C/D/终点都可继续） */
                StartMovingToTarget();
            }
            else
            {
                /* 不该发生（终点应在走完 END 段后进 DONE） */
                g_run_state = RUN_ST_DONE;
            }
        }
        return;
    }

    /* ==== 路径段执行（RUN_ST_MOVING） ==== */

    if (cur_seg_idx >= cur_path_len)
    {
        /* 当前表走完：停稳，先校准 yaw 归零，再决定下一步 */
        g_cmd.cmd = CMD_NONE;
        Car_YawCalibrate();          /* 到位自动校准（后续按钮手动微调代替视觉纠偏） */

        /* 若刚走的是终点（目标 4），到达即全部完成，通知 DDD 任务 */
        if (g_target_idx >= (TARGET_COUNT - 1))
        {
            g_end_reached = 1;       /* DDDTask 消费此标志执行终点动作 */
            g_run_state  = RUN_ST_DONE;
        }
        else
        {
            g_run_state = RUN_ST_TASK;   /* 到 A/B/C/D → 等任务完成 */
        }
        return;
    }

    const PathSeg *seg = &cur_path[cur_seg_idx];

    switch (seg->type)
    {
        case PATH_STR:
        {
            if (seg_phase == SEG_WAIT_SEND)
            {
                IssueCmd(CMD_GO, 0, seg->param);
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                if (Go_IsActive()) seg_phase = SEG_WAIT_DONE;
                return;
            }

            /* SEG_WAIT_DONE：直行到位（Go_IsActive 归 0） */
            if (!Go_IsActive()) SegNext();
            return;
        }

        case PATH_ARC:
        {
            if (seg_phase == SEG_WAIT_SEND)
            {
                /* 累加理论航向：左转(dir=+1)+deg，右转(dir=-1)-deg */
                heading_ref += (float)seg->dir * seg->param;
                IssueCmd(CMD_ARC, seg->dir, seg->param);
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                if (Arc_IsActive()) seg_phase = SEG_WAIT_DONE;
                return;
            }

            if (!Arc_IsActive()) SegNext();
            return;
        }

        case PATH_TURN:
        {
            /* 原地转弯（绕车体中心）：Turn_Start 收带符号相对角度，
             * 正=左转、负=右转，故 dir*param 转角度；同样更新理论航向 */
            if (seg_phase == SEG_WAIT_SEND)
            {
                heading_ref += (float)seg->dir * seg->param;
                IssueCmd(CMD_TURN, seg->dir, seg->param);
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                if (Turn_IsActive()) seg_phase = SEG_WAIT_DONE;
                return;
            }

            if (!Turn_IsActive()) SegNext();
            return;
        }

        case PATH_CALIB:
        {
            /* 停车校准（过减速带后重标 yaw）：
             * 停车 param(ms) → Car_YawCalibrate() 归零 → 切下一段。
             * 注意：校准只把「当前朝向记为 0°」，不改变车头朝向。
             * 停车时长用 5ms tick 计数累加（path_runner 每 5ms tick 一次）。 */
            if (seg_phase == SEG_WAIT_SEND)
            {
                /* 停止任何运动命令，确保车停稳 */
                g_cmd.cmd = CMD_NONE;
                Stop();
                calib_waited_ms = 0;
                seg_phase = SEG_WAIT_ACTIVE;   /* 复用 WAIT_ACTIVE 作「停车计时中」 */
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                calib_waited_ms += 5;   /* 每 tick 5ms */
                if (calib_waited_ms >= (uint32_t)seg->param)
                {
                    Car_YawCalibrate();   /* 校准 yaw 归零 */
                    heading_ref = 0.0f;   /* 同步清累计航向：CALIB 语义=当前朝向为新 0° 基准 */
                    SegNext();
                }
                return;
            }

            /* 不该走到 SEG_WAIT_DONE，兜底切段 */
            SegNext();
            return;
        }

        case PATH_SCAN:
        {
            /* 减速扫描段：慢速通过目标区域，车侧摄像头观察物体顺序，
             * 编码器全程记录实际位移（供后续 PATH_BCK 倒回参考）。
             * 复用「发命令 → 等 active 1 → 等 active 0」三段式。 */
            if (seg_phase == SEG_WAIT_SEND)
            {
                IssueCmd(CMD_SCAN, 0, seg->param);
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                if (Scan_IsActive()) seg_phase = SEG_WAIT_DONE;
                return;
            }

            if (!Scan_IsActive()) SegNext();
            return;
        }

        case PATH_BCK:
        {
            /* 倒车段：反向退到目标区域中间（写死距离）。 */
            if (seg_phase == SEG_WAIT_SEND)
            {
                IssueCmd(CMD_BCK, 0, seg->param);
                return;
            }

            if (seg_phase == SEG_WAIT_ACTIVE)
            {
                if (Back_IsActive()) seg_phase = SEG_WAIT_DONE;
                return;
            }

            if (!Back_IsActive()) SegNext();
            return;
        }

        default:
            /* PATH_TASK/PATH_END 暂不支持，跳过 */
            SegNext();
            return;
    }
}

uint8_t PathRunner_IsBusy(void)
{
    return (g_run_state != RUN_ST_IDLE && g_run_state != RUN_ST_DONE);
}
