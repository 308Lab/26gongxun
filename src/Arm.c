/**
 * @file    Arm.c
 * @brief   机械臂步进电机（Emm42_V5.0）控制层实现
 */

#include "Arm.h"
#include <string.h>

/* 读参数应答超时（ms），阻塞等待上限 */
#define ARM_TIMEOUT_MS      200

/* ZDT 官方库句柄 */
static ZDT_HandleTypeDef g_zdt_handle;

/* 最近一帧原始回包的快照（由解析函数收到帧时回填，供 OLED 读，避免竞态） */
static uint8_t g_last_frame[ZDT_MAX_RESP_SIZE];
static uint8_t g_last_frame_len = 0;

static void Arm_SaveLastFrame(const uint8_t *frame, uint8_t len);

/* ============================================================
 * 初始化：绑定 USART1
 * ============================================================ */
void Arm_Init(void)
{
    ZDT_EmmV5_Init(&g_zdt_handle, &huart1, ARM_ADDR);
}

/* ============================================================
 * 发一条「读参数」命令：[addr][cmd][0x6B]（绕开官方库错位映射）
 * ============================================================ */
static void Arm_SendReadCmd(uint8_t cmd)
{
    uint8_t frame[3];

    /* 发送前清空旧回包，避免读到上一次按键的残留帧 */
    ZDT_UART_ClearBuffer();

    frame[0] = ARM_ADDR;
    frame[1] = cmd;
    frame[2] = ZDT_FRAME_END;
    ZDT_UART_SendCmd(frame, 3);
}

/* ============================================================
 * 同步收发：发「读参数」命令，等应答，从环形缓冲区解析
 * ============================================================ */
int Arm_ReadParam(uint8_t cmd, uint8_t *out, uint8_t expect)
{
    uint32_t start;
    uint8_t b;
    uint8_t frame[ZDT_MAX_RESP_SIZE];
    uint8_t len = 0;

    /* 1. 清空旧数据 */
    ZDT_UART_ClearBuffer();

    /* 2. 发送读命令：地址 + 命令码 + 0x6B */
    ZDT_EmmV5_Read_Sys_Params(ARM_ADDR, (ZDT_SysParams_t)cmd);

    /* 3. 阻塞等应答：读到结束符 0x6B 或超时 */
    start = HAL_GetTick();
    len = 0;
    while (1)
    {
        /* 超时 */
        if ((HAL_GetTick() - start) > ARM_TIMEOUT_MS)
        {
            return -1;
        }

        /* 逐字节读环形缓冲区，遇到 0x6B 结束 */
        if (ZDT_UART_ReadByte(&b) == 0)
        {
            if (len < ZDT_MAX_RESP_SIZE)
            {
                frame[len++] = b;
            }
            if (b == ZDT_FRAME_END)  /* 0x6B */
            {
                break;
            }
        }
    }

    /* 4. 校验帧：至少 地址(1)+命令码(1)+...+结束符(1) */
    if (len < 4)
    {
        return -2;
    }

    /* 错误返回：地址 + 0x00 + 0xEE + 0x6B */
    if (frame[1] == 0x00 && frame[2] == ARM_STAT_ERR)
    {
        return -3;
    }

    /* 5. 校验命令码回显：读到的是这条命令的返回 */
    /*    （注意：读参数命令，驱动器回的是「命令码」，即我们发的 cmd） */
    if (frame[1] != cmd)
    {
        return -2;
    }

    /* 6. 数据区 = frame[2 .. len-2]（去掉 地址/命令码/结束符） */
    uint8_t data_len = len - 3;  /* 总长 - 地址 - 命令码 - 结束符 */
    if (data_len < expect)
    {
        return -2;
    }

    if (out != NULL)
    {
        for (uint8_t i = 0; i < expect; i++)
        {
            out[i] = frame[2 + i];
        }
    }

    return 0;
}

/* ============================================================
 * 读版本号：固件版本 + 硬件版本
 * 手册 §5.5.2 返回：Addr 1F FW_Ver(2B) HW_Series HW_Type HW_Ver 6B（账 8B）
 * 实机回包示例（本次上机确认）：01 1F 00 CC 23 14 6B （只有 7B！）
 *   → 实机缺 HW_Ver 独立字节，“23 14” 即 HW_Series + HW_Type 后直接接 6B。
 *
 * 结论：以【实机回包】为准，FW_Ver 为 2 字节大端，其后 1~2 字节为硬件信息。
 * 故此处按 7 字节（含结束符）解析；若未来板子回 8 字节也能兼容（多读 1 字节）。
 * ============================================================ */
int Arm_ReadVersion(Arm_Version_t *ver)
{
    uint8_t frame[ZDT_MAX_RESP_SIZE];
    uint8_t len = 0;
    int ret;

    if (ver == NULL)
    {
        return -1;
    }
    ver->ok = 0;
    ver->fw_ver = 0;
    ver->hw_ser  = 0;
    ver->hw_type = 0;
    ver->hw_ver  = 0;

    /* 发读版本命令 + 收一帧 */
    Arm_SendReadCmd(ARM_CMD_VER);

    ret = Arm_CollectFrame(frame, (uint8_t)sizeof(frame), &len);
    if (ret != 0)
    {
        return -1;   /* 本次未锁存到完整帧 */
    }

    /* 兼容 7~8 字节：地址 + 1F + FW(2) + HW信息(2~3) + 6B */
    if (len < 7 || len > 8)
    {
        return -2;
    }
    if (frame[0] != ARM_ADDR || frame[1] != ARM_CMD_VER)
    {
        return -2;
    }

    ver->fw_ver  = ((uint16_t)frame[2] << 8) | frame[3];
    ver->hw_ser  = frame[4];
    ver->hw_type = frame[5];
    /* 实机 7B：frame[6] 已是 6B；若 8B，frame[6] 是 HW_Ver，frame[7] 是 6B */
    ver->hw_ver = (len == 8) ? frame[6] : 0;
    ver->ok = 1;
    Arm_SaveLastFrame(frame, len);
    return 0;
}

/* ============================================================
 * 读「符号+数值」型参数（位置/误差/转速），手册 §5.5 格式解析
 * 帧格式：addr cmd 符号(00/01) 值(2B 或 4B 大端) 6B
 * ============================================================ */
int Arm_ReadValue(uint8_t cmd, Arm_Value_t *out)
{
    uint8_t frame[ZDT_MAX_RESP_SIZE];
    uint8_t len = 0;
    uint8_t val_bytes;
    int ret;

    if (out == NULL)
    {
        return -1;
    }
    out->neg = 0;
    out->val = 0;

    /* 数值区字节数：转速 VEL=2 字节，其余(位置/误差/目标位置)=4 字节 */
    val_bytes = (cmd == ARM_CMD_VEL) ? 2 : 4;

    /* 发读命令 + 收一帧 */
    Arm_SendReadCmd(cmd);

    ret = Arm_CollectFrame(frame, (uint8_t)sizeof(frame), &len);
    if (ret != 0)
    {
        return -1;
    }

    /* 期望长度 = 地址(1) + 命令(1) + 符号(1) + 值(val_bytes) + 结束符(1) */
    if (len != (uint8_t)(4 + val_bytes))
    {
        return -2;
    }
    if (frame[0] != ARM_ADDR || frame[1] != cmd)
    {
        return -2;
    }
    if (frame[3 + val_bytes] != ZDT_FRAME_END)
    {
        return -2;
    }

    /* 符号：00=正，01=负 */
    out->neg = (frame[2] != 0x00) ? 1 : 0;

    /* 数值：大端拼合 */
    out->val = 0;
    for (uint8_t i = 0; i < val_bytes; i++)
    {
        out->val = (out->val << 8) | frame[3 + i];
    }

    Arm_SaveLastFrame(frame, len);
    return 0;
}

/* ============================================================
 * 读电机状态标志（0x3A）：addr 3A 标志(1B) 6B
 * ============================================================ */
int Arm_ReadFlags(uint8_t *flags)
{
    uint8_t frame[ZDT_MAX_RESP_SIZE];
    uint8_t len = 0;
    int ret;

    if (flags == NULL)
    {
        return -1;
    }
    *flags = 0;

    /* 发读标志命令 + 收一帧 */
    Arm_SendReadCmd(ARM_CMD_FLAG);

    ret = Arm_CollectFrame(frame, (uint8_t)sizeof(frame), &len);
    if (ret != 0)
    {
        return -1;
    }

    /* 期望 5 字节：地址 + 3A + 标志 + 结束符 + （见手册 §5.5.15 返回 addr 3A 标志 6B） */
    if (len != 5)
    {
        return -2;
    }
    if (frame[0] != ARM_ADDR || frame[1] != ARM_CMD_FLAG)
    {
        return -2;
    }
    if (frame[4] != ZDT_FRAME_END)
    {
        return -2;
    }

    *flags = frame[3];
    Arm_SaveLastFrame(frame, len);
    return 0;
}

/* ============================================================
 * 符号+数值 → 带符号 32 位整数
 * ============================================================ */
int32_t Arm_ValueToInt32(const Arm_Value_t *v)
{
    if (v == NULL)
    {
        return 0;
    }
    int32_t s = (int32_t)(v->val & 0x7FFFFFFF);
    return (v->neg) ? -s : s;
}

/* ============================================================
 * Emm 量纲：点值(0-65535=一圈) → 角度(°)
 * ============================================================ */
float Arm_RawToDeg(uint32_t raw)
{
    return ((float)raw * 360.0f) / 65536.0f;
}

/* ============================================================
 * 非阻塞读取驱动器回包原始字节（调试验证回包链路用）
 * ============================================================ */
int Arm_ReadRawFrame(uint8_t *out, uint8_t max)
{
    uint8_t b;
    int n = 0;

    if (out == NULL || max == 0)
    {
        /* 只消费不保存：清空缓冲 */
        while (ZDT_UART_ReadByte(&b) == 0) { /* drain */ }
        return 0;
    }

    while (n < max && ZDT_UART_ReadByte(&b) == 0)
    {
        out[n++] = b;
    }
    return n;
}

/* ============================================================
 * 帧锁存接收：持续收字节，收到结束符 0x6B 才锁定一帧
 * 跨调用保留已收字节，防“一闪而过”；收到完整帧后返回 0。
 * ============================================================ */
int Arm_CollectFrame(uint8_t *out, uint8_t max, uint8_t *len)
{
    uint32_t start;
    uint8_t acc[ZDT_MAX_RESP_SIZE];
    uint8_t acc_len = 0;
    uint8_t collecting = 0;
    uint8_t b;

    if (out == NULL || len == NULL)
    {
        return -1;
    }

    /* 阻塞等待：在 ARM_TIMEOUT_MS 窗口内持续收字节，直到锁到 0x6B 才返回。
     * 否则第一次按键时回包尚未到达，立刻读会抢跑返回失败；
     * 第二次按键“恰好读到上一帧残留”的说法只是表象 —— 真正原因是没等回包。 */
    start = HAL_GetTick();
    while (1)
    {
        /* 每拍把环缓冲里所有可用字节搬入累积区 */
        while (ZDT_UART_ReadByte(&b) == 0)
        {
            if (!collecting)
            {
                /* 以帧首地址字节 0x01 才开始收，跳过回环杂声 */
                if (b == ARM_ADDR)
                {
                    collecting = 1;
                    acc_len = 0;
                }
                else
                {
                    continue;
                }
            }

            if (acc_len < max)
            {
                acc[acc_len++] = b;
            }
            else
            {
                /* 溢出：放弃本帧 */
                return -1;
            }

            if (b == ZDT_FRAME_END)   /* 0x6B */
            {
                memcpy(out, acc, acc_len);
                *len = acc_len;
                return 0;
            }
        }

        /* 超时未锁到完整帧 */
        if ((HAL_GetTick() - start) > ARM_TIMEOUT_MS)
        {
            /* 清环缓冲残留，避免下次读到半截旧帧 */
            while (ZDT_UART_ReadByte(&b) == 0) { /* drain */ }
            return -1;
        }
    }
}

/* ============================================================
 * 绝对位置运动：以手册 §5.3.12 为准手动组帧，显式发运动模式 0x01
 * 帧：Addr FD 方向 速度(2B) 加速度 脉冲数(4B) 运动模式(01) 同步 6B
 * ⚠️ 脉冲数量纲 = 3200/圈（细分16），非 65536/圈。
 * ============================================================ */
void Arm_MoveAbsolute(uint32_t pulses)
{
    uint8_t cmd[13];
    uint16_t vel = 60;         /* 默认 60 RPM，低速安全便于观察 */
    uint8_t  acc = 50;         /* 曲线加减速档位，带缓起缓停 */
    uint8_t  dir = ARM_DIR_CCW;  /* 绝对定位转向初向 */

    /* 发送前清空旧回包 */
    ZDT_UART_ClearBuffer();

    cmd[0]  = ARM_ADDR;
    cmd[1]  = ARM_MOVE_CMD;            /* 0xFD */
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel & 0xFF);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(pulses >> 24);
    cmd[7]  = (uint8_t)(pulses >> 16);
    cmd[8]  = (uint8_t)(pulses >> 8);
    cmd[9]  = (uint8_t)(pulses & 0xFF);
    cmd[10] = ARM_MOVE_ABS_ZERO;       /* 运动模式 01 = 绝对（相对坐标零点）*/
    cmd[11] = 0;                       /* 同步标志 0 = 立即执行 */
    cmd[12] = ZDT_FRAME_END;

    ZDT_UART_SendCmd(cmd, 13);
}

/* ============================================================
 * 相对位置运动：以当前实时位置为基准，转指定脉冲增量
 * 帧：Addr FD 方向(2B) 速度(2B) 加速度 脉冲数(4B) 运动模式(02) 同步 6B
 * 脉冲数量纲 = 3200/圈（细分16），90° = 800 脉冲。
 * ============================================================ */
void Arm_MoveRelative(uint32_t pulses, uint8_t dir)
{
    uint8_t cmd[13];
    uint16_t vel = 60;         /* 60 RPM */
    uint8_t  acc = 50;         /* 曲线加减速档位 */

    if (dir > ARM_DIR_CCW)
    {
        dir = ARM_DIR_CW;      /* 非法方向兜底到 CW */
    }

    ZDT_UART_ClearBuffer();

    cmd[0]  = ARM_ADDR;
    cmd[1]  = ARM_MOVE_CMD;            /* 0xFD */
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel & 0xFF);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(pulses >> 24);
    cmd[7]  = (uint8_t)(pulses >> 16);
    cmd[8]  = (uint8_t)(pulses >> 8);
    cmd[9]  = (uint8_t)(pulses & 0xFF);
    cmd[10] = ARM_MOVE_REL_CUR;        /* 运动模式 02 = 相对当前实时位置 */
    cmd[11] = 0;                       /* 同步标志 0 = 立即执行 */
    cmd[12] = ZDT_FRAME_END;

    ZDT_UART_SendCmd(cmd, 13);
}

/* ============================================================
 * 回填最近一帧快照（解析函数收到完帧后调用）
 * ============================================================ */
static void Arm_SaveLastFrame(const uint8_t *frame, uint8_t len)
{
    if (frame != NULL && len <= ZDT_MAX_RESP_SIZE)
    {
        memcpy(g_last_frame, frame, len);
        g_last_frame_len = len;
    }
}

/* ============================================================
 * 获取最近一帧原始回包快照（无副作用，供 OLED 显示）
 * ============================================================ */
int Arm_GetLastFrame(uint8_t *out, uint8_t max)
{
    uint8_t n;
    if (out == NULL || max == 0)
    {
        return 0;
    }
    n = (g_last_frame_len < max) ? g_last_frame_len : max;
    if (n > 0)
    {
        memcpy(out, g_last_frame, n);
    }
    return n;
}
