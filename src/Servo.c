#include "Servo.h"
#include <stdio.h>
#include <string.h>

/* ==================== 接收缓存 ==================== */
static uint8_t servo_rx_buf[SERVO_RX_BUF_SIZE];    /* 线性接收缓存 */
static volatile uint16_t servo_rx_head = 0;        /* 写入指针 */
static volatile uint16_t servo_rx_tail = 0;        /* 读取指针（主循环消费） */

/* 单字节接收缓存（HAL_UART_Receive_IT 用） */
static uint8_t s_rx_byte = 0;

/* 当前方向标志（半双工） */
static servo_dir_t s_dir = SERVO_DIR_RX;

/* 前置声明 */
static void parse_reply(char *frame, uint8_t len);

/* ==================== 微秒延时（DWT 周期计数器） ====================
 * 方向切换（见 Servo_SetDir）需要极短的稳定时间，不能只靠 GPIO 写寄存器
 * 就立刻发字节 —— 半双工收发器（SN74LVC1G126 之类）的使能/禁用有传播延迟，
 * 切换后立刻发，首字节会被削掉，表现为"那个舵机没收到指令、不动"。
 * 用 DWT 而不是空循环：F103 是 Cortex-M3 带 DWT，计时精确，也不怕 -O2 把循环优化掉。 */
static void servo_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t need;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* DWT 的总开关，必须先置 */
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    start = DWT->CYCCNT;
    need  = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < need) { }
}

/* ==================== 通信底层 ==================== */

void Servo_UartInit(void)
{
    /* 默认进入接收方向：TX 断开、RX 导通 */
    Servo_SetDir(SERVO_DIR_RX);
    /* 启动逐字节接收 */
    HAL_UART_Receive_IT(SERVO_UART, &s_rx_byte, 1);
}

/* 切换半双工收发方向：
 *   PC9(TX 开关) 高导通/低断开
 *   PA11(RX 开关) 高导通/低断开
 *   TX 方向 = PC9 高 + PA11 低；RX 方向 = PC9 低 + PA11 高 */
void Servo_SetDir(servo_dir_t dir)
{
    s_dir = dir;
    if (dir == SERVO_DIR_TX)
    {
        HAL_GPIO_WritePin(SERVO_RX_SW_PORT, SERVO_RX_SW_PIN, GPIO_PIN_RESET);  /* RX 断开 */
        HAL_GPIO_WritePin(SERVO_TX_SW_PORT, SERVO_TX_SW_PIN, GPIO_PIN_SET);    /* TX 导通 */
    }
    else
    {
        HAL_GPIO_WritePin(SERVO_TX_SW_PORT, SERVO_TX_SW_PIN, GPIO_PIN_RESET);  /* TX 断开 */
        HAL_GPIO_WritePin(SERVO_RX_SW_PORT, SERVO_RX_SW_PIN, GPIO_PIN_SET);    /* RX 导通 */
    }

    /* 通道切换稳定延时 —— 2026-10-14 恢复。
     * 原先这里注释说"测试无延时是否影响收发"，实际是有影响的：
     * 三个关节是连发三条指令，中间零等待地反复翻方向，收发器的使能/禁用
     * 有传播延迟，第二条、第三条的首字节会被削掉 —— 表现就是"某个关节不动"、
     * "多关节动作变成逐个动"。20µs 对 115200 下 18 字节（1.56ms）的发送可忽略。 */
    servo_delay_us(SERVO_DIR_SETTLE_US);
}

void Servo_SendString(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;

    /* 半双工：发送前切 TX 方向 */
    Servo_SetDir(SERVO_DIR_TX);
    HAL_UART_Transmit(SERVO_UART, (uint8_t *)str, len, 100);
    /* 发完回到 RX 方向以接收应答（隔离收发，无回显） */
    Servo_SetDir(SERVO_DIR_RX);
}

uint16_t Servo_Available(void)
{
    return (uint16_t)((servo_rx_head - servo_rx_tail) % SERVO_RX_BUF_SIZE);
}

uint16_t Servo_Read(uint8_t *buf, uint16_t len)
{
    uint16_t n = 0;
    while (n < len && servo_rx_tail != servo_rx_head) {
        buf[n++] = servo_rx_buf[servo_rx_tail];
        servo_rx_tail = (servo_rx_tail + 1) % SERVO_RX_BUF_SIZE;
    }
    return n;
}

/* 由 UART4 RxCplt 回调调用：写入缓存并续接接收 */
void Servo_RxCplt(uint8_t byte)
{
    uint16_t next = (uint16_t)((servo_rx_head + 1) % SERVO_RX_BUF_SIZE);
    if (next == servo_rx_tail) {
        /* 缓存满，丢弃最旧一个字节 */
        servo_rx_tail = (servo_rx_tail + 1) % SERVO_RX_BUF_SIZE;
    }
    servo_rx_buf[servo_rx_head] = byte;
    servo_rx_head = next;

    /* 续接逐字节接收 */
    HAL_UART_Receive_IT(SERVO_UART, &s_rx_byte, 1);
}

/* 中断回调入口：读自己的 s_rx_byte 并续接接收 */
void Servo_RxCpltISR(void)
{
    Servo_RxCplt(s_rx_byte);
}

/* ==================== 零位偏移（机械臂整体平移） ==================== */
/*
 * 为什么偏移加在【这一层】：
 *   全仓所有舵机角度下发最终都汇聚到下面的 Servo_SetAngle / Servo_SetAngleTime，
 *   到位判据则汇聚到 Servo_IsReached。在最底层加，调用点一行都不用改，也就不会漏 ——
 *   这正是"重装机械臂后无需重标 20 多个姿态"能成立的地方。
 *
 * ★ Servo_LastPos() 【不】加偏移：标定要靠它读舵机的真实物理位置。
 * ★ 标定自身 / 回读测试 / 诊断 —— 走 *_Raw 版本（绕过偏移，避免自我叠加）。
 *
 * 语义（2026-10-14 定）：标定时把机械臂摆到【收起态】，记下三路实测 PWM；
 *   偏移 = 记忆值 − arm_config 里的收起态基准（ARM_POSE_STOW_*）。
 *   机械装配偏了 → 偏移非零 → 所有姿态跟着一起平移。恒等时全 0，行为与不启用时一致。
 */
static int16_t s_zero_offset[SERVO_POS_CACHE_N];

/* 加偏移并钳到舵机行程内 */
static uint16_t apply_zero_offset(uint8_t id, uint16_t pwm)
{
    int32_t v;

    if (id >= SERVO_POS_CACHE_N) return pwm;   /* 超出缓存范围的 ID：不加偏移 */
    v = (int32_t)pwm + (int32_t)s_zero_offset[id];
    if (v > (int32_t)SERVO_PWM_MAX) v = (int32_t)SERVO_PWM_MAX;
    if (v < (int32_t)SERVO_PWM_MIN) v = (int32_t)SERVO_PWM_MIN;
    return (uint16_t)v;
}

void Servo_SetZeroOffset(uint8_t id, int16_t off)
{
    if (id < SERVO_POS_CACHE_N) s_zero_offset[id] = off;
}

int16_t Servo_GetZeroOffset(uint8_t id)
{
    return (id < SERVO_POS_CACHE_N) ? s_zero_offset[id] : (int16_t)0;
}

void Servo_ClearZeroOffset(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_POS_CACHE_N; i++) s_zero_offset[i] = (int16_t)0;
}

/* ==================== 三路臂舵机的力矩使能 ==================== */
/* 标定时要人工摆臂，而舵机通电时是抱死的、掰不动 —— 先失力。
 * ★ 状态只能软件跟踪：锁定状态在舵机自己内部，协议里没有查询命令。
 *   上电默认「有力」，与硬件一致（舵机上电即保持位置）。 */
static uint8_t s_arm_torque = 1u;

void Servo_SetArmTorque(uint8_t on)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++)
    {
        if (on != 0u)
        {
            Servo_Lock((uint8_t)(i + 1u));
        }
        else
        {
            Servo_Unlock((uint8_t)(i + 1u));
        }
    }
    s_arm_torque = (on != 0u) ? 1u : 0u;
}

uint8_t Servo_GetArmTorque(void)
{
    return s_arm_torque;
}

/* ==================== 指令封装 ==================== */

/* 控制位置：#<ID>P<PWM>!  （立即，【不含】零位偏移） */
void Servo_SetAngleRaw(uint8_t id, uint16_t pwm)
{
    char cmd[16];
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;
    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    snprintf(cmd, sizeof(cmd), "#%03uP%04u!", id, pwm);
    Servo_SendString(cmd);
}

/* 控制位置：#<ID>P<PWM>!  （立即，含零位偏移） */
void Servo_SetAngle(uint8_t id, uint16_t pwm)
{
    Servo_SetAngleRaw(id, apply_zero_offset(id, pwm));
}

/* 控制位置 + 时间：#<ID>P<PWM>T<TIME>!  （【不含】零位偏移） */
void Servo_SetAngleTimeRaw(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char cmd[24];
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;
    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (time_ms > SERVO_TIME_MAX) time_ms = SERVO_TIME_MAX;
    snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!", id, pwm, time_ms);
    Servo_SendString(cmd);
}

/* 控制位置 + 时间：#<ID>P<PWM>T<TIME>!  （含零位偏移） */
void Servo_SetAngleTime(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    Servo_SetAngleTimeRaw(id, apply_zero_offset(id, pwm), time_ms);
}

/* ==================== 多关节同步下发 ==================== */

/* 把一条 #IDPPWMTTIME! 追加到 body 尾部，返回新的写入位置；放不下则原样返回。
 * 返回 NULL 表示溢出 —— 调用方据此降级为逐条发送。 */
static char *append_move(char *p, char *end, uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char one[24];
    int  n;

    if (time_ms > SERVO_TIME_MAX) time_ms = SERVO_TIME_MAX;
    n = snprintf(one, sizeof(one), "#%03uP%04uT%04u!", id, pwm, time_ms);
    if ((n <= 0) || (p + n >= end))
    {
        return NULL;
    }
    (void)memcpy(p, one, (size_t)n);
    return p + n;
}

void Servo_SyncMove(uint8_t n, const uint8_t *ids, const uint16_t *pwms, uint16_t time_ms)
{
    char    body[72];
    char*   p   = body;
    char*   end = body + sizeof(body) - 2;   /* 留出结尾的 '}' 和 '\0' */
    char    cmd[80];
    uint8_t i;

    if ((n == 0u) || (n > 3u) || (ids == NULL) || (pwms == NULL))
    {
        return;
    }
    if (time_ms > SERVO_TIME_MAX) time_ms = SERVO_TIME_MAX;

    for (i = 0u; i < n; i++)
    {
        p = append_move(p, end, ids[i], apply_zero_offset(ids[i], pwms[i]), time_ms);
        if (p == NULL)
        {
            /* 溢出（正常不会发生：3×19 + 2 = 59 < 72）—— 降级为逐条，别丢动作 */
            for (i = 0u; i < n; i++)
            {
                Servo_SetAngleTime(ids[i], pwms[i], time_ms);
            }
            return;
        }
    }

    *p = '\0';
    /* 成组同步指令：{G0000<动作><动作>...}
     * ★ 只切【一次】收发方向 —— 逐条发要切 n 次（2n 次 GPIO 翻转），
     *   正是丢首字节的根源（见 Servo_SetDir）。
     * ★ 舵机在收到 '}' 后才同时启动，所以各路是真正同步的。 */
    (void)snprintf(cmd, sizeof(cmd), "{G0000%s}", body);
    Servo_SendString(cmd);
}

/* 请求回读位置：#<ID>PRAD! */
void Servo_ReadAngle(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPRAD!", id);
    Servo_SendString(cmd);
}

/* 请求回读温度+电压：#<ID>PRTV! */
void Servo_ReadTempVolt(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPRTV!", id);
    Servo_SendString(cmd);
}

/* 探测 ID 是否在线：#<ID>PID!  （有回应 #<ID>P! 表示在线） */
void Servo_ProbeId(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPID!", id);
    Servo_SendString(cmd);
}

/* 修改 ID：#<OLD>PID<NEW>! */
void Servo_ChangeId(uint8_t old_id, uint8_t new_id)
{
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "#%03uPID%03u!", old_id, new_id);
    Servo_SendString(cmd);
}

/* 释放扭力：#<ID>PULK! */
void Servo_Unlock(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPULK!", id);
    Servo_SendString(cmd);
}

/* 恢复扭力：#<ID>PULR! */
void Servo_Lock(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPULR!", id);
    Servo_SendString(cmd);
}

/* 设置最小值：#<ID>PSMI! （当前位设为最小值） */
void Servo_SetMin(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPSMI!", id);
    Servo_SendString(cmd);
}

/* 设置最大值：#<ID>PSMX! （当前位设为最大值） */
void Servo_SetMax(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPSMX!", id);
    Servo_SendString(cmd);
}

/* 设置启动初始位置：#<ID>PCSD! （当前位设为初始值，默认1500） */
void Servo_SetInitPos(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPCSD!", id);
    Servo_SendString(cmd);
}

/* 恢复初始值：#<ID>PCSR! （启动恢复力矩） */
void Servo_SetInitResume(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03uPCSR!", id);
    Servo_SendString(cmd);
}

/* 成组同步控制：body 形如 "#001P1400T3000!#002P800T3000!"，自动包成 {G0000..} */
void Servo_SendGroup(const char *body)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "{G0000%s}", body);
    Servo_SendString(cmd);
}

/* ==================== 回读解析 ==================== */

volatile servo_reply_t servo_last_reply;     /* 最近一次解析结果（跨任务共享，须 volatile） */

/* 按 ID 缓存的最近一次位置读回值（PWM）。
 *
 * 存在的理由：Servo_IsReached() 要回答"舵机 N 到目标了吗"，而 servo_last_reply 只装着
 * 【最后一次】解析的那一条 —— 三个舵机轮询时它一直在变，没法回答"某个特定 ID"。
 * 只缓存 1..SERVO_POS_CACHE_N-1 号（本工程只用到 1/2/3），不铺 256 字节。 */
static volatile uint16_t s_servo_pos[SERVO_POS_CACHE_N];

int Servo_IsReached(uint8_t id, uint16_t target_pwm, uint16_t tol)
{
    uint16_t cur;
    uint16_t tgt;
    int32_t  diff;

    if (id == 0 || id >= SERVO_POS_CACHE_N) return -1;  /* 不缓存这个 ID */
    cur = s_servo_pos[id];
    if (cur == 0) return -1;                            /* 还没读到过 —— 不能判"到位" */

    /* ★ 目标值必须加【同一个】零位偏移：`cur` 是舵机实际 PWM，而 target 是调用方从
     *   arm_config 传进来的原始值。下发时加了偏移、这里不加，两者就永远差着那个量 ——
     *   判据静默失效，而任务靠 timeout 照样能跑完，极难发现。 */
    tgt = apply_zero_offset(id, target_pwm);

    diff = (int32_t)cur - (int32_t)tgt;
    if (diff < 0) diff = -diff;
    return (diff <= (int32_t)tol) ? 1 : 0;
}

uint16_t Servo_LastPos(uint8_t id)
{
    return (id < SERVO_POS_CACHE_N) ? s_servo_pos[id] : 0;
}

/* 回包环形队列：解决「连发多条读命令时，单标志丢包」问题。
 * 视觉伺服/快速采样场景下，连续发 3 条 PRAD，回包可能在同一 Servo_Poll 周期内
 * 连续到达，单标志会被覆盖丢弃。用队列缓存多个解析结果。 */
#define SERVO_REPLY_QUEUE_SIZE   8
static servo_reply_t s_reply_queue[SERVO_REPLY_QUEUE_SIZE];
static volatile uint8_t s_reply_head = 0;   /* 写入位置（中断/IPC 侧） */
static volatile uint8_t s_reply_tail = 0;   /* 读取位置（主循环侧） */

static uint8_t s_reply_count(void)
{
    return (uint8_t)((s_reply_head - s_reply_tail) % SERVO_REPLY_QUEUE_SIZE);
}

/* 帧组装缓存：从 '#' 一直到 '!' */
static char s_frame[32];
static uint8_t s_frame_len = 0;
static uint8_t s_in_frame = 0;

/* 喂入一个回读字节，做帧边界切分 */
void Servo_Feed(uint8_t byte)
{
    if (byte == '#') {
        /* 帧开始 */
        s_frame_len = 0;
        s_frame[s_frame_len++] = (char)byte;
        s_in_frame = 1;
    } else if (s_in_frame) {
        if (s_frame_len < sizeof(s_frame) - 1) {
            s_frame[s_frame_len++] = (char)byte;
        }
        if (byte == '!') {
            /* 帧结束，解析 */
            s_frame[s_frame_len] = '\0';
            s_in_frame = 0;
            parse_reply(s_frame, s_frame_len);
            /* 入队（非阻塞；队满丢弃最旧一条，优先保留最新） */
            uint8_t next = (uint8_t)((s_reply_head + 1) % SERVO_REPLY_QUEUE_SIZE);
            if (next != s_reply_tail) {
                s_reply_queue[s_reply_head] = servo_last_reply;
                s_reply_head = next;
            }
        }
    }
}

/* 解析一帧应答，填充 servo_last_reply */
static void parse_reply(char *frame, uint8_t len)
{
    uint8_t i;
    (void)len;

    /* 默认清空 */
    servo_last_reply.kind = SERVO_REPLY_NONE;
    servo_last_reply.id = 0;
    servo_last_reply.pwm = 0;

    /* #OK! */
    if (frame[0] == '#' && frame[1] == 'O' && frame[2] == 'K') {
        servo_last_reply.kind = SERVO_REPLY_OK;
        return;
    }

    if (frame[0] != '#') {
        return;
    }

    /* 解析 ID：'#' 后跟 3 位数字 */
    if (frame[1] < '0' || frame[1] > '9') return;
    if (frame[2] < '0' || frame[2] > '9') return;
    if (frame[3] < '0' || frame[3] > '9') return;
    servo_last_reply.id = (uint8_t)((frame[1]-'0')*100 + (frame[2]-'0')*10 + (frame[3]-'0'));

    char *p = &frame[4];  /* 指向 'P' */

    /* 位置 #<ID>P<PWM>! */
    if (p[0] == 'P' && p[1] >= '0' && p[1] <= '9') {
        uint16_t val = 0;
        uint8_t n = 0;
        for (i = 1; p[i] >= '0' && p[i] <= '9'; i++) {
            val = val * 10 + (uint16_t)(p[i] - '0');
            n++;
            if (n >= 4) break;
        }
        servo_last_reply.kind = SERVO_REPLY_POSITION;
        servo_last_reply.pwm = val;
        /* 顺便更新按 ID 的位置缓存 —— Servo_IsReached() 查它。
         * 用 0 当"尚未读到过"的哨兵是安全的：PWM 有效范围是 500~2500。 */
        if (servo_last_reply.id < SERVO_POS_CACHE_N) {
            s_servo_pos[servo_last_reply.id] = val;
        }
        return;
    }

    /* ID 应答 #<ID>P! （无数字，纯 'P' 后直接 '!'） */
    if (p[0] == 'P' && p[1] == '!') {
        servo_last_reply.kind = SERVO_REPLY_ID;
        return;
    }

    /* 温度电压 #<ID>T<温度>V<电压>!  （例 #002T28.3V5.1! ，带小数点） */
    if (p[0] == 'T') {
        int32_t temp = 0, volt = 0;
        uint8_t neg_t = 0;
        /* 解析温度：T 后整数（可能负） + 可选 .小数 */
        p++;
        if (*p == '-') { neg_t = 1; p++; }
        while (*p >= '0' && *p <= '9') { temp = temp * 10 + (*p - '0'); p++; }
        int32_t tfrac = 0, tfrac_div = 1;
        if (*p == '.') {
            p++;
            while (*p >= '0' && *p <= '9') { tfrac = tfrac * 10 + (*p - '0'); tfrac_div *= 10; p++; }
        }
        if (neg_t) { temp = -temp; tfrac = -tfrac; }
        int32_t temp_x10 = temp * 10 + (tfrac * 10 / tfrac_div);

        if (*p != 'V') return;  /* 不是电压字段，退出 */
        p++;
        int32_t vfrac = 0, vfrac_div = 1;
        while (*p >= '0' && *p <= '9') { volt = volt * 10 + (*p - '0'); p++; }
        if (*p == '.') {
            p++;
            while (*p >= '0' && *p <= '9') { vfrac = vfrac * 10 + (*p - '0'); vfrac_div *= 10; p++; }
        }
        int32_t volt_x10 = volt * 10 + (vfrac * 10 / vfrac_div);

        servo_last_reply.kind = SERVO_REPLY_TV;
        servo_last_reply.temp_x10 = (int16_t)temp_x10;
        servo_last_reply.volt_x10 = (int16_t)volt_x10;
        return;
    }
}

/* 主循环轮询：把缓存里的字节搬进 Servo_Feed 解析 */
void Servo_Poll(void)
{
    uint8_t byte;
    while (servo_rx_tail != servo_rx_head) {
        byte = servo_rx_buf[servo_rx_tail];
        servo_rx_tail = (servo_rx_tail + 1) % SERVO_RX_BUF_SIZE;
        Servo_Feed(byte);
    }
}

uint8_t Servo_HasReply(void)
{
    return (s_reply_count() > 0) ? 1 : 0;
}

servo_reply_t Servo_GetReply(void)
{
    servo_reply_t r;
    __disable_irq();
    if (s_reply_tail == s_reply_head) {
        r.kind = SERVO_REPLY_NONE;
        r.id = 0;
        r.pwm = 0;
    } else {
        r = s_reply_queue[s_reply_tail];
        s_reply_tail = (uint8_t)((s_reply_tail + 1) % SERVO_REPLY_QUEUE_SIZE);
    }
    __enable_irq();
    return r;
}

/* ==================== PWM 夹爪舵机（PD15 / TIM4_CH4） ==================== */
/*
 * 与总线舵机（UART4 ASCII）无关，独立一路 PWM。
 * TIM4：PSC=71 / ARR=19999 → 72MHz / (71+1) = 1MHz → 1 count = 1us。
 * 周期 20000us = 20ms = 50Hz。
 * 角度 0~270° ↔ 500~2500us。
 */

/* 外部 TIM4 句柄（main.c 定义并由 MX_TIM4_Init 配置） */
extern TIM_HandleTypeDef htim4;

void PWM_Servo_SetPulseUs(uint16_t us)
{
    if (us < SERVO_PWM_MIN)  us = SERVO_PWM_MIN;   /* 500 */
    if (us > SERVO_PWM_MAX)  us = SERVO_PWM_MAX;   /* 2500 */
    /* 1 count = 1us，CCR 直接写脉宽 us 值 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, us);
}

void PWM_Servo_SetAngle(float deg)
{
    float us;
    if (deg < 0.0f)    deg = 0.0f;
    if (deg > 270.0f)  deg = 270.0f;
    /* 0~270° 线性映射到 500~2500us：us = 500 + deg * 2000 / 270 */
    us = 500.0f + deg * (2000.0f / 270.0f);
    PWM_Servo_SetPulseUs((uint16_t)(us + 0.5f));
}
