/**
 * @file    Arm.h
 * @brief   机械臂步进电机（Emm42_V5.0）控制层
 * @note    封装「发命令 + 收应答解析」，弥补官方 ZDT_EmmV5 库「只发不收」的空缺。
 *          本模块单独建层，不污染 lib/ZDT_EmmV5 官方库。
 *
 * 依赖：USART1（PA9=TX / PA10=RX，已接 Emm_V5 步进闭环驱动器）
 */

#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "zdt_emmv5.h"

/* EmmV5 从机地址（菜单 ID_Addr，默认 1） */
#define ARM_ADDR            0x01

/* 校验方式：0x6B 结尾（菜单 Checksum 默认 0x6B，未改 XOR/CRC-8） */

/* ============================================================
 * 读应答状态码（说明书 §6.3 通用）
 *   0x02 = 命令正确执行
 *   0xE2 = 命令收到但条件不满足
 *   0xEE = 错误命令（返回帧 地址+0x00+0xEE+0x6B）
 * ============================================================ */
#define ARM_STAT_OK         0x02
#define ARM_STAT_COND       0xE2
#define ARM_STAT_ERR        0xEE

/* ============================================================
 * 读数据命令码（以《ZDT_X42S 用户手册 V1.0.5》§5.5 为准）
 *
 * 注意：官方库 zdt_emmv5.c 的 ZDT_SysParams→命令码映射有错位：
 *   - 其 ZDT_PARAM_CPOS 实际发 0x37（位置误差），而非 0x36（实时位置）；
 *   - 其 ZDT_PARAM_TPOS 发 0x33 = 目标位置（此条映射正确）；
 *   - 缺 PERR(0x37) 显式映射。
 * 因此本模块【不走官方库 Read_Sys_Params 枚举】，直接用下面的
 * ARM_CMD_* 常量手动组帧，绕开错位坑。
 * ============================================================ */
#define ARM_CMD_VER         0x1F   /* 固件+硬件版本；返回 addr 1F FW(2B) HW_Ser HW_Type HW_Ver 6B = 7B */
#define ARM_CMD_TPOS        0x33   /* 目标位置；  返回 addr 33 符号 值(4B) 6B = 8B */
#define ARM_CMD_VEL         0x35   /* 实时转速；  返回 addr 35 符号 值(2B) 6B = 6B，Emm 单位 RPM */

/* 方向 */
#define ARM_DIR_CW          0x00   /* 注意：经减速齿轮镜像，0x00 实测=转台逆时针！ */
#define ARM_DIR_CCW         0x01   /* 0x01 实测=转台顺时针（反直觉） */

/* ============================================================
 * 姿态流转测试桩用脉冲常量（临时，正式实现后移除/替换）
 * 量纲：位置模式 3200 脉冲/圈，减速比 7：1 → 转台一圈 = 22400 脉冲
 * ============================================================ */
#define ARM_TEST_STEP_ULSE   800    /* 伺服微调步长：800 脉冲（电机轴 90° ≈ 转台 12.86°） */
#define ARM_TEST_OPEN_ULSE   5600   /* 左侧展开：5600 脉冲 = 转台 90° */

/* 运动模式（手册权威口径，勿与官方库 Pos_Control 的 relative 形参混淆） */
#define ARM_MOVE_CMD        0xFD   /* 位置模式控制（Emm）*/
#define ARM_MOVE_REL_PREV   0x00   /* 相对上一输入目标位置 */
#define ARM_MOVE_ABS_ZERO   0x01   /* 相对坐标零点（= 绝对位置） */
#define ARM_MOVE_REL_CUR    0x02   /* 相对当前实时位置 */

/* 方向 */
#define ARM_DIR_CW          0x00   /* 注意：经减速齿轮镜像，0x00 实测=转台逆时针！ */
#define ARM_DIR_CCW         0x01   /* 0x01 实测=转台顺时针（反直觉） */
#define ARM_CMD_CPOS        0x36   /* 实时位置；  返回 addr 36 符号 值(4B) 6B = 8B */
#define ARM_CMD_PERR        0x37   /* 位置误差；  返回 addr 37 符号 值(4B) 6B = 8B */
#define ARM_CMD_FLAG        0x3A   /* 状态标志；  返回 addr 3A 标志(1B) 6B = 5B */
#define ARM_CMD_ENCL        0x31   /* 线性化编码器值；返回 addr 31 值(2B 0-65535/圈) 6B */

/* ============================================================
 * 电机状态标志位（0x3A 返回，说明书 §5.5.15）
 * ============================================================ */
#define ARM_FLAG_ENABLE     0x01   /* bit0 Ens_TF：使能 */
#define ARM_FLAG_REACHED    0x02   /* bit1 Prf_TF：到位 */
#define ARM_FLAG_CLOG       0x04   /* bit2 Cgi_TF：堵转 */
#define ARM_FLAG_CLOG_PROT  0x08   /* bit3 Cgp_TF：堵转保护 */
#define ARM_FLAG_LEFT_LIM   0x10   /* bit4 Esi_LF：左限位 */
#define ARM_FLAG_RIGHT_LIM  0x20   /* bit5 Esi_RF：右限位 */
#define ARM_FLAG_POWER_OFF  0x80   /* bit7 Oac_TF：掉电 */

/* ============================================================
 * 读版本结果结构体
 * ============================================================ */
typedef struct {
    uint16_t fw_ver;    /* 固件版本 16 位（手册 §5.5.1：如 200=V2.0.0） */
    uint8_t  hw_ser;    /* 硬件系列（bit[15:12]，0=X 系 / 1=Y 系） */
    uint8_t  hw_type;   /* 硬件类型（bit[11:8]，2/3/4/5/6=28/35/42/57/86） */
    uint8_t  hw_ver;    /* 硬件版本（bit[7:0]，如 14=V2.0） */
    uint8_t  ok;        /* 1=解析成功，0=失败/超时 */
} Arm_Version_t;

/* ============================================================
 * 读「符号 + 数值」型结果的统一结构（位置/误差/转速共用）
 * ============================================================ */
typedef struct {
    uint8_t  neg;       /* 符号：0=正(+)，1=负(-) */
    uint32_t val;       /* 原始数值（未乘符号的绝对值） */
} Arm_Value_t;

/* ============================================================
 * 接口声明
 * ============================================================ */

/**
 * @brief 绑定 ZDT 库到 USART1 并初始化
 * @note  上电后调用一次。会调 ZDT_EmmV5_Init 启动 USART1 中断接收。
 */
void Arm_Init(void);

/**
 * @brief 同步收发一条「读参数」命令并解析返回
 * @param  cmd    读命令码（ARM_CMD_*）
 * @param  out    返回数据缓冲区（字节数组，长度需足够）
 * @param  expect 期望返回的数据字节数（不含 地址/命令码/结束符）
 * @retval 0=成功解析；-1=超时；-2=长度不符；-3=错误返回
 * @note  阻塞式：发送后死等应答，超时 ARM_TIMEOUT_MS 退出。
 *        仅用于一次性调试 / 低速读参数，不可用于高实时路径。
 */
int Arm_ReadParam(uint8_t cmd, uint8_t *out, uint8_t expect);

/**
 * @brief 读固件/硬件版本（一次性调试验证链路用）
 * @param  ver 输出结构体
 * @retval 0=成功，非0=失败
 * @note   按手册 §5.5.1 解析 7 字节回包：
 *         addr 1F FW_Ver(2B,大端) HW_Ser HW_Type HW_Ver 6B
 *         FW=204 即 V2.04。实机回包示例 01 1F 00 CC 23 14 6B。
 */
int Arm_ReadVersion(Arm_Version_t *ver);

/**
 * @brief 读「符号+数值」型参数（位置/误差/转速），以手册 §5.5 格式解析
 * @param  cmd  命令码（ARM_CMD_CPOS/ARM_CMD_PERR/ARM_CMD_VEL 等）
 * @param  out  输出：符号 + 原始数值（绝对值）
 * @retval 0=成功解析；-1=超时；-2=帧长度/命令码不符；-3=错误返回
 * @note   基于 Arm_CollectFrame 帧锁存接收，阻塞等待一帧。
 *         数据长度自适应：4 字节数值(CPOS/PERR/TPOS) / 2 字节数值(VEL)。
 */
int Arm_ReadValue(uint8_t cmd, Arm_Value_t *out);

/**
 * @brief 读电机状态标志（0x3A）
 * @param  flags 输出：状态标志字节（用 ARM_FLAG_* 位掩码判定）
 * @retval 0=成功；负=失败（同 Arm_ReadValue）
 */
int Arm_ReadFlags(uint8_t *flags);

/**
 * @brief 把 Emm 固件的「符号+数值」转为带符号 32 位整数
 * @param  v    Arm_ReadValue 的输出
 * @retval 带符号值（neg=1 时取负）
 */
int32_t Arm_ValueToInt32(const Arm_Value_t *v);

/**
 * @brief Emm 固件量纲转换：点值(0-65535=一圈) → 角度(°)
 * @param  raw   原始无符号值
 * @retval 角度，单位度（float）
 * @note   手册 §5.5.13：角度 = 值 * 360 / 65536
 */
float Arm_RawToDeg(uint32_t raw);

/**
 * @brief 非阻塞读取驱动器回包原始字节（调试验证回包链路用）
 * @param  out   输出缓冲区（调用方保证足够大，建议 >= ZDT_MAX_RESP_SIZE）
 * @param  max   缓冲区最大长度
 * @retval 实际读到的字节数（0 = 本次无新数据）
 * @note   每次调用会清空环形缓冲，把其中累积的字节一次性搬到 out。
 *         与 Arm_ReadParam 的阻塞式解析互斥：调试期间二选一使用。
 */
int Arm_ReadRawFrame(uint8_t *out, uint8_t max);

/**
 * @brief 累积接收一帧回包（帧锁存）：持续从环缓冲收字节，收到结束符 0x6B 锁定一帧
 * @param  out   输出缓冲区（锁存的完整帧）
 * @param  max   缓冲区最大长度（含结束符）
 * @retval 0=本帧已收满锁定；1=仍在收集中；-1=缓冲溢出/异常
 * @note   用于 OLED 长期显示回包：调用一次只在收到完整帧后返回 0，
 *         中途收半个帧返回 1（继续调），不会清掉已收字节，防“一闪而过”。
 */
int Arm_CollectFrame(uint8_t *out, uint8_t max, uint8_t *len);

/**
 * @brief 绝对位置运动：转到指定脉冲数（相对坐标零点）
 * @param  pulses   目标脉冲数
 * @param  vel      速度（RPM，0~3000）
 * @param  acc      加速度档位（0~255，0=不用曲线直接启停）
 * @param  dir      方向（ARM_DIR_CW / ARM_DIR_CCW）
 * @param  sync     同步标志（0=执行，1=缓存待同步触发）
 * @note   以手册 §5.3.12 为准，显式发「运动模式=ARM_MOVE_ABS_ZERO(0x01)」。
 *         ⚠️ 未直接调官方 ZDT_EmmV5_Pos_Control：其 relative 形参是 bool 语义，
 *         若传 0 会发出运动模式 0x00(相对上一目标)，与「绝对定位=0x01」相反，易错。
 *
 * ⚠️⚠️ 量纲坑（2026-10-01 踩坑记录）：
 *   位置模式「脉冲数」字段 = 细分16 → 3200 脉冲/圈（每脉冲 0.1125°），
 *   而「实时位置(0x36)/编码器(0x31)」读回的是 65536/圈。两套量纲不同！
 *   角度换算：pulses = 角度° × 3200 / 360（90° = 800 脉冲，不是 16384）。
 */
void Arm_MoveAbsolute(uint32_t pulses);

/**
 * @brief 相对位置运动：以当前实时位置为基准，转指定脉冲增量
 * @param  pulses   脉冲增量（Emm 位置模式 = 3200 脉冲/圈，90° = 800）
 * @param  dir      方向（ARM_DIR_CW 顺时针 / ARM_DIR_CCW 逆时针）
 * @note   手动组帧发「运动模式=ARM_MOVE_REL_CUR(0x02)」，速度 60RPM、加速度 50。
 *         以当前实时位置为基准做相对运动，绕开「绝对定位坐标零点」死结，
 *         与定稿方案 B「手动收起态 + 相对定位」一致。
 */
void Arm_MoveRelative(uint32_t pulses, uint8_t dir);

/**
 * @brief 获取最近一次成功锁存的原始回包帧（供 OLED 显示，无副作用）
 * @param  out   输出缓冲（拷贝快照）
 * @param  max   缓冲长度
 * @retval 快照长度；0=尚未收到过任何完整帧
 * @note   纯读、无竞态：由 Arm_ReadValue/ReadVersion/ReadFlags 收到帧时回填快照，
 *         OLED 任务只读此快照，不参与收帧（避免与 MainTask 抢环缓冲导致闪屏）。
 */
int Arm_GetLastFrame(uint8_t *out, uint8_t max);

#endif /* __ARM_H */
