/**
 * @file    ui_form.c
 * @brief   OLED 表单式人机界面（u8g2 + MUI）实现
 *
 * 结构就三样东西：muif 字段表、fds 表单描述、mui 实例 —— MUI 官方的写法。
 * 导航：三个键（见下面的 UI_KEY_*）映射到 MUI 的三个动作。
 *
 * ⚠️ 尚未上机验证。首次上机会重点看三件事：
 *   ① 中文有没有缺字（wqy12_t_chinese1 只是【常用字子集】，缺字会显示成方块）；
 *   ② 一屏放不放得下（12px 字体，128x64 大概 4 行）；
 *   ③ 翻页/改值时刷屏闪不闪（全缓冲一次推，理论上不闪）。
 */

#include "ui_form.h"

#include <stdio.h>

#include "cmsis_os.h"     /* osDelay（上电自检图的停留） */

#include "u8g2_port.h"
#include "task_seq.h"     /* Task_RunAt（操作页的任务入口） */
#include "Vision.h"       /* g_vision：执行页显示扫码三位数与错误码 */
#include "arm_zero.h"     /* 标定页：记录零位 / 查询是否已标定 */
#include "Key.h"          /* KEY_CALIB = 物理键 9（标定页的使能/失能） */
#include "Servo.h"

#include "u8g2.h"
#include "mui.h"
#include "mui_u8g2.h"

/* ── 键盘映射 ─────────────────────────────────────────────────────────────
 * 用【底排】三个键（物理上自左向右），编号 = row*3+col，见 Key.h：
 *   键 1（编号 6）→ 进入
 *   键 2（编号 7）→ 下一个（到头循环）
 *   键 3（编号 8）→ 返回上一页
 *
 * ★ 没有"上一个"：三个键不够分。菜单项最多 4 条，一路"下一个"循环比上下分两个键
 *   更省事；真需要再给键 2 加长按反向前进（Key.h 已有 KEY_LONG / KEY_REPEAT）。 */
#define UI_KEY_ENTER  6
#define UI_KEY_NEXT   7
#define UI_KEY_BACK   8

static mui_t s_mui;

/* ── 无 snprintf 的数字格式化 ────────────────────────────────────────────
 * ★ 为什么不用 snprintf：newlib-nano 的 snprintf 单帧 136B，再叠 _svfiprintf_r
 *   116B，合计 250+B —— 在 OLEDTask 这条本就吃栈的路径上代价过大；而且它的
 *   %u 路径会走 _malloc_r（FreeRTOS 堆只有 8KB，20ms 重绘里反复申请有碎片风险）。
 *   这几个字段只画 1~4 位无符号数（或带符号的小偏移），手写展开即可，
 *   栈开销从 250+B 降到几十字节。这就是「子页面卡死」的直接修法之一。 */
static char* u16_to_str(char *p, uint16_t v)
{
    char    tmp[6];
    uint8_t n = 0u;

    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u);
    while (n != 0u) { *p++ = tmp[--n]; }
    *p = '\0';
    return p;
}

/* 始终带符号，等价 printf 的 %+d（偏移量显示要保持正数带 '+'，与原先一致） */
static char* i16_sign_str(char *p, int16_t v)
{
    if (v < 0) { *p++ = '-'; return u16_to_str(p, (uint16_t)(-(int32_t)v)); }
    *p++ = '+';
    return u16_to_str(p, (uint16_t)v);
}

/* ── 自定义字段：动态只读文本 ──────────────────────────────────────────── */

/* 三路舵机的【实测】位置。数据源是 Servo_LastPos()（回读缓存），
 * 不是命令值 —— 标定时要看的就是"舵机实际停在哪"。 */
static uint8_t draw_servo_pwm(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        static const char* const name[3] = {"P1", "P2", "P3"};
        u8g2_t*      g = mui_get_U8g2(ui);
        u8g2_uint_t  x = mui_get_x(ui);
        u8g2_uint_t  y = mui_get_y(ui);
        char         buf[16];
        int          i;

        for (i = 0; i < 3; i++)
        {
            const uint16_t v = Servo_LastPos((uint8_t)(i + 1));
            char*          p = buf;

            p[0] = name[i][0]; p[1] = name[i][1]; p[2] = ' '; p += 3;
            if (v == 0U) { *p++ = '-'; *p++ = '-'; *p++ = '-'; *p = '\0'; }
            else         { (void)u16_to_str(p, v); }   /* 还没读到过则显示 --- */

            u8g2_DrawUTF8(g, x, y + (u8g2_uint_t)(16 * i), buf);   /* 行距 16，同 fds 的约定 */
        }
    }
    return 0;
}

/* ── 执行页的两行动态文本 ──────────────────────────────────────────────── */

/* 二维码三位数（协议 §3.2，2026-10-13 起随 0x11 回传）。
 * 用途是现场核对「扫到的码 = 抽签的码」—— 扫错码却一路跑下去是最贵的错。 */
static uint8_t draw_qr_digits(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t*     g = mui_get_U8g2(ui);
        u8g2_uint_t x = mui_get_x(ui);
        u8g2_uint_t y = mui_get_y(ui);
        char        buf[20];

        /* 前缀 "二维码 " 是 4 个 UTF-8 字（每个 3 字节），共 12 字节；其余是纯 ASCII。 */
        static const char k_prefix[] = "\xe4\xba\x8c\xe7\xbb\xb4\xe7\xa0\x81 ";  /* "二维码 " */
        char* p = buf;
        uint8_t i;

        for (i = 0u; i < sizeof(k_prefix) - 1u; i++) *p++ = k_prefix[i];

        if (g_vision.scan_has_digits != 0U)
        {
            *p++ = (char)g_vision.scan_digits[0];
            *p++ = (char)g_vision.scan_digits[1];
            *p++ = (char)g_vision.scan_digits[2];
            *p = '\0';
        }
        else if (g_vision.scan_done != 0U)
        {
            static const char k_tail[] = "\xe6\x9c\xaa\xe5\xb8\xa6\xe5\x9b\x9e";  /* "未带回" */
            for (i = 0u; i < sizeof(k_tail) - 1u; i++) *p++ = k_tail[i];
            *p = '\0';
        }
        else
        {
            static const char k_tail[] = "\xe7\xad\x89\xe5\xbe\x85\xe4\xb8\xad";  /* "等待中" */
            for (i = 0u; i < sizeof(k_tail) - 1u; i++) *p++ = k_tail[i];
            *p = '\0';
        }
        u8g2_DrawUTF8(g, x, y, buf);
    }
    return 0;
}

/* 最近一次识别错误（协议 §3.1b 的 0x12 错误码）。 */
static uint8_t draw_last_error(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t*     g = mui_get_U8g2(ui);
        u8g2_uint_t x = mui_get_x(ui);
        u8g2_uint_t y = mui_get_y(ui);
        char        buf[20];
        char*       p = buf;
        uint8_t     i;

        if (g_vision.last_err_code != 0U)
        {
            static const char k_pre[]  = "\xe9\x94\x99\xe8\xaf\xaf\xe7\xa0\x81 ";  /* "错误码 " */
            for (i = 0u; i < sizeof(k_pre) - 1u; i++) *p++ = k_pre[i];
            (void)u16_to_str(p, (uint16_t)g_vision.last_err_code);
        }
        else
        {
            static const char k_ok[] = "\xe7\x8a\xb6\xe6\x80\x81 \xe6\xad\xa3\xe5\xb8\xb8";  /* "状态 正常" */
            for (i = 0u; i < sizeof(k_ok) - 1u; i++) *p++ = k_ok[i];
            *p = '\0';
        }
        u8g2_DrawUTF8(g, x, y, buf);
    }
    return 0;
}

/* ── 标定页（form 11）的字段 ───────────────────────────────────────────── */

/* 上次「记忆为零位」的结果，供状态行显示失败原因（0=成功或还没按过） */
static int s_zero_rc = 0;

/* 三路实测 PWM 挤在一行。
 * 尺寸账：wqy12 的中文是全角、ASCII 是半角（数字步进 6px），
 * 所以 "1:1976 2:2425 3:1762" 这 19 个字符约 109px，128px 宽放得下 —— 不需要另换字体。 */
static uint8_t draw_zero_pwms(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t*  g = mui_get_U8g2(ui);
        char     buf[28];
        char*    p = buf;
        uint16_t a = Servo_LastPos(1);
        uint16_t b = Servo_LastPos(2);
        uint16_t c = Servo_LastPos(3);

        if ((a != 0U) && (b != 0U) && (c != 0U))
        {
            *p++ = '1'; *p++ = ':'; p = u16_to_str(p, a);
            *p++ = ' '; *p++ = '2'; *p++ = ':'; p = u16_to_str(p, b);
            *p++ = ' '; *p++ = '3'; *p++ = ':'; p = u16_to_str(p, c);
        }
        else
        {
            static const char k_wait[] = "\xe5\x9b\x9e\xe8\xaf\xbb\xe4\xb8\xad \xe8\xaf\xb7\xe7\xa8\x8d\xe5\x80\x99"; /* "回读中 请稍候" */
            uint8_t i;
            for (i = 0u; i < sizeof(k_wait) - 1u; i++) *p++ = k_wait[i];
            *p = '\0';
        }
        u8g2_DrawUTF8(g, mui_get_x(ui), mui_get_y(ui), buf);
    }
    return 0;
}

/* 当前生效的偏移量；按下记忆键失败时改为显示原因。
 * 未标定要明确写出来 —— 免得对着全 0 的偏移猜"到底标上没有"。 */
static uint8_t draw_zero_offset(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t* g = mui_get_U8g2(ui);
        char    buf[28];
        char*   p = buf;
        uint8_t i;

        if (s_zero_rc == -1)
        {
            static const char k[] = "\xe5\x9b\x9e\xe8\xaf\xbb\xe6\x97\xa0\xe6\x95\x88 \xe7\xa8\x8d\xe5\x80\x99\xe5\x86\x8d\xe8\xaf\x95"; /* "回读无效 稍候再试" */
            for (i = 0u; i < sizeof(k) - 1u; i++) *p++ = k[i];
            *p = '\0';
        }
        else if (s_zero_rc == -2)
        {
            static const char k[] = "\xe5\x81\x8f\xe7\xa7\xbb\xe8\xb6\x85\xe9\x99\x90 \xe6\xb2\xa1\xe6\x91\x86\xe5\x88\xb0\xe4\xbd\x8d?"; /* "偏移超限 没摆到位?" */
            for (i = 0u; i < sizeof(k) - 1u; i++) *p++ = k[i];
            *p = '\0';
        }
        else if (s_zero_rc == -3)
        {
            static const char k[] = "Flash \xe5\x86\x99\xe5\x85\xa5\xe5\xa4\xb1\xe8\xb4\xa5"; /* "Flash 写入失败" */
            for (i = 0u; i < sizeof(k) - 1u; i++) *p++ = k[i];
            *p = '\0';
        }
        else if (ArmZero_IsCalibrated() != 0U)
        {
            static const char k[] = "\xe5\x81\x8f "; /* "偏 " */
            for (i = 0u; i < sizeof(k) - 1u; i++) *p++ = k[i];
            p = i16_sign_str(p, Servo_GetZeroOffset(1)); *p++ = ' ';
            p = i16_sign_str(p, Servo_GetZeroOffset(2)); *p++ = ' ';
            p = i16_sign_str(p, Servo_GetZeroOffset(3));
        }
        else
        {
            static const char k[] = "\xe6\x9c\xaa\xe6\xa0\x87\xe5\xae\x9a\xef\xbc\x88\xe5\x87\xba\xe5\x8e\x82\xe9\x9b\xb6\xe4\xbd\x8d\xef\xbc\x89"; /* "未标定（出厂零位）" */
            for (i = 0u; i < sizeof(k) - 1u; i++) *p++ = k[i];
            *p = '\0';
        }
        u8g2_DrawUTF8(g, mui_get_x(ui), mui_get_y(ui), buf);
    }
    return 0;
}

/* 舵机力矩状态。★ 这是【安全信息】，必须常驻可见 —— 失力时臂会因自重垮下来。 */
static uint8_t draw_lock_state(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t* g = mui_get_U8g2(ui);
        u8g2_DrawUTF8(g, mui_get_x(ui), mui_get_y(ui),
                      (Servo_GetArmTorque() != 0U) ? "已锁定(键9失力)" : "已失力 可摆臂");
    }
    return 0;
}

/* 记忆为零位：读三路【原始】回读值交给 ArmZero_Record，成功后立即恢复力矩。
 * 画法与 GOTO 的 _w1_ 一致（整行宽），选中时不跳页。 */
static uint8_t btn_zero_record(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t* g = mui_get_U8g2(ui);
        mui_u8g2_draw_button_fi(ui, u8g2_GetDisplayWidth(g) - mui_get_x(ui) * 2U,
                                mui_get_x(ui) - 1U, ui->text);
    }
    else if (msg == MUIF_MSG_CURSOR_SELECT)
    {
        uint16_t raw[3];
        raw[0] = Servo_LastPos(1);
        raw[1] = Servo_LastPos(2);
        raw[2] = Servo_LastPos(3);
        s_zero_rc = ArmZero_Record(raw);
        if (s_zero_rc == 0)
        {
            Servo_SetArmTorque(1U);   /* 记完立即上力，别让它一直瘫着 */
        }
    }
    return 0;
}

/* 隐藏字段：离开本页（物理键 3 返回、或任何 GOTO）时自动恢复力矩。
 * MUI 的 mui_LeaveForm 会对当前表单的每个字段派发 MUIF_MSG_FORM_END，挂这里正合适。
 * 防的是"解了锁、摆完臂、忘了按记忆就退出"—— 那时臂是瘫的。 */
static uint8_t zero_form_end(mui_t *ui, uint8_t msg)
{
    (void)ui;
    if (msg == MUIF_MSG_FORM_END)
    {
        Servo_SetArmTorque(1U);
    }
    return 0;
}

/* ── 自定义字段：启动任务 ────────────────────────────────────────────────
 * 画法与 GOTO 的 _w1_ 一致（整行宽的框 + 左对齐文字），但选中时不跳页，
 * 而是按 fds 里传来的 arg 启动对应任务。
 * arg 就是任务点号（A=0 / B=1 / C=2 / D=3），引擎按它选任务表。 */
static uint8_t btn_run_task(mui_t *ui, uint8_t msg)
{
    if (msg == MUIF_MSG_DRAW)
    {
        u8g2_t* g = mui_get_U8g2(ui);
        mui_u8g2_draw_button_fi(ui, u8g2_GetDisplayWidth(g) - mui_get_x(ui) * 2U,
                                mui_get_x(ui) - 1U, ui->text);
    }
    else if (msg == MUIF_MSG_CURSOR_SELECT)
    {
        /* 从【当前位置】直接去那个点并执行（Ryan 选的方案）——
         * 不跑前面的路，调试时能单独测某个任务。 */
        Task_RunAt((uint8_t)ui->arg);

        /* 紧接着跳到执行页（form 5）：一边跑一边能看扫码结果与报错。
         * ★ 用 mui_GotoFormAutoCursorPosition，不用裸 mui_GotoForm ——
         *   MUI 源码对 mui_GotoForm/mui_SaveForm 明写："if called from a field
         *   function, then the current field variables are destroyed, so that
         *   call should be the last call in the field callback"。官方按钮
         *   （mui_u8g2_btn_goto_w1_fi）走的正是 AutoCursorPosition 这个变体。
         *   （mui_Init 有 memset，menu_form_id[] 全 0 → cursor_position 恒为 0，
         *     与原先显式传 0 行为等价，见 mui_GotoFormAutoCursorPosition。） */
        mui_SaveForm(ui);
        (void)mui_GotoFormAutoCursorPosition(ui, 5);
    }
    return 0;
}

/* ── 字段表（muif）：MUI 认识哪些字段类型 ───────────────────────────────
 *
 * ★ 这里【没有】用 u8g2 原生的 MUIF_* 宏：它们把 id 写成字符串再取 id[0]/id[1]，
 *   而"字符串字面量取下标"在 C 里不是整数常量表达式 —— 拿它做数组的静态初始化
 *   会被 GCC 拒绝（`initializer element is not constant`）。
 *   u8g2 的示例都是 C++（Arduino），那边允许，所以上游没暴露这个问题。
 *   自写一个等价宏，id 直接给两个字符字面量即可。
 *   muif_t = { id0, id1, cflags, dflags, data, cb }，顺序见 mui.h 的 MUIF 宏。 */
#define UI_MUIF(id0, id1, cflags, data, cb) { (id0), (id1), (cflags), 0, (data), (cb) }

static muif_t muif_list[] = {
    /* 字体样式 0：中文（wqy12 ≈ 文泉驿 12px）。MUI 不自带默认字体，必须自己注册。 */
    UI_MUIF('S', '0', 0, (void *)u8g2_font_wqy12_t_gb2312,
            mui_u8g2_set_font_style_function),
    UI_MUIF('.', 'L', 0, 0, mui_u8g2_draw_text),          /* 静态文本 */
    UI_MUIF('P', 'W', 0, 0, draw_servo_pwm),              /* 动态只读：三路舵机 PWM */
    UI_MUIF('Q', 'R', 0, 0, draw_qr_digits),              /* 动态只读：二维码三位数 */
    UI_MUIF('E', 'R', 0, 0, draw_last_error),             /* 动态只读：最近错误码 */
    /* 标定页（form 11）。前缀 'Z' 与上面各字段不冲突。 */
    UI_MUIF('Z', 'P', 0, 0, draw_zero_pwms),              /* 动态只读：三路实测 PWM（挤一行） */
    UI_MUIF('Z', 'O', 0, 0, draw_zero_offset),            /* 动态只读：当前偏移量 / 报错 */
    UI_MUIF('Z', 'S', 0, 0, draw_lock_state),             /* 动态只读：舵机力矩状态（安全信息） */
    UI_MUIF('Z', 'M', MUIF_CFLAG_IS_CURSOR_SELECTABLE, 0,
            btn_zero_record),                             /* 记忆为零位（动作按钮） */
    UI_MUIF('Z', 'E', 0, 0, zero_form_end),               /* 隐藏：离开本页自动恢复力矩 */
    /* ★ 用 _w1_ 变体而不是 _wm_：_wm_ 的框宽是【自动算文字宽度】得到的
     *   （mui_u8g2_draw_button_utf(..., H_CENTER, width=0, is_fixed_width=1, ...)），
     *   而中文字体的 UTF8 宽度在那个路径上没算对 —— 框与文字错开，上机表现为
     *   "只剩一部分字 + 上下一点框"。_w1_ 的宽度是显式的（屏宽 - x*2，整行铺满），
     *   完全不碰宽度计算。上机验证：自检图正常、LABEL 正常、只有 GOTO 异常 → 就是这里。 */
    UI_MUIF('.', 'G', MUIF_CFLAG_IS_CURSOR_SELECTABLE, 0,
            mui_u8g2_btn_goto_w1_fi),                     /* 跳子页（整行宽） */
    UI_MUIF('B', 'K', MUIF_CFLAG_IS_CURSOR_SELECTABLE, 0,
            mui_u8g2_btn_back_w1_fi),                     /* 返回上一页（整行宽） */
    UI_MUIF('T', 'N', MUIF_CFLAG_IS_CURSOR_SELECTABLE, 0,
            btn_run_task),                                /* 启动任务（见上） */
};

/* ── 表单描述（fds）：页面长什么样 ───────────────────────────────────────
 *
 * ★ 坐标只能写字面量：MUI_x / MUI_y 是把数字【编码成单字节】的宏，不接受表达式，
 *   所以行距没法做成可复用的宏，只能照下面这个约定排。
 *
 * ★ 行距 = 16px。屏幕 128x64，而 wqy12 的实际字高【比 12 大】（含汉字全角 bbox，
 *   约 15~16px）—— 第一版按 12px 排（y=12/28/44/60），结果最后两行被切到底。
 *   64 / 16 = 4 行，所以一页最多 4 行内容是硬上限，排布时先数行数。 */
static fds_t fds_data[] =
MUI_FORM(1)
MUI_STYLE(0)
MUI_GOTO(4, 10, 3, "操作")
MUI_GOTO(4, 26, 2, "状态")
MUI_GOTO(4, 42, 10, "标定")
MUI_GOTO(4, 58, 20, "测试")

MUI_FORM(2)
MUI_STYLE(0)
MUI_XY("PW", 4, 10)       /* 内部画 3 行：10 / 26 / 42 */
MUI_XYAT("BK", 4, 58, 1, "返回")

/* 操作页：四个入口，arg 即 formal_end_t（A=0/B=1/C=2/D=3）。
 * ★ 语义是【从起点出发跑过去并完成到那一步】—— formal_end_t 是累积的
 *   （AFTER_B 会先跑完 A 的路再在 B 停下），所以没有"只做 B 不跑 A"的选项。
 *   若你要的是"直接跳到某点"（不跑前面的路），说一声，那走 PathRunner_Goto()。 */
MUI_FORM(3)
MUI_STYLE(0)
MUI_XYAT("TN", 4, 10, 0, "任务A 二维码识别")
MUI_XYAT("TN", 4, 26, 1, "任务B 小球排爆")
MUI_XYAT("TN", 4, 42, 2, "任务C 激光反恐")
MUI_XYAT("TN", 4, 58, 3, "任务D 抓取人质")

/* 任务执行页：从操作页按确认进来（见 btn_run_task），任务同时在跑。
 * 显示扫码三位数（现场核对用）与最近一次识别错误码。 */
MUI_FORM(5)
MUI_STYLE(0)
MUI_LABEL(4, 10, "任务执行")
MUI_XY("QR", 4, 26)
MUI_XY("ER", 4, 42)
MUI_XYAT("BK", 4, 58, 1, "返回")

/* 标定菜单：目前只有零位标定一项。第三行是常驻提示 —— θ1 是增量式步进、无绝对零点，
 * 它的"零位"就是开机位置，只能靠人保证（详见 arm_config.h 的 D 段）。 */
MUI_FORM(10)
MUI_STYLE(0)
MUI_LABEL(4, 10, "标定")
MUI_GOTO(4, 26, 11, "零位标定")
MUI_LABEL(4, 42, "θ1零位=开机位")
MUI_XYAT("BK", 4, 58, 1, "返回")

/* 零位标定页：把臂摆到收起态后按【记忆为零位】。
 * · 使能/失能走【物理键 9】（KEY_CALIB），不占屏幕行 —— 见 Ui_Tick。
 * · 本页不排"返回"行：物理键 3 就是全局返回（Ui_Tick 里无条件处理）。 */
MUI_FORM(11)
MUI_STYLE(0)
MUI_XY("ZP", 4, 10)
MUI_XY("ZO", 4, 26)
MUI_XY("ZS", 4, 42)
MUI_XYAT("ZM", 4, 58, 0, "记忆为零位")
MUI_XY("ZE", 0, 0)      /* 隐藏字段：只为接 FORM_END（离开本页自动恢复力矩） */

MUI_FORM(20)
MUI_STYLE(0)
MUI_LABEL(4, 10, "测试")
MUI_LABEL(4, 26, "待迁移")
MUI_XYAT("BK", 4, 58, 1, "返回")
;

/* ── 对外接口 ──────────────────────────────────────────────────────────── */

/* ── 上电自检图 ──────────────────────────────────────────────────────────
 * 不是装饰，是为了让"显示不对"这件事变得可以描述、可以定位：
 *   · 四边都有边框           → 坐标原点和屏幕尺寸对
 *   · 对角线从左【上】到右【下】 → 方向对（镜像/翻转会立刻看出来）
 *   · "1234" 在左上、圆在正中 → 文字/图形的基准位置对
 * 看完 UI_SELFTEST_MS 毫秒后自动进菜单。不需要时把它删掉即可。 */
#define UI_SELFTEST_MS 2000

static void ui_selftest(void)
{
    u8g2_t* g = U8g2Port_Handle();

    u8g2_ClearBuffer(g);
    u8g2_DrawFrame(g, 0, 0, 128, 64);
    u8g2_DrawLine(g, 0, 0, 127, 63);
    u8g2_DrawCircle(g, 64, 32, 20, U8G2_DRAW_ALL);
    u8g2_SetFont(g, u8g2_font_wqy12_t_gb2312);
    u8g2_DrawStr(g, 2, 12, "1234");
    u8g2_SendBuffer(g);
}

void Ui_Init(void)
{
    u8g2_t* g;

    U8g2Port_Init();          /* 屏幕先起来 */
    g = U8g2Port_Handle();
    u8g2_SetFont(g, u8g2_font_wqy12_t_gb2312);

    ui_selftest();            /* 自检图（见上） */
    osDelay(UI_SELFTEST_MS);  /* 本函数只在 OLEDTask 上下文被调用，可以阻塞 */

    mui_Init(&s_mui, g, fds_data, muif_list,
             sizeof(muif_list) / sizeof(muif_t));
    mui_GotoForm(&s_mui, 1, 0);   /* 从主菜单开始 */

    Ui_Refresh();
}

void Ui_Refresh(void)
{
    u8g2_t* g = U8g2Port_Handle();

    u8g2_ClearBuffer(g);
    mui_Draw(&s_mui);
    U8g2Port_SendBuffer();    /* 全缓冲一次推完，不闪 */
}

/* ── 重画节流：页面「内容签名」变了才重画 ────────────────────────────────
 *
 * 软件 I2C 推一屏 1024 字节约 90ms，而 OLEDTask 每 20ms 就跑一次 ——
 * 无脑周期重画会吃掉两成以上 CPU。所以记住上次画过的内容，只有真变了才重画。
 *
 * ★ 2026-10-14 修（Ryan 上机发现）：原先只比较【扫码】相关的三个量，于是「状态」页
 *   （三路舵机 PWM）进去后画一次就再也不刷新。判据扩成覆盖所有会变的页面元素。
 *   标定页也要显示实时 PWM，会踩同一个坑，所以这里是通用修复、不是给状态页打补丁。
 *
 * ★ 舵机 PWM 带 ±1 回读抖动（回读值由 50ms 轮询更新），必须留死区 ——
 *   否则静止时也会反复重画，把上面那个 90ms/屏 的代价一直付着。 */
#define UI_PWM_DEADBAND  2U

static uint8_t  s_shown_digits[3] = {0, 0, 0};
static uint8_t  s_shown_scan   = 0xFF;   /* 0xFF = 还没显示过 */
static uint8_t  s_shown_err    = 0xFF;
static uint16_t s_shown_pwm[3] = {0, 0, 0};

/* 两个 PWM 是否【明显】不同（超出死区） */
static uint8_t pwm_differs(uint16_t a, uint16_t b)
{
    return (uint8_t)(((a > b) ? (a - b) : (b - a)) > UI_PWM_DEADBAND);
}

static uint8_t page_view_changed(void)
{
    uint8_t i;

    if (s_shown_scan != g_vision.scan_has_digits) return 1;
    if (s_shown_err  != g_vision.last_err_code)   return 1;
    if (g_vision.scan_has_digits != 0U)
    {
        if ((s_shown_digits[0] != g_vision.scan_digits[0]) ||
            (s_shown_digits[1] != g_vision.scan_digits[1]) ||
            (s_shown_digits[2] != g_vision.scan_digits[2])) return 1;
    }

    /* 三路舵机实测 PWM：状态页（form 2）与标定页（form 11）都靠它刷新。
     * Servo_LastPos 返回 0 = "还没读到过"，0 → 实测值 同样会触发重画。 */
    for (i = 0; i < 3U; i++)
    {
        if (pwm_differs(s_shown_pwm[i], Servo_LastPos((uint8_t)(i + 1U))) != 0U) return 1;
    }
    return 0;
}

static void page_view_mark_shown(void)
{
    uint8_t i;

    s_shown_scan = g_vision.scan_has_digits;
    s_shown_err  = g_vision.last_err_code;
    s_shown_digits[0] = g_vision.scan_digits[0];
    s_shown_digits[1] = g_vision.scan_digits[1];
    s_shown_digits[2] = g_vision.scan_digits[2];
    for (i = 0; i < 3U; i++) s_shown_pwm[i] = Servo_LastPos((uint8_t)(i + 1U));
}

void Ui_Tick(void)
{
    uint8_t redraw = 0;

    /* Key_Check 读到即清标志（KEY_HOLD 除外），所以这里就是边沿事件。 */
    if (Key_Check(UI_KEY_NEXT,  KEY_DOWN) != 0U) { mui_NextField(&s_mui);  redraw = 1; }
    if (Key_Check(UI_KEY_ENTER, KEY_DOWN) != 0U) { mui_SendSelect(&s_mui); redraw = 1; }
    if (Key_Check(UI_KEY_BACK,  KEY_DOWN) != 0U)
    {
        /* 返回上一页。MUI 只保存【一层】（mui_t 里就一份 last_form），而我们最深
         * 只有"主菜单 → 子页"两层，所以够用：跳进子页时 mui_u8g2_btn_goto_w1_fi
         * 会自己调 mui_SaveForm()，这里 restore 回去即可。
         * 已经在主菜单时 restore 到自己，无副作用。 */
        (void)mui_RestoreForm(&s_mui);
        redraw = 1;
    }

    /* 键 9（物理键 9 = 编号 2 = KEY_CALIB）：三路臂舵机 锁定 ⇄ 失力，供标定时手动摆臂。
     *
     * ★ 必须【无条件读取】把它消费掉：Key_Check 读到即清、不读不清。若只在标定页读，
     *   在别的页面按下的那一次会一直挂着，等进了标定页时才补触发 —— 机械臂会突然垮下来。
     *   所以读取无条件，只有【动作】才判当前页面。
     * ★ 限定在 form 11：任务跑一半误按 9 会导致失力、臂自重砸下，这是最贵的错。 */
    if (Key_Check(KEY_CALIB, KEY_DOWN) != 0U)
    {
        if (mui_GetCurrentFormId(&s_mui) == 11)
        {
            Servo_SetArmTorque((uint8_t)((Servo_GetArmTorque() == 0U) ? 1U : 0U));
            s_zero_rc = 0;   /* 清了上次的报错，免得占着状态行 */
            redraw = 1;
        }
    }

    /* 页面内容变了也重画（执行页的扫码结果、状态页/标定页的三路 PWM 都靠这条刷新） */
    if (page_view_changed() != 0U)
    {
        redraw = 1;
    }

    if (redraw != 0U)
    {
        Ui_Refresh();
        page_view_mark_shown();
    }
}
