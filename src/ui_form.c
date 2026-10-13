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
#include "Key.h"
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
        char         buf[20];
        int          i;

        for (i = 0; i < 3; i++)
        {
            const uint16_t v = Servo_LastPos((uint8_t)(i + 1));
            if (v == 0U)
            {
                (void)snprintf(buf, sizeof(buf), "%s ---", name[i]);  /* 还没读到过 */
            }
            else
            {
                (void)snprintf(buf, sizeof(buf), "%s %u", name[i], (unsigned)v);
            }
            u8g2_DrawUTF8(g, x, y + (u8g2_uint_t)(16 * i), buf);   /* 行距 16，同 fds 的约定 */
        }
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
    UI_MUIF('P', 'W', 0, 0, draw_servo_pwm),              /* 动态只读 */
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

MUI_FORM(10)
MUI_STYLE(0)
MUI_LABEL(4, 10, "标定")
MUI_LABEL(4, 26, "待迁移")
MUI_XYAT("BK", 4, 58, 1, "返回")

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

    if (redraw != 0U)
    {
        Ui_Refresh();
    }
}
