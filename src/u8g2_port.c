/**
 * @file    u8g2_port.c
 * @brief   u8g2 的 STM32 HAL 移植层（裸机，非 Arduino）
 *
 * 移植只需实现【一个】回调 `gpio_and_delay`：
 *   字节传输直接用库自带的 `u8x8_byte_sw_i2c` —— 它靠 GPIO_I2C_CLOCK /
 *   GPIO_I2C_DATA 两条消息驱动引脚，正好落在我们要实现的回调里。
 *   （SSD1306 的命令/数据控制字节由 u8g2 的 CAD 层自动发，不用我们管。）
 *
 * 显示：SSD1306 128x64，软件 I2C，SCL=PB8 / SDA=PB9（与旧 OLED.c 同一组引脚），
 * 从机地址 0x3C（u8g2 内部默认 0x078 = 0x3C<<1）。
 *
 * ★ 用全缓冲（`_f`，1KB 静态显存，来自库的 u8g2_d_memory.c，不占堆）：
 *   一次 SendBuffer 就推完，不用 picture loop，也不会闪。
 *
 * ⚠️ 尚未上机验证 —— 目前只保证编译零警告。首次上机要确认两件事：
 *   ① 屏幕点亮且内容正确；
 *   ② 若花屏/抖动，调 U8G2_I2C_DELAY_US（软件 I2C 半周期延时，默认 2us ≈ 250kHz）。
 */

#include "u8g2_port.h"
#include "main.h"

#include "u8g2.h"

/* 显示引脚 —— 与旧 OLED.c 完全一致，硬件不用动 */
#define U8G2_SCL_PORT   OLED_SCL_GPIO_Port
#define U8G2_SCL_PIN    OLED_SCL_Pin
#define U8G2_SDA_PORT   OLED_SDA_GPIO_Port
#define U8G2_SDA_PIN    OLED_SDA_Pin

/* 软件 I2C 的半周期延时（微秒）。
 *   5us → 约 100kHz（SSD1306 的标准速率，最稳）。刷新一屏 1024 字节 ≈ 90ms。
 *   ★ 曾用 2us（250kHz）—— 上机后屏幕内容整体偏移、只剩一部分，这是典型的
 *     "I2C 丢字节"（丢一个字节，后面整条数据前移，视觉上就是内容左移）。
 *     SSD1306 标称支持 400kHz，但软件位翻转 + DWT 延时未必准，先退到 100kHz。
 *     若确认没问题再往上调（每半周期 2.5us 对应 200kHz）。 */
#define U8G2_I2C_DELAY_US   5U

/* u8g2 实例。静态分配（全缓冲 1KB 在库内的静态数组里，不走堆）。 */
static u8g2_t s_u8g2;

/* ============================================================
 * 微秒延时：用 DWT 周期计数器
 *   F103 是 Cortex-M3，带 DWT。比空循环精确，也不怕 -O2 把循环优化掉。
 * ============================================================ */
static void port_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t need;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        /* 首次调用时打开：DEMCR 的 TRCENA 是 DWT 的总开关，必须先置它。 */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    start = DWT->CYCCNT;
    need = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < need)
    {
        /* 忙等 —— 软件 I2C 的位间隔，不需要让出 CPU */
    }
}

/* ============================================================
 * u8g2 要求的回调：GPIO 与延时
 *   返回值：1 = 已处理，0 = 不理会（库对多数消息不看返回值）
 * ============================================================ */
static uint8_t port_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg)
    {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            /* 引脚方向与模式已在 MX_GPIO_Init 里配好（PB8/PB9 推挽输出），
             * 这里无需重复。 */
            break;

        case U8X8_MSG_DELAY_MILLI:
            /* 复位时序用。HAL 时基是 TIM2，与 FreeRTOS 的 SysTick 不冲突。 */
            HAL_Delay(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:
            port_delay_us((uint32_t)arg_int * 10U);
            break;

        case U8X8_MSG_DELAY_I2C:
            port_delay_us(U8G2_I2C_DELAY_US);
            break;

        case U8X8_MSG_GPIO_I2C_CLOCK:
            HAL_GPIO_WritePin(U8G2_SCL_PORT, U8G2_SCL_PIN,
                              (arg_int != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_I2C_DATA:
            HAL_GPIO_WritePin(U8G2_SDA_PORT, U8G2_SDA_PIN,
                              (arg_int != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        default:
            /* 其余消息（DELAY_NANO / 100NANO、SPI 的 CS/DC/D0-D7、复位脚……）
             * 软件 I2C 的 SSD1306 用不到。注意本模块没有接复位脚
             * （旧 OLED.c 也没用），所以 GPIO_RESET 一并落到这里。 */
            return 0;
    }
    return 1;
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void U8g2Port_Init(void)
{
    /* 全缓冲构造函数：第 3 个参数传库自带的软件 I2C 字节传输，
     * 第 4 个是我们实现的 GPIO/延时回调。 */
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&s_u8g2, U8G2_R0,
                                           u8x8_byte_sw_i2c,
                                           port_gpio_and_delay);
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);   /* 0 = 开显示 */
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
}

u8g2_t* U8g2Port_Handle(void)
{
    return &s_u8g2;
}

void U8g2Port_SendBuffer(void)
{
    u8g2_SendBuffer(&s_u8g2);
}
