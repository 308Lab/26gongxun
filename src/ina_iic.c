#include "main.h"
#include "ina_iic.h"

/* ============================================================
 * INA219 软件 I2C 实现（仿 OLED 软 IIC 风格 + 补齐读能力）
 * ============================================================ */

/* SDA 输出方向（开漏输出，写 1 = 释放总线，靠上拉拉高） */
static void INA_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = INA219_SDA_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏 */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* 上拉，保证释放后是高电平 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INA219_SDA_GPIO_Port, &GPIO_InitStruct);
}

/* SDA 输入方向（读 ACK / 读数据时用，上拉输入） */
static void INA_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = INA219_SDA_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(INA219_SDA_GPIO_Port, &GPIO_InitStruct);
}

/* 端口初始化：SCL 开漏输出上拉，SDA 先置输出 */
void INA_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SCL：开漏输出 + 上拉 */
    GPIO_InitStruct.Pin   = INA219_SCL_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INA219_SCL_GPIO_Port, &GPIO_InitStruct);

    /* SDA：开漏输出 + 上拉（读前再切输入） */
    INA_SDA_OUT();

    INA_W_SCL(1);
    INA_W_SDA(1);
}

/* 起始信号 */
void INA_I2C_Start(void)
{
    INA_SDA_OUT();
    INA_W_SDA(1);
    INA_W_SCL(1);
    INA_W_SDA(0);
    INA_W_SCL(0);
}

/* 停止信号 */
void INA_I2C_Stop(void)
{
    INA_SDA_OUT();
    INA_W_SDA(0);
    INA_W_SCL(1);
    INA_W_SDA(1);
}

/* 发送一个字节，MSB 在前 */
void INA_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    INA_SDA_OUT();
    for (i = 0; i < 8; i++)
    {
        INA_W_SDA(Byte & (0x80 >> i));
        INA_W_SCL(1);
        INA_W_SCL(0);
    }
}

/* 读一个字节，MSB 在前。ack=1 读完回 ACK(低)，ack=0 回 NACK(高) */
uint8_t INA_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, Byte = 0;
    INA_SDA_IN();                       /* SDA 切输入，准备收数据 */
    for (i = 0; i < 8; i++)
    {
        INA_W_SCL(1);
        if (INA_R_SDA())                /* 读到的位 */
        {
            Byte |= (0x80 >> i);
        }
        INA_W_SCL(0);
    }

    /* 读完回 ACK/NACK */
    INA_SDA_OUT();
    if (ack)
    {
        INA_W_SDA(0);                   /* ACK：拉低 */
    }
    else
    {
        INA_W_SDA(1);                   /* NACK：拉高 */
    }
    INA_W_SCL(1);
    INA_W_SCL(0);
    INA_W_SDA(1);                       /* 释放 SDA */

    return Byte;
}

/* 等待从机 ACK：返回 0=收到 ACK，1=未收到(NACK) */
uint8_t INA_I2C_WaitAck(void)
{
    uint8_t ack;
    INA_SDA_IN();                       /* SDA 切输入，读从机 ACK */
    INA_W_SCL(1);
    ack = INA_R_SDA();                  /* 0=从机拉低(ACK)，1=从机释放(NACK) */
    INA_W_SCL(0);
    INA_SDA_OUT();
    return ack;
}
