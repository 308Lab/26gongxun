#ifndef __INA_IIC_H
#define __INA_IIC_H

#include "main.h"

/*
 * INA219 软件 I2C 驱动（仿 OLED 软 IIC 风格，但补齐「读」能力）
 *
 * 与 OLED 软 IIC 的关键差异：
 *   1. OLED 只写不读，SDA 始终输出；INA219 必须读，SDA 要能切输入。
 *   2. 要读 ACK（从机应答）判断是否响应。
 *   3. 要连续读 2 字节寄存器数据（高字节+低字节）。
 *
 * 引脚：SCL=PC2，SDA=PC3（见 main.h 的 INA219_SCL_Pin / INA219_SDA_Pin）
 * 器件地址：0x80(写) / 0x81(读)，即 7 位地址 0x40（A0=A1=GND 默认）
 */

/* ---- 引脚宏（对齐 OLED 的 W_SCL/W_SDA 风格） ---- */
#define INA_W_SCL(x)    HAL_GPIO_WritePin(INA219_SCL_GPIO_Port, INA219_SCL_Pin, (GPIO_PinState)(x))
#define INA_W_SDA(x)    HAL_GPIO_WritePin(INA219_SDA_GPIO_Port, INA219_SDA_Pin, (GPIO_PinState)(x))
#define INA_R_SDA()     HAL_GPIO_ReadPin(INA219_SDA_GPIO_Port, INA219_SDA_Pin)

/* ---- 器件地址 ---- */
#define INA219_ADDR_WRITE   0x80    /* 7位地址0x40 左移1位 + 写位0 */
#define INA219_ADDR_READ    0x81    /* 7位地址0x40 左移1位 + 读位1 */

/* ---- INA219 寄存器地址 ---- */
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLT   0x01
#define INA219_REG_BUS_VOLT     0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

/* ---- 接口 ---- */
void INA_I2C_Init(void);
void INA_I2C_Start(void);
void INA_I2C_Stop(void);
void INA_I2C_SendByte(uint8_t Byte);
uint8_t INA_I2C_ReadByte(uint8_t ack);   /* ack=1 读完后回 ACK，ack=0 回 NACK */
uint8_t INA_I2C_WaitAck(void);

#endif
