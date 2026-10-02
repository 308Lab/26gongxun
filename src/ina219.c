#include "main.h"
#include "ina219.h"
#include "ina_iic.h"

/* 全局变量（规范版：RunTask 读存，OLED 只读显示） */
uint32_t g_bus_voltage_mv = 0;
int32_t  g_current_ma = 0;
int32_t  g_power_mw = 0;
int32_t  g_shunt_voltage_mv = 0;

/* ============================================================
 * INA219 应用层实现
 * 寄存器读写 + 电压/电流/功率换算（换算系数见 ina219.h 注释）
 * ============================================================ */

/* 写一个 16 位寄存器（高字节在前） */
static void INA219_WriteReg(uint8_t reg, uint16_t data)
{
    INA_I2C_Start();
    INA_I2C_SendByte(INA219_ADDRESS);     /* 写地址 */
    INA_I2C_WaitAck();
    INA_I2C_SendByte(reg);               /* 寄存器地址 */
    INA_I2C_WaitAck();
    INA_I2C_SendByte((uint8_t)(data >> 8));    /* 高字节 */
    INA_I2C_WaitAck();
    INA_I2C_SendByte((uint8_t)(data & 0xFF));  /* 低字节 */
    INA_I2C_WaitAck();
    INA_I2C_Stop();
}

/* 读一个 16 位寄存器（高字节在前） */
static uint16_t INA219_ReadReg(uint8_t reg)
{
    uint16_t val = 0;

    INA_I2C_Start();
    INA_I2C_SendByte(INA219_ADDRESS);     /* 写地址，指定要读的寄存器 */
    INA_I2C_WaitAck();
    INA_I2C_SendByte(reg);
    INA_I2C_WaitAck();

    INA_I2C_Start();                      /* 重复起始，切读模式 */
    INA_I2C_SendByte(INA219_ADDRESS | 0x01);
    INA_I2C_WaitAck();

    val = (uint16_t)INA_I2C_ReadByte(1) << 8;   /* 高字节，回 ACK */
    val |= INA_I2C_ReadByte(0);                  /* 低字节，回 NACK */

    INA_I2C_Stop();
    return val;
}

void INA219_Init(void)
{
    INA_I2C_Init();
    INA219_WriteReg(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
    INA219_WriteReg(INA219_REG_CALIBRATION, INA_CAL);
}

/* 总线电压（mV）。寄存器 bit15~bit3 是电压（4mV/LSB），低 3 位是状态位 */
uint32_t INA219_GetBusVoltage_MV(void)
{
    uint16_t raw = INA219_ReadReg(INA219_REG_BUS_VOLT);
    return (uint32_t)((raw >> 3) * 4);
}

/* 电流（mA）。寄存器直接 × 电流LSB，带符号（可能为负） */
uint32_t INA219_GetCurrent_MA(void)
{
    int16_t raw = (int16_t)INA219_ReadReg(INA219_REG_CURRENT);
    return (uint32_t)((int32_t)raw * INA_CURRENT_LSB);
}

/* 功率（mW）。寄存器直接 × 功率LSB，带符号 */
uint32_t INA219_GetPower_MW(void)
{
    int16_t raw = (int16_t)INA219_ReadReg(INA219_REG_POWER);
    return (uint32_t)((int32_t)raw * INA_POWER_LSB);
}

/* 分流器压降（mV）。寄存器 10uV/bit，带符号 */
uint32_t INA219_GetShuntVoltage_MV(void)
{
    int16_t raw = (int16_t)INA219_ReadReg(INA219_REG_SHUNT_VOLT);
    /* 10uV/bit → mV：raw/100 */
    return (uint32_t)((int32_t)raw / 100);
}

/* 一次性读 4 个量存全局变量，供 OLED 等其他任务显示（避免 OLED 任务直接碰 I2C） */
void INA219_Update(void)
{
    g_bus_voltage_mv   = INA219_GetBusVoltage_MV();
    g_current_ma       = (int32_t)((int16_t)INA219_ReadReg(INA219_REG_CURRENT) * INA_CURRENT_LSB);
    g_power_mw         = (int32_t)((int16_t)INA219_ReadReg(INA219_REG_POWER) * INA_POWER_LSB);
    g_shunt_voltage_mv = (int32_t)((int16_t)INA219_ReadReg(INA219_REG_SHUNT_VOLT) / 100);
}
