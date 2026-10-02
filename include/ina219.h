#ifndef __INA219_H
#define __INA219_H

#include "main.h"

/*
 * INA219 分流器监测芯片驱动（应用层，依赖 ina_iic.c 软 I2C）
 *
 * 测：总线电压(mV) / 电流(mA) / 功率(mW) / 分流器压降(mV)
 *
 * 关键参数（与硬件/量程强相关，改硬件要同步改这里）：
 *   分流电阻 R_SHUNT = 0.1 Ω
 *   最大检测电流  I_MAX = 3 A
 *   PGA 量程 320mV（Gain 8）
 *   校准寄存器 CAL = 4096
 *   电流 LSB = 0.1 mA，功率 LSB = 2 mW
 */

/* ---- 器件地址（7位 0x40，A0=A1=GND 默认） ---- */
#define INA219_ADDRESS          (0x40 << 1)   /* 写地址；读地址 = +1 */

/* ---- 寄存器地址 ---- */
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNT_VOLT     0x01
#define INA219_REG_BUS_VOLT       0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05

/* ---- 配置参数 ---- */
#define INA_R_SHUNT         0.1f    /* 分流电阻（Ω） */
#define INA_CAL             4096    /* 校准寄存器值 */

/* 电流 LSB（mA），功率 LSB（mW） */
#define INA_CURRENT_LSB     0.1f    /* mA/bit */
#define INA_POWER_LSB       2.0f    /* mW/bit */

/*
 * 配置寄存器值（0x3C1F）：
 *   BRNG=0x2000  32V 量程
 *   PGA =0x1800  ±320mV（Gain 8）
 *   BADC=0x0400  12bit 总线 ADC
 *   SADC=0x0018  12bit 分流 ADC（1 采样，532us）
 *   MODE=0x0007  分流+总线连续采样
 */
#define INA219_CONFIG_VALUE   0x3C1F

/* ---- 接口 ---- */
void     INA219_Init(void);
uint32_t INA219_GetBusVoltage_MV(void);    /* 总线电压 mV */
uint32_t INA219_GetCurrent_MA(void);       /* 电流 mA */
uint32_t INA219_GetPower_MW(void);         /* 功率 mW */
uint32_t INA219_GetShuntVoltage_MV(void);  /* 分流器压降（原始，0.01mV步进） */
void     INA219_Update(void);              /* 一次性读 4 个量存全局变量（供其他任务显示） */

/* ---- 全局变量（规范版：RunTask 读存，OLED 只读显示） ---- */
extern uint32_t g_bus_voltage_mv;
extern int32_t  g_current_ma;
extern int32_t  g_power_mw;
extern int32_t  g_shunt_voltage_mv;

#endif
