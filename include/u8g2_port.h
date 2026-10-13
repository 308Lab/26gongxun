/**
 * @file    u8g2_port.h
 * @brief   u8g2 图形库在本工程（裸机 STM32F103 + HAL，非 Arduino）上的移植接口
 *
 * 只暴露最小面：初始化、拿句柄、推屏。UI 层（ui_form.c）用句柄画。
 * u8g2.h 不外传 —— 只有 u8g2_port.c 与 ui_form.c 需要 include 它。
 */

#ifndef __U8G2_PORT_H
#define __U8G2_PORT_H

#include <stdint.h>

/* 前向声明，避免把 u8g2.h 传染给每一个包含本头的编译单元 */
typedef struct u8g2_struct u8g2_t;

/**
 * @brief 初始化显示屏（重启时序 + 上电配置）。
 * @note  可与 osKernelStart 前后调用；内部用 HAL_Delay（时基是 TIM2，不与
 *        FreeRTOS 的 SysTick 冲突）。
 */
void U8g2Port_Init(void);

/** u8g2 实例句柄。UI 层用它调 u8g2_* / mui_*。 */
u8g2_t* U8g2Port_Handle(void);

/** 把当前缓冲推到屏幕（全缓冲模式，一次 I2C 传输，不闪）。 */
void U8g2Port_SendBuffer(void);

#endif /* __U8G2_PORT_H */
