#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

#include "ina219.h"
#include "Key.h"

#include "Car_control.h"
#include "Encoder.h"
#include "odometer.h"
#include "path_runner.h"
#include "path_script.h"

#include "Host.h"
#include "CommParser.h"

#include "Servo.h"

/* OLED.h 已随旧显示层一并移除 —— 显示现在走 u8g2（见 u8g2_port.h / ui_form.h）。
 * 注意：引脚宏 OLED_SCL_Pin / OLED_SDA_Pin 来自 CubeMX 的 main.h，与上面那个头无关，仍在使用。 */

/* ====================  ==================== */

#define BUS_SERVO_001_RESERVE 0
#define BUS_SERVO_002_RESERVE 0
#define BUS_SERVO_003_RESERVE 0

/* EASL_SetServo_Pos() 已于 2026-10-14 删除（死代码，语义与新零位偏移冲突）。 */

void Control_Init (void);

#endif
