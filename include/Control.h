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

#include "OLED.h"

/* ====================  ==================== */

#define BUS_SERVO_001_RESERVE 0
#define BUS_SERVO_002_RESERVE 0
#define BUS_SERVO_003_RESERVE 0

void EASL_SetServo_Pos(uint16_t addr, int16_t Pos, uint16_t time);



void Control_Init (void);

#endif
