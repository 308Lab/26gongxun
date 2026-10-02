#include "Control.h"

void Control_Init (void)
{
    INA219_Init();        /* INA219 电流监测（PC2=SCL, PC3=SDA） */
    
    Car_Control_Init();
    Host_Init();
    OLED_Init();
    Encoder_Init();
    Gyro_Init(&huart2);   /* JY61P 接 USART2（PA3=RX） */
    Servo_UartInit();
}

void EASL_SetServo_Pos(uint16_t addr, int16_t Pos, uint16_t time)
{
    /* 重写后：总线舵机直接按 PWM 值控制（1500 为中值），无需切模式 */
    int32_t pwm = 1500 + Pos;
    if (pwm > 2500) pwm = 2500;
    if (pwm < 500)  pwm = 500;
    Servo_SetAngleTime((uint8_t)addr, (uint16_t)pwm, time);
}